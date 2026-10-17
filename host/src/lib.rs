use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use io_proto::{
    read_u16, Frame, Parser, KIND_EVENT, KIND_REQ, KIND_RESP, MAX_BODY, MAX_FRAME, OP_ADC_READ,
    OP_BITSTREAM, OP_GPIO_READ, OP_GPIO_WRITE, OP_I2C_CONFIG, OP_I2C_TRANSFER, OP_PIN_CONFIG,
    OP_I2S_CONFIG, OP_I2S_READ, OP_PULSE_CAPTURE_START, OP_PULSE_CAPTURE_STOP, OP_PULSE_PLAY,
    OP_PWM_SET, OP_SPI_CONFIG,
    OP_SPI_TRANSFER, OP_UART_CONFIG, OP_UART_READ, OP_UART_WRITE, OP_VERSION, PIN_INPUT,
    PIN_OUTPUT, PULL_NONE, SPI_MISO_NONE, ST_OK, EVT_PULSE,
};
use serialport::SerialPort;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("serial: {0}")]
    Serial(#[from] serialport::Error),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("firmware status {0}")]
    Status(u8),
    #[error("response too short")]
    Short,
    #[error("timed out waiting for seq {0}")]
    Timeout(u16),
}

#[derive(Debug, Clone, Copy)]
pub struct Version {
    pub protocol: u16,
    pub firmware: u16,
    pub pins: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct PulseEdge {
    pub pin: u8,
    pub level: u8,
    pub width_us: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct BitTiming {
    pub t0h_ns: u16,
    pub t0l_ns: u16,
    pub t1h_ns: u16,
    pub t1l_ns: u16,
    pub reset_us: u16,
}

impl BitTiming {
    pub fn ws2812() -> Self {
        Self {
            t0h_ns: 400,
            t0l_ns: 850,
            t1h_ns: 800,
            t1l_ns: 450,
            reset_us: 80,
        }
    }
}

pub struct Board {
    port: Box<dyn SerialPort>,
    incoming: Receiver<Vec<u8>>,
    stop_read: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
    parser: Parser,
    seq: u16,
    pending: Vec<PulseEdge>,
    audio: VecDeque<Vec<u8>>,
}

impl Board {
    pub fn open(path: &str) -> Result<Self, Error> {
        let mut port = serialport::new(path, 115_200)
            .timeout(Duration::from_millis(30))
            .dtr_on_open(true)
            .open()?;
        let mut reader_port = port.try_clone()?;
        let _ = port.set_timeout(Duration::from_millis(30));
        let _ = reader_port.set_timeout(Duration::from_millis(30));
        let (tx, incoming) = mpsc::channel();
        let stop_read = Arc::new(AtomicBool::new(false));
        let stop = Arc::clone(&stop_read);
        let reader = thread::spawn(move || {
            let mut buf = [0u8; 1024];
            while !stop.load(Ordering::Relaxed) {
                match reader_port.read(&mut buf) {
                    Ok(0) => {
                        eprintln!("[board] 后台读线程读到 0 字节，退出");
                        break;
                    }
                    Ok(n) => {
                        if tx.send(buf[..n].to_vec()).is_err() {
                            eprintln!("[board] 后台读线程发送失败（接收端已关闭），退出");
                            break;
                        }
                    }
                    Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(err) => {
                        eprintln!("[board] 后台读线程读取出错，退出: {err}");
                        break;
                    }
                }
            }
        });
        Ok(Self {
            port,
            incoming,
            stop_read,
            reader: Some(reader),
            parser: Parser::new(),
            seq: 1,
            pending: Vec::new(),
            audio: VecDeque::new(),
        })
    }

    fn shutdown(&mut self) {
        self.stop_read.store(true, Ordering::Relaxed);
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }

    fn recv_chunk(&mut self, wait: Duration) -> Result<Option<Vec<u8>>, Error> {
        match self.incoming.recv_timeout(wait) {
            Ok(bytes) => Ok(Some(bytes)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(Error::Timeout(0)),
        }
    }

    pub fn version(&mut self) -> Result<Version, Error> {
        let body = self.call(OP_VERSION, &[])?;
        if body.len() < 5 {
            return Err(Error::Short);
        }
        Ok(Version {
            protocol: u16::from_le_bytes([body[0], body[1]]),
            firmware: u16::from_le_bytes([body[2], body[3]]),
            pins: body[4],
        })
    }

    pub fn pin_output(&mut self, pin: u8, high: bool) -> Result<(), Error> {
        self.call(
            OP_PIN_CONFIG,
            &[pin, PIN_OUTPUT, PULL_NONE, u8::from(high)],
        )?;
        Ok(())
    }

    pub fn pin_input(&mut self, pin: u8) -> Result<(), Error> {
        self.call(OP_PIN_CONFIG, &[pin, PIN_INPUT, PULL_NONE, 0])?;
        Ok(())
    }

    pub fn gpio_write(&mut self, pin: u8, high: bool) -> Result<(), Error> {
        self.call(OP_GPIO_WRITE, &[pin, u8::from(high)])?;
        Ok(())
    }

    pub fn gpio_read(&mut self, pin: u8) -> Result<bool, Error> {
        let body = self.call(OP_GPIO_READ, &[pin])?;
        Ok(body.first().copied().unwrap_or(0) != 0)
    }

    pub fn pwm_set(&mut self, pin: u8, freq_hz: u32, duty_permille: u16) -> Result<(), Error> {
        let mut req = [0u8; 7];
        req[0] = pin;
        req[1..5].copy_from_slice(&freq_hz.to_le_bytes());
        req[5..7].copy_from_slice(&duty_permille.to_le_bytes());
        self.call(OP_PWM_SET, &req)?;
        Ok(())
    }

    pub fn adc_read(&mut self, pin: u8) -> Result<u16, Error> {
        let body = self.call(OP_ADC_READ, &[pin])?;
        read_u16(&body, 0).ok_or(Error::Short)
    }

    pub fn i2c_config(&mut self, sda: u8, scl: u8, hz: u32) -> Result<(), Error> {
        let mut req = [0u8; 6];
        req[0] = sda;
        req[1] = scl;
        req[2..6].copy_from_slice(&hz.to_le_bytes());
        self.call(OP_I2C_CONFIG, &req)?;
        Ok(())
    }

    pub fn i2c_transfer(&mut self, bus: u8, addr: u8, write: &[u8], read_len: usize) -> Result<Vec<u8>, Error> {
        let mut req = vec![bus, addr, 0, 0, 0, 0];
        req[2..4].copy_from_slice(&(write.len() as u16).to_le_bytes());
        req[4..6].copy_from_slice(&(read_len as u16).to_le_bytes());
        req.extend_from_slice(write);
        self.call(OP_I2C_TRANSFER, &req)
    }

    pub fn spi_config(&mut self, sck: u8, mosi: u8, miso: Option<u8>, hz: u32, mode: u8) -> Result<(), Error> {
        let mut req = [0u8; 8];
        req[0] = sck;
        req[1] = mosi;
        req[2] = miso.unwrap_or(SPI_MISO_NONE);
        req[3..7].copy_from_slice(&hz.to_le_bytes());
        req[7] = mode;
        self.call(OP_SPI_CONFIG, &req)?;
        Ok(())
    }

    pub fn spi_transfer(&mut self, bus: u8, data: &[u8]) -> Result<Vec<u8>, Error> {
        let mut req = Vec::with_capacity(1 + data.len());
        req.push(bus);
        req.extend_from_slice(data);
        self.call(OP_SPI_TRANSFER, &req)
    }

    pub fn uart_config(&mut self, tx: u8, rx: u8, baud: u32) -> Result<(), Error> {
        let mut req = [0u8; 6];
        req[0] = tx;
        req[1] = rx;
        req[2..6].copy_from_slice(&baud.to_le_bytes());
        self.call(OP_UART_CONFIG, &req)?;
        Ok(())
    }

    pub fn uart_write(&mut self, bus: u8, data: &[u8]) -> Result<(), Error> {
        let mut req = Vec::with_capacity(1 + data.len());
        req.push(bus);
        req.extend_from_slice(data);
        self.call(OP_UART_WRITE, &req)?;
        Ok(())
    }

    pub fn uart_read(&mut self, bus: u8, max: u16) -> Result<Vec<u8>, Error> {
        let mut req = [0u8; 3];
        req[0] = bus;
        req[1..3].copy_from_slice(&max.to_le_bytes());
        self.call(OP_UART_READ, &req)
    }

    pub fn pulse_play(&mut self, pin: u8, pulses: &[(u8, u32)]) -> Result<(), Error> {
        for chunk in pulses.chunks(40) {
            let mut req = vec![pin, 0, 0];
            req[1..3].copy_from_slice(&(chunk.len() as u16).to_le_bytes());
            for (level, us) in chunk {
                req.push(*level);
                req.extend_from_slice(&us.to_le_bytes());
            }
            self.call(OP_PULSE_PLAY, &req)?;
        }
        Ok(())
    }

    pub fn pulse_capture_start(&mut self, pin: u8) -> Result<(), Error> {
        self.call(OP_PULSE_CAPTURE_START, &[pin])?;
        Ok(())
    }

    pub fn pulse_capture_stop(&mut self, pin: u8) -> Result<(), Error> {
        self.call(OP_PULSE_CAPTURE_STOP, &[pin])?;
        Ok(())
    }

    pub fn take_edges(&mut self) -> Vec<PulseEdge> {
        std::mem::take(&mut self.pending)
    }

    pub fn release(mut self) {
        self.shutdown();
    }

    pub fn i2s_config(&mut self, bclk: u8, ws: u8, data: u8, frame_hz: u32) -> Result<(), Error> {
        let mut req = [0u8; 7];
        req[0] = bclk;
        req[1] = ws;
        req[2] = data;
        req[3..7].copy_from_slice(&frame_hz.to_le_bytes());
        self.call(OP_I2S_CONFIG, &req)?;
        Ok(())
    }

    pub fn i2s_read(&mut self, max: u16) -> Result<Vec<u8>, Error> {
        self.call(OP_I2S_READ, &max.to_le_bytes())
    }

    pub fn read_i2s_push(&mut self) -> Result<Vec<u8>, Error> {
        if let Some(chunk) = self.audio.pop_front() {
            return Ok(chunk);
        }
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while std::time::Instant::now() < deadline {
            let Some(got) = self.recv_chunk(Duration::from_millis(50))? else {
                continue;
            };
            let mut audio = Vec::new();
            self.parser.push(&got, |frame| {
                if frame.kind == KIND_EVENT && frame.op == OP_I2S_READ && !frame.body.is_empty() {
                    audio.push(frame.body.to_vec());
                }
            });
            if !audio.is_empty() {
                let first = audio.remove(0);
                self.audio.extend(audio);
                return Ok(first);
            }
        }
        eprintln!("[board] read_i2s_push 2s 内未收到任何音频事件帧");
        Err(Error::Timeout(0))
    }

    pub fn bitstream(&mut self, pin: u8, timing: BitTiming, data: &[u8]) -> Result<(), Error> {
        for chunk in data.chunks(200) {
            let mut req = vec![pin, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            req[1..3].copy_from_slice(&timing.t0h_ns.to_le_bytes());
            req[3..5].copy_from_slice(&timing.t0l_ns.to_le_bytes());
            req[5..7].copy_from_slice(&timing.t1h_ns.to_le_bytes());
            req[7..9].copy_from_slice(&timing.t1l_ns.to_le_bytes());
            req[9..11].copy_from_slice(&timing.reset_us.to_le_bytes());
            req.extend_from_slice(chunk);
            self.call(OP_BITSTREAM, &req)?;
        }
        Ok(())
    }

    fn call(&mut self, op: u16, body: &[u8]) -> Result<Vec<u8>, Error> {
        if body.len() > MAX_BODY {
            return Err(Error::Status(io_proto::ST_NO_SPACE));
        }
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1).max(1);
        let mut raw = [0u8; MAX_FRAME];
        let n = io_proto::encode(
            Frame {
                kind: KIND_REQ,
                seq,
                op,
                status: 0,
                body,
            },
            &mut raw,
        )
        .expect("frame fits");
        let deadline = std::time::Instant::now() + Duration::from_secs(3);
        let mut off = 0;
        while std::time::Instant::now() < deadline {
            if off < n {
                match self.port.write(&raw[off..n]) {
                    Ok(0) => {}
                    Ok(k) => off += k,
                    Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(err) => return Err(err.into()),
                }
            }
            let Some(got) = self.recv_chunk(Duration::from_millis(20))? else {
                continue;
            };
            let mut found = None;
            let mut edges = Vec::new();
            let mut audio = Vec::new();
            self.parser.push(&got, |frame| {
                if frame.kind == KIND_EVENT && frame.op == EVT_PULSE && frame.body.len() >= 6 {
                    edges.push(PulseEdge {
                        pin: frame.body[0],
                        level: frame.body[1],
                        width_us: u32::from_le_bytes(frame.body[2..6].try_into().unwrap()),
                    });
                } else if frame.kind == KIND_EVENT && frame.op == OP_I2S_READ && !frame.body.is_empty() {
                    audio.push(frame.body.to_vec());
                } else if frame.kind == KIND_RESP && frame.seq == seq {
                    found = Some((frame.status, frame.body.to_vec()));
                }
            });
            self.pending.extend(edges);
            self.audio.extend(audio);
            if let Some((status, body)) = found {
                if status != ST_OK {
                    return Err(Error::Status(status));
                }
                return Ok(body);
            }
        }
        eprintln!("[board] call 请求 op={op} seq={seq} 3s 内未收到应答");
        Err(Error::Timeout(seq))
    }
}

impl Drop for Board {
    fn drop(&mut self) {
        self.shutdown();
    }
}
