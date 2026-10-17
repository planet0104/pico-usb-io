use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub use io_proto::{
    PIO_JOIN_NONE, PIO_JOIN_RX, PIO_JOIN_TX, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};
use io_proto::{
    read_u16, read_u32, Frame, Parser, KIND_EVENT, KIND_REQ, KIND_RESP, MAX_BODY, MAX_FRAME,
    OP_ADC_READ, OP_GPIO_READ, OP_GPIO_WRITE, OP_I2C_CONFIG, OP_I2C_TRANSFER, OP_PIN_CONFIG,
    OP_PIO_LOAD, OP_PIO_RX_POP, OP_PIO_SM_CONFIG, OP_PIO_SM_ENABLE, OP_PIO_SM_RESTART,
    OP_PIO_STREAM_EVENT, OP_PIO_STREAM_START, OP_PIO_STREAM_STATUS, OP_PIO_STREAM_STOP,
    OP_PIO_TX_PUSH, OP_PIO_UNLOAD, OP_PWM_SET, OP_SPI_CONFIG, OP_SPI_TRANSFER, OP_UART_CONFIG,
    OP_UART_READ, OP_UART_WRITE, OP_VERSION, PIN_INPUT, PIN_OUTPUT, PIO_AUTOPULL, PIO_AUTOPUSH,
    PIO_IN_SHIFT_RIGHT, PIO_LOAD_AUTO, PIO_NO_PIN, PIO_OUT_SHIFT_RIGHT, PIO_SIDESET_OPT,
    PIO_SIDESET_PINDIRS, PIO_SM_CONFIG_LEN, PIO_STREAM_SPARSE, PIO_TX_WAIT, PULL_NONE,
    SPI_MISO_NONE, ST_BUSY, ST_OK,
};
use serialport::SerialPort;

pub const SYS_CLK_HZ: u32 = 125_000_000;

pub type Program = pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>;

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
pub struct PioProgram {
    pub pio: u8,
    pub origin: u8,
    pub count: u8,
    side_bits: u8,
    side_opt: bool,
    side_pindirs: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct SmConfig {
    pub clkdiv: (u16, u8),
    pub out_pins: (u8, u8),
    pub set_pins: (u8, u8),
    pub in_pins: (u8, u8),
    pub sideset_base: u8,
    pub out_shift_right: bool,
    pub in_shift_right: bool,
    pub autopull: bool,
    pub autopush: bool,
    pub pull_thresh: u8,
    pub push_thresh: u8,
    pub fifo_join: u8,
    pub jmp_pin: Option<u8>,
    pub out_pindirs: u32,
}

impl Default for SmConfig {
    fn default() -> Self {
        Self {
            clkdiv: (1, 0),
            out_pins: (0, 0),
            set_pins: (0, 0),
            in_pins: (0, 0),
            sideset_base: 0,
            out_shift_right: false,
            in_shift_right: false,
            autopull: false,
            autopush: false,
            pull_thresh: 32,
            push_thresh: 32,
            fifo_join: PIO_JOIN_NONE,
            jmp_pin: None,
            out_pindirs: 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StreamStatus {
    pub active: bool,
    pub ring_used: u32,
    pub dropped_words: u32,
}

pub fn clkdiv(freq_hz: u32) -> (u16, u8) {
    let int = SYS_CLK_HZ / freq_hz;
    let frac = (((SYS_CLK_HZ % freq_hz) as u64) << 8) / freq_hz as u64;
    (int as u16, frac as u8)
}

pub struct Board {
    port: Box<dyn SerialPort>,
    incoming: Receiver<Vec<u8>>,
    stop_read: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
    parser: Parser,
    seq: u16,
    stream: VecDeque<Vec<u32>>,
}

impl Board {
    pub fn open(path: &str) -> Result<Self, Error> {
        let port = serialport::new(path, 115_200)
            .timeout(Duration::from_millis(30))
            .dtr_on_open(true)
            .open()?;
        let mut reader_port = port.try_clone()?;
        let (tx, incoming) = mpsc::channel();
        let stop_read = Arc::new(AtomicBool::new(false));
        let stop = Arc::clone(&stop_read);
        let reader = thread::spawn(move || {
            let mut buf = [0u8; 1024];
            while !stop.load(Ordering::Relaxed) {
                match reader_port.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        if tx.send(buf[..n].to_vec()).is_err() {
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
            stream: VecDeque::new(),
        })
    }

    pub fn release(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop_read.store(true, Ordering::Relaxed);
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
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
        self.call(OP_PIN_CONFIG, &[pin, PIN_OUTPUT, PULL_NONE, u8::from(high)])?;
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

    pub fn pio_load(&mut self, pio: u8, program: &Program) -> Result<PioProgram, Error> {
        let (flags, origin) = match program.origin {
            Some(origin) => (0, origin),
            None => (PIO_LOAD_AUTO, 0),
        };
        let mut req = vec![
            pio,
            flags,
            origin,
            program.wrap.target,
            program.wrap.source,
            program.code.len() as u8,
        ];
        for instr in &program.code {
            req.extend_from_slice(&instr.to_le_bytes());
        }
        let body = self.call(OP_PIO_LOAD, &req)?;
        if body.len() < 2 {
            return Err(Error::Short);
        }
        Ok(PioProgram {
            pio,
            origin: body[0],
            count: body[1],
            side_bits: program.side_set.bits(),
            side_opt: program.side_set.optional(),
            side_pindirs: program.side_set.pindirs(),
        })
    }

    pub fn pio_unload(&mut self, prog: PioProgram) -> Result<(), Error> {
        self.call(OP_PIO_UNLOAD, &[prog.pio, prog.origin, prog.count])?;
        Ok(())
    }

    pub fn pio_sm_config(&mut self, sm: u8, prog: &PioProgram, cfg: &SmConfig) -> Result<(), Error> {
        let mut flags = 0;
        for (on, bit) in [
            (prog.side_opt, PIO_SIDESET_OPT),
            (prog.side_pindirs, PIO_SIDESET_PINDIRS),
            (cfg.out_shift_right, PIO_OUT_SHIFT_RIGHT),
            (cfg.in_shift_right, PIO_IN_SHIFT_RIGHT),
            (cfg.autopull, PIO_AUTOPULL),
            (cfg.autopush, PIO_AUTOPUSH),
        ] {
            if on {
                flags |= bit;
            }
        }
        let mut req = [0u8; PIO_SM_CONFIG_LEN];
        req[0] = prog.pio;
        req[1] = sm;
        req[2] = prog.origin;
        req[3..5].copy_from_slice(&cfg.clkdiv.0.to_le_bytes());
        req[5] = cfg.clkdiv.1;
        req[6] = cfg.out_pins.0;
        req[7] = cfg.out_pins.1;
        req[8] = cfg.set_pins.0;
        req[9] = cfg.set_pins.1;
        req[10] = cfg.in_pins.0;
        req[11] = cfg.in_pins.1;
        req[12] = cfg.sideset_base;
        req[13] = prog.side_bits;
        req[14] = flags;
        req[15] = cfg.pull_thresh;
        req[16] = cfg.push_thresh;
        req[17] = cfg.fifo_join;
        req[18] = cfg.jmp_pin.unwrap_or(PIO_NO_PIN);
        req[20..24].copy_from_slice(&cfg.out_pindirs.to_le_bytes());
        self.call(OP_PIO_SM_CONFIG, &req)?;
        Ok(())
    }

    pub fn pio_sm_enable(&mut self, pio: u8, sm: u8, on: bool) -> Result<(), Error> {
        self.call(OP_PIO_SM_ENABLE, &[pio, sm, u8::from(on)])?;
        Ok(())
    }

    pub fn pio_sm_restart(&mut self, pio: u8, sm: u8, flags: u8) -> Result<(), Error> {
        self.call(OP_PIO_SM_RESTART, &[pio, sm, flags])?;
        Ok(())
    }

    pub fn pio_tx_push(&mut self, pio: u8, sm: u8, words: &[u32]) -> Result<(), Error> {
        let mut sent = 0;
        while sent < words.len() {
            let chunk = &words[sent..(sent + 255).min(words.len())];
            let mut req = vec![pio, sm, PIO_TX_WAIT, chunk.len() as u8];
            for word in chunk {
                req.extend_from_slice(&word.to_le_bytes());
            }
            let body = self.call(OP_PIO_TX_PUSH, &req)?;
            let pushed = *body.first().ok_or(Error::Short)? as usize;
            if pushed == 0 {
                return Err(Error::Status(ST_BUSY));
            }
            sent += pushed;
        }
        Ok(())
    }

    pub fn pio_rx_pop(&mut self, pio: u8, sm: u8, max: u8) -> Result<Vec<u32>, Error> {
        let body = self.call(OP_PIO_RX_POP, &[pio, sm, max])?;
        Ok(words(&body))
    }

    pub fn pio_stream_start(&mut self, pio: u8, sm: u8, sparse: bool, chunk_words: u8) -> Result<(), Error> {
        self.stream.clear();
        let flags = if sparse { PIO_STREAM_SPARSE } else { 0 };
        self.call(OP_PIO_STREAM_START, &[pio, sm, flags, chunk_words])?;
        Ok(())
    }

    pub fn pio_stream_stop(&mut self, pio: u8, sm: u8) -> Result<(), Error> {
        self.call(OP_PIO_STREAM_STOP, &[pio, sm])?;
        self.stream.clear();
        Ok(())
    }

    pub fn pio_stream_status(&mut self) -> Result<StreamStatus, Error> {
        let body = self.call(OP_PIO_STREAM_STATUS, &[])?;
        Ok(StreamStatus {
            active: *body.first().ok_or(Error::Short)? != 0,
            ring_used: read_u32(&body, 1).ok_or(Error::Short)?,
            dropped_words: read_u32(&body, 5).ok_or(Error::Short)?,
        })
    }

    pub fn read_stream(&mut self, wait: Duration) -> Result<Vec<u32>, Error> {
        let deadline = Instant::now() + wait;
        loop {
            if let Some(chunk) = self.stream.pop_front() {
                return Ok(chunk);
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(Error::Timeout(0));
            }
            let Some(got) = self.recv_chunk(left.min(Duration::from_millis(50)))? else {
                continue;
            };
            let mut chunks = Vec::new();
            self.parser.push(&got, |frame| {
                if frame.kind == KIND_EVENT && frame.op == OP_PIO_STREAM_EVENT && !frame.body.is_empty() {
                    chunks.push(words(frame.body));
                }
            });
            self.stream.extend(chunks);
        }
    }

    fn recv_chunk(&mut self, wait: Duration) -> Result<Option<Vec<u8>>, Error> {
        match self.incoming.recv_timeout(wait) {
            Ok(bytes) => Ok(Some(bytes)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(Error::Timeout(0)),
        }
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
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut off = 0;
        while Instant::now() < deadline {
            if off < n {
                match self.port.write(&raw[off..n]) {
                    Ok(k) => off += k,
                    Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(err) => return Err(err.into()),
                }
            }
            let Some(got) = self.recv_chunk(Duration::from_millis(20))? else {
                continue;
            };
            let mut found = None;
            let mut chunks = Vec::new();
            self.parser.push(&got, |frame| {
                if frame.kind == KIND_EVENT && frame.op == OP_PIO_STREAM_EVENT && !frame.body.is_empty() {
                    chunks.push(words(frame.body));
                } else if frame.kind == KIND_RESP && frame.seq == seq {
                    found = Some((frame.status, frame.body.to_vec()));
                }
            });
            self.stream.extend(chunks);
            if let Some((status, body)) = found {
                if status != ST_OK {
                    return Err(Error::Status(status));
                }
                return Ok(body);
            }
        }
        Err(Error::Timeout(seq))
    }
}

impl Drop for Board {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn words(bytes: &[u8]) -> Vec<u32> {
    bytes
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
        .collect()
}
