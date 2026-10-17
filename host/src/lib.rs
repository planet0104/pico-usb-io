//! 上位机库：通过 USB 串口控制 pico-usb-io 固件。
//!
//! `Board` 封装了串口收发和帧协议，每个方法对应固件的一条命令，调用后等待应答再返回。
//! 基础 IO（GPIO / PWM / ADC / I2C / SPI / UART）直接调用即可；
//! 需要精确时序的功能用 `pio_*` 方法：自己写 PIO 程序（`pio::pio_asm!`）、装载、配置状态机、
//! 收发数据。具体用法可参考 record.rs（I2S 录音）、ws2812.rs、servo.rs、ir.rs。

use std::collections::VecDeque;
use std::io::{Read, Write};
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

/// 固件的系统时钟（embassy 默认配置），PIO 分频按它计算。
pub const SYS_CLK_HZ: u32 = 125_000_000;

/// `pio::pio_asm!` 汇编出来的程序类型（最多 32 条指令）。
pub type Program = pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("serial: {0}")]
    Serial(#[from] serialport::Error),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    /// 固件应答里的错误码，见 protocol 里的 ST_* 常量。
    #[error("firmware status {0}")]
    Status(u8),
    #[error("response too short")]
    Short,
    /// 规定时间内没收到应答（seq 为 0 表示等流数据超时）。
    #[error("timed out waiting for seq {0}")]
    Timeout(u16),
}

#[derive(Debug, Clone, Copy)]
pub struct Version {
    pub protocol: u16,
    pub firmware: u16,
    pub pins: u8,
}

/// 已装载到板子上的 PIO 程序。卸载、配置状态机时要用到。
#[derive(Debug, Clone, Copy)]
pub struct PioProgram {
    /// 装在哪个 PIO 块（0 或 1）。
    pub pio: u8,
    /// 实际装载起始地址。
    pub origin: u8,
    /// 指令条数。
    pub count: u8,
    // 程序的 side-set 设置，来自 `.side_set` 指令，配置状态机时自动带上。
    side_bits: u8,
    side_opt: bool,
    side_pindirs: bool,
}

/// 状态机配置，字段与固件 OP_PIO_SM_CONFIG 一一对应。引脚区间都写成 (起始引脚, 个数)。
#[derive(Debug, Clone, Copy)]
pub struct SmConfig {
    /// 分频 (整数, 小数/256)，一般用 `clkdiv(目标频率)` 计算。
    pub clkdiv: (u16, u8),
    /// `out pins` 写的引脚。
    pub out_pins: (u8, u8),
    /// `set pins` 写的引脚（最多 5 个）。
    pub set_pins: (u8, u8),
    /// `in pins` / `wait pin` 读的引脚。个数只用于固件的引脚占用登记。
    pub in_pins: (u8, u8),
    /// side-set 的第一个引脚，个数由程序的 `.side_set` 决定。
    pub sideset_base: u8,
    /// OUT 先移出低位（true）还是高位（false）。
    pub out_shift_right: bool,
    /// IN 新数据从高位移入（true）还是低位（false）。
    pub in_shift_right: bool,
    /// OSR 用完 `pull_thresh` 位后自动从 TX FIFO 取下一个字。
    pub autopull: bool,
    /// ISR 攒够 `push_thresh` 位后自动推进 RX FIFO。
    pub autopush: bool,
    pub pull_thresh: u8,
    pub push_thresh: u8,
    /// PIO_JOIN_NONE / PIO_JOIN_TX / PIO_JOIN_RX：单向使用时合并成 8 级 FIFO。
    pub fifo_join: u8,
    /// `jmp pin` 判断的引脚。
    pub jmp_pin: Option<u8>,
    /// 第 n 位为 1 表示 GPIO n 设为输出；只能包含上面用到的引脚。
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

/// 流状态。`dropped_words` 不为 0 说明板上缓冲满过、丢了数据（上位机读得太慢）。
#[derive(Debug, Clone, Copy)]
pub struct StreamStatus {
    pub active: bool,
    pub ring_used: u32,
    pub dropped_words: u32,
}

/// 计算让 PIO 跑在 `freq_hz` 的分频值，结果与固件 embassy 的 `calculate_pio_clock_divider` 相同。
/// 例如 48 kHz I2S 需要 48000 × 128 Hz，得 (20, 88)。
pub fn clkdiv(freq_hz: u32) -> (u16, u8) {
    let int = SYS_CLK_HZ / freq_hz;
    let frac = (((SYS_CLK_HZ % freq_hz) as u64) << 8) / freq_hz as u64;
    (int as u16, frac as u8)
}

/// 串口单次读 / 写的超时。读到数据会立即返回，这个值只决定"没数据时最多等多久"。
const IO_TIMEOUT: Duration = Duration::from_millis(5);

/// 一块已连接的板子。
///
/// 读写都在调用者的线程里按顺序进行：先把请求写完，再读应答。不能另开线程一直挂着读：
/// Windows 上串口是同步句柄，同一个句柄（包括 try_clone 复制出来的）上的读和写会被系统排队，
/// 后台线程的读没超时之前，主线程的写就发不出去，每条命令要多等几十毫秒。
/// 固件用独立任务收命令，即使上位机暂时不读，板子也能收下请求，不会互相等待。
pub struct Board {
    port: Box<dyn SerialPort>,
    parser: Parser,
    seq: u16,
    /// 已收到但还没被 `read_stream` 取走的流数据。
    stream: VecDeque<Vec<u32>>,
}

impl Board {
    /// 打开串口（Windows 上如 "COM5"，Linux 上如 "/dev/ttyACM0"）。
    /// 打开时置 DTR，固件据此判断上位机已连接；关闭后 DTR 消失，固件会自动停掉 PIO。
    pub fn open(path: &str) -> Result<Self, Error> {
        let port = serialport::new(path, 115_200)
            .timeout(IO_TIMEOUT)
            .dtr_on_open(true)
            .open()?;
        Ok(Self {
            port,
            parser: Parser::new(),
            seq: 1,
            stream: VecDeque::new(),
        })
    }

    /// 主动关闭串口。直接丢弃 `Board` 也会关闭。
    pub fn release(self) {}

    /// 查询协议版本、固件版本和引脚数，也可以用来确认连上的是不是这块板子。
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

    /// 把引脚设为输出并给定电平（会释放它原来的功能）。
    pub fn pin_output(&mut self, pin: u8, high: bool) -> Result<(), Error> {
        self.call(OP_PIN_CONFIG, &[pin, PIN_OUTPUT, PULL_NONE, u8::from(high)])?;
        Ok(())
    }

    /// 把引脚设为无上下拉的输入。
    pub fn pin_input(&mut self, pin: u8) -> Result<(), Error> {
        self.call(OP_PIN_CONFIG, &[pin, PIN_INPUT, PULL_NONE, 0])?;
        Ok(())
    }

    /// 写电平；引脚还不是 GPIO 时固件会自动切成输出。
    pub fn gpio_write(&mut self, pin: u8, high: bool) -> Result<(), Error> {
        self.call(OP_GPIO_WRITE, &[pin, u8::from(high)])?;
        Ok(())
    }

    /// 读电平，不改变引脚当前功能。
    pub fn gpio_read(&mut self, pin: u8) -> Result<bool, Error> {
        let body = self.call(OP_GPIO_READ, &[pin])?;
        Ok(body.first().copied().unwrap_or(0) != 0)
    }

    /// 硬件 PWM：频率 Hz，占空比为千分比（0..1000）。
    /// 频率为 0 时把占空比清零，引脚已经在输出 PWM 时即变为恒低。
    pub fn pwm_set(&mut self, pin: u8, freq_hz: u32, duty_permille: u16) -> Result<(), Error> {
        let mut req = [0u8; 7];
        req[0] = pin;
        req[1..5].copy_from_slice(&freq_hz.to_le_bytes());
        req[5..7].copy_from_slice(&duty_permille.to_le_bytes());
        self.call(OP_PWM_SET, &req)?;
        Ok(())
    }

    /// 读 ADC（只支持 GP26..GP29），返回 12 位原始值，0..4095 对应 0..3.3 V。
    pub fn adc_read(&mut self, pin: u8) -> Result<u16, Error> {
        let body = self.call(OP_ADC_READ, &[pin])?;
        read_u16(&body, 0).ok_or(Error::Short)
    }

    /// 打开 I2C 主机。SDA 必须是偶数脚、SCL = SDA + 1；总线号由引脚决定
    /// （SDA % 4 == 0 是总线 0，== 2 是总线 1），之后 `i2c_transfer` 用这个总线号。
    pub fn i2c_config(&mut self, sda: u8, scl: u8, hz: u32) -> Result<(), Error> {
        let mut req = [0u8; 6];
        req[0] = sda;
        req[1] = scl;
        req[2..6].copy_from_slice(&hz.to_le_bytes());
        self.call(OP_I2C_CONFIG, &req)?;
        Ok(())
    }

    /// I2C 先写 `write` 再读 `read_len` 字节（任一可为空），`addr` 是 7 位地址。
    pub fn i2c_transfer(&mut self, bus: u8, addr: u8, write: &[u8], read_len: usize) -> Result<Vec<u8>, Error> {
        let mut req = vec![bus, addr, 0, 0, 0, 0];
        req[2..4].copy_from_slice(&(write.len() as u16).to_le_bytes());
        req[4..6].copy_from_slice(&(read_len as u16).to_le_bytes());
        req.extend_from_slice(write);
        self.call(OP_I2C_TRANSFER, &req)
    }

    /// 打开 SPI 主机，`mode` 是标准 SPI 模式 0..3，`miso` 为 None 时只发不收。
    /// 片选请自己用 `gpio_write` 控制。
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

    /// SPI 全双工收发，返回与发送等长的接收数据。
    pub fn spi_transfer(&mut self, bus: u8, data: &[u8]) -> Result<Vec<u8>, Error> {
        let mut req = Vec::with_capacity(1 + data.len());
        req.push(bus);
        req.extend_from_slice(data);
        self.call(OP_SPI_TRANSFER, &req)
    }

    /// 打开 UART（8N1），UART 编号由 TX 引脚决定。
    pub fn uart_config(&mut self, tx: u8, rx: u8, baud: u32) -> Result<(), Error> {
        let mut req = [0u8; 6];
        req[0] = tx;
        req[1] = rx;
        req[2..6].copy_from_slice(&baud.to_le_bytes());
        self.call(OP_UART_CONFIG, &req)?;
        Ok(())
    }

    /// UART 发送。
    pub fn uart_write(&mut self, bus: u8, data: &[u8]) -> Result<(), Error> {
        let mut req = Vec::with_capacity(1 + data.len());
        req.push(bus);
        req.extend_from_slice(data);
        self.call(OP_UART_WRITE, &req)?;
        Ok(())
    }

    /// UART 读取已收到的数据（不等待，可能为空）。硬件 FIFO 只有 32 字节，需要定期轮询。
    pub fn uart_read(&mut self, bus: u8, max: u16) -> Result<Vec<u8>, Error> {
        let mut req = [0u8; 3];
        req[0] = bus;
        req[1..3].copy_from_slice(&max.to_le_bytes());
        self.call(OP_UART_READ, &req)
    }

    /// 把 PIO 程序装到 PIO 块 `pio`（0 或 1）。程序里没写 `.origin` 时由固件自动选地址。
    /// 每块共 32 条指令，用完记得 `pio_unload`。
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

    /// 卸载程序。还有状态机在运行这段程序时会返回 ST_BUSY，先 `pio_sm_enable(.., false)`。
    pub fn pio_unload(&mut self, prog: PioProgram) -> Result<(), Error> {
        self.call(OP_PIO_UNLOAD, &[prog.pio, prog.origin, prog.count])?;
        Ok(())
    }

    /// 让 `prog` 所在 PIO 块的状态机 `sm`（0..3）运行这段程序，并按 `cfg` 配置。
    /// 配置后状态机处于停用状态，需要 `pio_sm_enable` 才开始运行。
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
        // 按固件约定的 24 字节布局填写，第 19 字节保留为 0。
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

    /// 使能 / 停用状态机。停用后引脚保持最后的电平。
    pub fn pio_sm_enable(&mut self, pio: u8, sm: u8, on: bool) -> Result<(), Error> {
        self.call(OP_PIO_SM_ENABLE, &[pio, sm, u8::from(on)])?;
        Ok(())
    }

    /// 复位状态机，`flags` 用 PIO_RESTART_CLKDIV / PIO_RESTART_FIFOS / PIO_RESTART_SM 组合。
    /// 一般在配置之后、使能之前三个都做一次，保证从干净的状态开始。
    pub fn pio_sm_restart(&mut self, pio: u8, sm: u8, flags: u8) -> Result<(), Error> {
        self.call(OP_PIO_SM_RESTART, &[pio, sm, flags])?;
        Ok(())
    }

    /// 往状态机的 TX FIFO 写数据，自动按每帧 255 个字拆分。
    /// FIFO 满时固件会等它腾出空间（每帧最多 50 ms）；一个字都写不进去时返回 ST_BUSY，
    /// 通常说明状态机没在运行。
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

    /// 一次性读取 RX FIFO 里现有的数据（最多 `max` 个字），适合偶尔读一下的场合；
    /// 持续接收请用流。
    pub fn pio_rx_pop(&mut self, pio: u8, sm: u8, max: u8) -> Result<Vec<u32>, Error> {
        let body = self.call(OP_PIO_RX_POP, &[pio, sm, max])?;
        Ok(words(&body))
    }

    /// 开始接收状态机的 RX 数据流，之后用 `read_stream` 取数据。
    /// `sparse = false` 是连续模式（DMA，适合 I2S 等高速数据）；`true` 是稀疏模式（适合红外等偶发数据）。
    /// `chunk_words` 是每帧最多带多少个字（0 表示 255）。请在 `pio_sm_enable` 之前调用。
    pub fn pio_stream_start(&mut self, pio: u8, sm: u8, sparse: bool, chunk_words: u8) -> Result<(), Error> {
        self.stream.clear();
        let flags = if sparse { PIO_STREAM_SPARSE } else { 0 };
        self.call(OP_PIO_STREAM_START, &[pio, sm, flags, chunk_words])?;
        Ok(())
    }

    /// 停止流（固件会同时停用该状态机），并丢掉还没读走的数据。
    pub fn pio_stream_stop(&mut self, pio: u8, sm: u8) -> Result<(), Error> {
        self.call(OP_PIO_STREAM_STOP, &[pio, sm])?;
        self.stream.clear();
        Ok(())
    }

    /// 查询流状态，主要用来确认有没有丢数据。
    pub fn pio_stream_status(&mut self) -> Result<StreamStatus, Error> {
        let body = self.call(OP_PIO_STREAM_STATUS, &[])?;
        Ok(StreamStatus {
            active: *body.first().ok_or(Error::Short)? != 0,
            ring_used: read_u32(&body, 1).ok_or(Error::Short)?,
            dropped_words: read_u32(&body, 5).ok_or(Error::Short)?,
        })
    }

    /// 取下一块流数据（一个事件帧的内容），最多等 `wait`，超时返回 `Error::Timeout(0)`。
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
            if let Some(got) = self.recv_chunk(left)? {
                self.absorb(&got, 0);
            }
        }
    }

    /// 读一块原始字节，有数据立即返回；`wait` 内一直没有数据返回 None。
    fn recv_chunk(&mut self, wait: Duration) -> Result<Option<Vec<u8>>, Error> {
        let deadline = Instant::now() + wait;
        let mut buf = [0u8; 4096];
        loop {
            match self.port.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => return Ok(Some(buf[..n].to_vec())),
                Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
                Err(err) => return Err(err.into()),
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
        }
    }

    /// 解析收到的字节：流数据放进队列；遇到序号为 `seq` 的应答就返回 (status, body)。
    fn absorb(&mut self, bytes: &[u8], seq: u16) -> Option<(u8, Vec<u8>)> {
        let mut found = None;
        let mut chunks = Vec::new();
        self.parser.push(bytes, |frame| {
            if frame.kind == KIND_EVENT && frame.op == OP_PIO_STREAM_EVENT && !frame.body.is_empty() {
                chunks.push(words(frame.body));
            } else if frame.kind == KIND_RESP && frame.seq == seq {
                found = Some((frame.status, frame.body.to_vec()));
            }
        });
        self.stream.extend(chunks);
        found
    }

    /// 发一条请求并等待对应 seq 的应答（最多 3 秒），返回应答 body。
    /// 等待期间收到的流数据会放进队列，不会丢。
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
        while off < n {
            match self.port.write(&raw[off..n]) {
                Ok(k) => off += k,
                Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
                Err(err) => return Err(err.into()),
            }
            if off < n {
                if Instant::now() >= deadline {
                    return Err(Error::Timeout(seq));
                }
                // 板子暂时收不下（例如正在做耗时的 I2C），顺便把已到的流数据读走。
                if let Some(got) = self.recv_chunk(Duration::ZERO)? {
                    self.absorb(&got, 0);
                }
            }
        }
        while Instant::now() < deadline {
            let Some(got) = self.recv_chunk(Duration::from_millis(20))? else {
                continue;
            };
            if let Some((status, body)) = self.absorb(&got, seq) {
                if status != ST_OK {
                    return Err(Error::Status(status));
                }
                return Ok(body);
            }
        }
        Err(Error::Timeout(seq))
    }
}

/// 把小端字节序列转成 u32 序列，末尾不足 4 字节的部分丢弃。
fn words(bytes: &[u8]) -> Vec<u32> {
    bytes
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
        .collect()
}
