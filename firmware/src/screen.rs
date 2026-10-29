//! 往 SPI 屏写大块数据：原始像素（OP_SPI_WRITE）、LZ4 压缩的像素（OP_SPI_WRITE_LZ4）、JPEG（OP_JPEG）。
//!
//! 数据都交给 DMA 发送，发出去就返回，这样发 SPI 的同时能接收下一块 USB 数据；
//! 下一条命令执行前由 `Screen::finish` 等上一次发完（尤其是要切换屏幕 DC 脚的时候）。
//!
//! JPEG 可以交给第二个核解码（JPEG_CORE1），两个核同时各解一张。两个核共用一条 SPI 和一个 DMA 通道，
//! 设窗口、发条带都要先拿 `with_spi` 的锁；同一时刻只有一次 DMA 在发。

use core::cell::Cell;

use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Instant;
use io_proto::{
    JPEG_CORE1, JPEG_FIRST, JPEG_LAST, JPEG_MAX, SPI_LZ4_MAX, ST_BAD_ARG, ST_BAD_PIN, ST_IO, ST_NO_SPACE,
};
use tjpgdec_rs::{JpegDecoder, MemoryPool, Rectangle, RECOMMENDED_POOL_SIZE};

use crate::pins;

/// SPI 发送用的 DMA 通道。0、1 归 PIO 流引擎。
const DMA_CH: usize = 2;
/// JPEG 解码时一条条带最宽多少像素（ST7789 最宽 320）、最高多少行（4:2:0 的 MCU 是 16 行）。
const STRIP_WIDTH: usize = 320;
const STRIP_ROWS: usize = 16;
const STRIP_BYTES: usize = STRIP_WIDTH * STRIP_ROWS * 2;

/// 可以用 DMA 发送的 SPI 总线。由 main.rs 根据总线是否已经打开传进来。
#[derive(Clone, Copy)]
pub struct Bus {
    spi: pac::spi::Spi,
    treq: TreqSel,
}

impl Bus {
    pub fn new(bus: u8) -> Result<Self, u8> {
        match bus {
            0 => Ok(Self { spi: pac::SPI0, treq: TreqSel::SPI0_TX }),
            1 => Ok(Self { spi: pac::SPI1, treq: TreqSel::SPI1_TX }),
            _ => Err(ST_BAD_ARG),
        }
    }
}

/// 一张 JPEG：画在哪、收了多少字节、交给哪个核解码。
#[derive(Clone, Copy)]
struct Jpeg {
    bus: u8,
    dc: u8,
    x: u16,
    y: u16,
    len: usize,
    core: usize,
}

type Shared<T> = Mutex<CriticalSectionRawMutex, Cell<T>>;

/// SPI 和 DMA 有没有核在用。
static SPI_LOCKED: Shared<bool> = Mutex::new(Cell::new(false));
/// 交给 DMA 后还没确认发完的总线（拿着 SPI 锁才能动）。
static DMA_BUSY: Shared<Option<Bus>> = Mutex::new(Cell::new(None));
/// 交给第二个核的 JPEG；第二个核取走后变回 None。
static CORE1_JOB: Shared<Option<Jpeg>> = Mutex::new(Cell::new(None));
/// 第二个核从接活到画完都是 true。
static CORE1_BUSY: Shared<bool> = Mutex::new(Cell::new(false));
/// 第二个核上一张的解码显示用时（微秒），出错时为 0。
static CORE1_US: Shared<u32> = Mutex::new(Cell::new(0));

/// 第二个核的主循环：等 JPEG，解码显示。
pub fn core1_main() -> ! {
    loop {
        match CORE1_JOB.lock(|job| job.take()) {
            Some(jpeg) => {
                let us = show_jpeg(&jpeg).unwrap_or(0);
                CORE1_US.lock(|c| c.set(us));
                CORE1_BUSY.lock(|c| c.set(false));
            }
            None => cortex_m::asm::wfe(),
        }
    }
}

/// 只在第一个核上用：OP_JPEG 收数据，以及其他写屏命令。
pub struct Screen {
    jpeg: Option<Jpeg>,
}

impl Screen {
    pub const fn new() -> Self {
        Self { jpeg: None }
    }

    /// 等第二个核画完、DMA 发完。除 OP_JPEG 以外的命令执行前都要调用，免得和正在画的图抢屏。
    pub fn finish(&mut self) {
        wait_core1();
        with_spi(dma_finish);
    }

    /// OP_SPI_WRITE：数据拷进 DMA 缓冲区发送。
    pub fn write(&mut self, bus: Bus, data: &[u8]) -> Result<usize, u8> {
        let buf = dma_buf();
        buf[..data.len()].copy_from_slice(data);
        with_spi(|| start(bus, &buf[..data.len()]));
        Ok(0)
    }

    /// OP_SPI_WRITE_LZ4：解压到 DMA 缓冲区再发送。
    pub fn write_lz4(&mut self, bus: Bus, data: &[u8]) -> Result<usize, u8> {
        let buf = dma_buf();
        let n = lz4_flex::block::decompress_into(data, buf).map_err(|_| ST_BAD_ARG)?;
        with_spi(|| start(bus, &buf[..n]));
        Ok(0)
    }

    /// OP_JPEG：收一帧；收到最后一帧时解码显示（或交给第二个核），应答用时（微秒）。
    /// `open` 判断这条总线是否已经用 OP_SPI_CONFIG 打开，`dc_ok` 判断 DC 脚是否已经设成 GPIO 输出。
    pub fn jpeg(
        &mut self,
        req: &[u8],
        out: &mut [u8],
        open: impl Fn(u8) -> bool,
        dc_ok: impl Fn(u8) -> bool,
    ) -> Result<usize, u8> {
        let flags = *req.first().ok_or(ST_BAD_ARG)?;
        let mut data = &req[1..];
        if flags & JPEG_FIRST != 0 {
            let head = data.get(..6).ok_or(ST_BAD_ARG)?;
            let (bus, dc) = (head[0], head[1]);
            if !open(bus) {
                return Err(ST_BAD_ARG);
            }
            if !dc_ok(dc) {
                return Err(ST_BAD_PIN);
            }
            let core = usize::from(flags & JPEG_CORE1 != 0);
            if core == 1 {
                // 第二个核的接收缓冲区要等它画完上一张才能覆盖
                wait_core1();
            }
            self.jpeg = Some(Jpeg {
                bus,
                dc,
                x: u16::from_le_bytes([head[2], head[3]]),
                y: u16::from_le_bytes([head[4], head[5]]),
                len: 0,
                core,
            });
            data = &data[6..];
        }
        let jpeg = self.jpeg.as_mut().ok_or(ST_BAD_ARG)?;
        let buf = jpeg_buf(jpeg.core);
        if jpeg.len + data.len() > buf.len() {
            self.jpeg = None;
            return Err(ST_NO_SPACE);
        }
        buf[jpeg.len..jpeg.len + data.len()].copy_from_slice(data);
        jpeg.len += data.len();
        if flags & JPEG_LAST == 0 {
            return Ok(0);
        }
        let jpeg = self.jpeg.take().ok_or(ST_BAD_ARG)?;
        let us = if jpeg.core == 1 {
            CORE1_BUSY.lock(|c| c.set(true));
            CORE1_JOB.lock(|job| job.set(Some(jpeg)));
            cortex_m::asm::sev();
            CORE1_US.lock(Cell::get)
        } else {
            show_jpeg(&jpeg)?
        };
        out[..4].copy_from_slice(&us.to_le_bytes());
        Ok(4)
    }
}

fn wait_core1() {
    while CORE1_BUSY.lock(Cell::get) {}
}

/// 拿到 SPI 锁后执行 `f`：另一个核正在用就一直等。
fn with_spi<R>(f: impl FnOnce() -> R) -> R {
    while !SPI_LOCKED.lock(|locked| !locked.replace(true)) {}
    let result = f();
    SPI_LOCKED.lock(|locked| locked.set(false));
    result
}

/// 解码并显示，返回用时（微秒）。每解出一行 MCU 就转成 RGB565 放进条带，条带满了设好窗口交给 DMA 发送；
/// 每个核有两块条带轮流用，DMA 发这一条的同时解下一条。
fn show_jpeg(jpeg: &Jpeg) -> Result<u32, u8> {
    let started = Instant::now();
    let core = jpeg.core;
    let data = &jpeg_buf(core)[..jpeg.len];
    let bus = Bus::new(jpeg.bus)?;
    let mut pool = MemoryPool::new(pool_buf(core));
    let mut decoder = JpegDecoder::new();
    decoder.prepare(data, &mut pool).map_err(|_| ST_BAD_ARG)?;
    let width = decoder.width() as usize;
    if width > STRIP_WIDTH {
        return Err(ST_NO_SPACE);
    }
    let mut strip = 0usize;
    let mut failed = None;
    let mut callback = |_: &JpegDecoder, bitmap: &[u8], rect: &Rectangle| {
        let rows = rect.height() as usize;
        if rows > STRIP_ROWS {
            failed = Some(ST_NO_SPACE);
            return Ok(false);
        }
        // 只有一次 DMA 在发，所以上一次发出去的另一块条带此时肯定已经发完，可以接着写这块
        let buf = strip_buf(core, strip);
        let (left, w) = (rect.left as usize, rect.width() as usize);
        for row in 0..rows {
            let src = &bitmap[row * w * 3..(row + 1) * w * 3];
            let dst = &mut buf[(row * width + left) * 2..(row * width + left + w) * 2];
            for (rgb, px) in src.chunks_exact(3).zip(dst.chunks_exact_mut(2)) {
                let c = (u16::from(rgb[0] & 0xF8) << 8)
                    | (u16::from(rgb[1] & 0xFC) << 3)
                    | u16::from(rgb[2] >> 3);
                px.copy_from_slice(&c.to_be_bytes());
            }
        }
        // 一行 MCU 的最后一块：整条带发出去
        if rect.right as usize + 1 == width {
            let top = jpeg.y + rect.top;
            with_spi(|| {
                dma_finish();
                set_window(bus, jpeg.dc, jpeg.x, top, jpeg.x + width as u16 - 1, top + rows as u16 - 1);
                start(bus, &buf[..width * rows * 2]);
            });
            strip ^= 1;
        }
        Ok(true)
    };
    let result = decoder.decompress(data, 0, mcu_buf(core), work_buf(core), &mut callback);
    // 下一张从 0 号条带开始写，先确认它已经发完
    with_spi(dma_finish);
    if let Some(status) = failed {
        return Err(status);
    }
    result.map_err(|_| ST_IO)?;
    Ok(started.elapsed().as_micros() as u32)
}

/// 等上一次 DMA 发完（拿着 SPI 锁时调用）。
fn dma_finish() {
    if let Some(bus) = DMA_BUSY.lock(|b| b.take()) {
        wait_idle(bus);
    }
}

/// 交给 DMA 发送（拿着 SPI 锁时调用）。
fn start(bus: Bus, data: &[u8]) {
    dma_finish();
    bus.spi.dmacr().write(|w| w.set_txdmae(true));
    let ch = pac::DMA.ch(DMA_CH);
    ch.read_addr().write_value(data.as_ptr() as u32);
    ch.write_addr().write_value(bus.spi.dr().as_ptr() as u32);
    ch.trans_count().write(|w| *w = data.len() as u32);
    let mut ctrl = pac::dma::regs::CtrlTrig(0);
    ctrl.set_en(true);
    ctrl.set_data_size(DataSize::SIZE_BYTE);
    ctrl.set_incr_read(true);
    ctrl.set_chain_to(DMA_CH as u8);
    ctrl.set_treq_sel(bus.treq);
    ctrl.set_irq_quiet(true);
    ch.ctrl_trig().write_value(ctrl);
    DMA_BUSY.lock(|b| b.set(Some(bus)));
}

/// 等 DMA 和 SPI 把数据发完。只发不收时 RX FIFO 会收满旧数据并置溢出标志，
/// 清掉它们，免得之后的 OP_SPI_TRANSFER 读到。
fn wait_idle(bus: Bus) {
    while pac::DMA.ch(DMA_CH).ctrl_trig().read().busy() {}
    while bus.spi.sr().read().bsy() {}
    while bus.spi.sr().read().rne() {
        let _ = bus.spi.dr().read();
    }
    bus.spi.icr().write(|w| w.set_roric(true));
    bus.spi.dmacr().write(|w| w.set_txdmae(false));
}

/// CPU 直接写几个字节（屏的命令和参数），写完才返回。
fn send(bus: Bus, bytes: &[u8]) {
    for &b in bytes {
        while !bus.spi.sr().read().tnf() {}
        bus.spi.dr().write(|w| w.set_data(u16::from(b)));
    }
    while bus.spi.sr().read().bsy() {}
    while bus.spi.sr().read().rne() {
        let _ = bus.spi.dr().read();
    }
    bus.spi.icr().write(|w| w.set_roric(true));
}

/// ST7789 一类的屏：DC 拉低发命令，拉高发参数。设好写入窗口后进入写显存（RAMWR），DC 保持高。
fn set_window(bus: Bus, dc: u8, x0: u16, y0: u16, x1: u16, y1: u16) {
    let cmd = |c: u8, params: &[u8]| {
        pins::write(dc, false);
        send(bus, &[c]);
        pins::write(dc, true);
        send(bus, params);
    };
    let range = |a: u16, b: u16| [(a >> 8) as u8, a as u8, (b >> 8) as u8, b as u8];
    cmd(0x2A, &range(x0, x1));
    cmd(0x2B, &range(y0, y1));
    cmd(0x2C, &[]);
}

// 下面几块缓冲区都比较大，放在 static 里而不是栈上。带 core 参数的每个核一份，各用各的。

/// OP_SPI_WRITE / OP_SPI_WRITE_LZ4 用，只有第一个核用。
fn dma_buf() -> &'static mut [u8; SPI_LZ4_MAX] {
    static mut BUF: [u8; SPI_LZ4_MAX] = [0; SPI_LZ4_MAX];
    unsafe { &mut *core::ptr::addr_of_mut!(BUF) }
}

fn strip_buf(core: usize, n: usize) -> &'static mut [u8; STRIP_BYTES] {
    static mut BUFS: [[[u8; STRIP_BYTES]; 2]; 2] = [[[0; STRIP_BYTES]; 2]; 2];
    unsafe { &mut (*core::ptr::addr_of_mut!(BUFS))[core][n] }
}

fn jpeg_buf(core: usize) -> &'static mut [u8; JPEG_MAX] {
    static mut BUFS: [[u8; JPEG_MAX]; 2] = [[0; JPEG_MAX]; 2];
    unsafe { &mut (*core::ptr::addr_of_mut!(BUFS))[core] }
}

fn pool_buf(core: usize) -> &'static mut [u8; RECOMMENDED_POOL_SIZE] {
    static mut BUFS: [[u8; RECOMMENDED_POOL_SIZE]; 2] = [[0; RECOMMENDED_POOL_SIZE]; 2];
    unsafe { &mut (*core::ptr::addr_of_mut!(BUFS))[core] }
}

/// 4:2:0 一个 MCU 是 4 个 Y 块加 Cb、Cr，共 6 × 64 个系数。
fn mcu_buf(core: usize) -> &'static mut [i16; 6 * 64] {
    static mut BUFS: [[i16; 6 * 64]; 2] = [[0; 6 * 64]; 2];
    unsafe { &mut (*core::ptr::addr_of_mut!(BUFS))[core] }
}

/// 一个 MCU 解出的 RGB888，最大 16×16。
fn work_buf(core: usize) -> &'static mut [u8; 16 * 16 * 3] {
    static mut BUFS: [[u8; 16 * 16 * 3]; 2] = [[0; 16 * 16 * 3]; 2];
    unsafe { &mut (*core::ptr::addr_of_mut!(BUFS))[core] }
}
