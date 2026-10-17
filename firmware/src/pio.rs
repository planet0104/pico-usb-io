//! 通用 PIO 引擎：固件只负责"装程序、配状态机、收发数据"，不认识任何具体用途。
//!
//! RP2040 有两个 PIO 块（PIO0、PIO1），每块 4 个状态机（SM）共享 32 条指令内存。
//! 上位机把汇编好的 PIO 程序和配置发过来，这里照着写寄存器；I2S、WS2812、舵机、
//! 红外等时序全部由上位机的 PIO 程序决定。
//!
//! 数据回传有两种"流"：
//! 连续模式用 DMA 把 RX FIFO 不停搬进环形缓冲区，适合 I2S 这类高速连续数据；
//! 稀疏模式不用 DMA，主循环有空时直接读 RX FIFO，适合红外边沿这类偶尔才有的数据。
//! 同一时间只允许一个流。
//!
//! 引脚号、PIO 编号、SM 编号都是运行时传进来的，所以直接用 PAC 寄存器按编号访问，
//! 而不用 embassy 的类型化 PIO 驱动。

use core::sync::atomic::{compiler_fence, Ordering};

use embassy_rp::interrupt::InterruptExt;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_time::{Duration, Instant, Timer};
use io_proto::{
    read_u16, read_u32, PIO_AUTOPULL, PIO_AUTOPUSH, PIO_IN_SHIFT_RIGHT, PIO_JOIN_RX, PIO_JOIN_TX,
    PIO_LOAD_AUTO, PIO_NO_PIN, PIO_OUT_SHIFT_RIGHT, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS,
    PIO_RESTART_SM, PIO_SIDESET_OPT, PIO_SIDESET_PINDIRS, PIO_SM_CONFIG_LEN, PIO_STREAM_SPARSE,
    PIO_TX_WAIT, ST_BAD_ARG, ST_BAD_PIN, ST_BUSY, ST_NO_SPACE,
};

/// DMA 双缓冲每一半的字数。48 kHz I2S 每帧一个字时约 21 ms 完成一半。
const HALF: usize = 1024;
/// 环形缓冲区字数（64 KB），48 kHz I2S 约可缓存 341 ms，用来扛住 USB 偶尔的停顿。
const RING: usize = 16384;
/// GPIO 功能选择：PIO0 是 6，PIO1 是 7。
const FUNC_PIO0: u8 = 6;
/// DMA 触发源编号：PIO0 的 RX0..3 是 4..7，PIO1 的 RX0..3 是 12..15。
const TREQ_PIO0_RX0: u8 = 4;

// 下面这些状态会在 DMA 中断里改，所以放在 static 里；主循环访问时都包在
// `cortex_m::interrupt::free` 里，避免和中断同时修改。

/// DMA 双缓冲：前一半给通道 0，后一半给通道 1，两个通道轮流写。
static mut DMA_BUF: [u32; HALF * 2] = [0; HALF * 2];
/// 环形缓冲：中断写入（RING_W），主循环读出（RING_R），RING_N 是当前未读字数。
static mut RING_BUF: [u32; RING] = [0; RING];
static mut RING_W: usize = 0;
static mut RING_R: usize = 0;
static mut RING_N: usize = 0;
/// 环形缓冲满时丢掉的字数，供上位机查询，判断是否丢数据。
static mut DROPPED: u32 = 0;
/// 连续流是否在运行；停止后中断里看到 false 就只清标志不搬数据。
static mut LIVE: bool = false;
/// 下一个应完成的 DMA 通道，两个通道同时完成时按这个顺序处理，保证数据不乱序。
static mut NEXT: u8 = 0;
/// DMA 读取源（RX FIFO 寄存器地址）和触发源编号，由 stream_start 设置。
static mut SRC_ADDR: u32 = 0;
static mut SRC_TREQ: u8 = 0;

/// 一段已装载的程序。wrap 地址已经加上了装载起点，是绝对地址。
#[derive(Clone, Copy)]
struct Prog {
    count: u8,
    wrap_target: u8,
    wrap: u8,
}

/// 当前正在运行的流。
#[derive(Clone, Copy)]
struct Stream {
    pio: usize,
    sm: usize,
    sparse: bool,
    /// 每个事件帧最多带多少个字。
    chunk: usize,
}

pub struct Engine {
    /// 每个 PIO 块的指令内存占用位图，第 n 位为 1 表示地址 n 已被占用。
    used: [u32; 2],
    /// 按装载起点记录程序信息，用于配置状态机和卸载时核对。
    progs: [[Option<Prog>; 32]; 2],
    /// 每个状态机占用的引脚位图，引脚被别的功能抢走时据此停掉对应状态机。
    sm_pins: [[u32; 4]; 2],
    /// 每个状态机绑定的程序起点，未配置为 None。
    sm_prog: [[Option<u8>; 4]; 2],
    stream: Option<Stream>,
}

impl Engine {
    pub const fn new() -> Self {
        Self {
            used: [0; 2],
            progs: [[None; 32]; 2],
            sm_pins: [[0; 4]; 2],
            sm_prog: [[None; 4]; 2],
            stream: None,
        }
    }

    /// OP_PIO_LOAD：把上位机发来的指令字写进指令内存，应答实际装载起点和条数。
    pub fn load(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let pio = pio_index(req)?;
        let flags = *req.get(1).ok_or(ST_BAD_ARG)?;
        let want = *req.get(2).ok_or(ST_BAD_ARG)? as usize;
        let wrap_target = *req.get(3).ok_or(ST_BAD_ARG)? as usize;
        let wrap = *req.get(4).ok_or(ST_BAD_ARG)? as usize;
        let count = *req.get(5).ok_or(ST_BAD_ARG)? as usize;
        if count == 0 || count > 32 || wrap_target >= count || wrap >= count {
            return Err(ST_BAD_ARG);
        }
        let code = req.get(6..6 + count * 2).ok_or(ST_BAD_ARG)?;
        let mask = if count == 32 { u32::MAX } else { (1u32 << count) - 1 };
        let origin = if flags & PIO_LOAD_AUTO != 0 {
            // 从低地址开始找第一段足够长的连续空闲区。
            (0..=32 - count)
                .find(|&o| self.used[pio] & (mask << o) == 0)
                .ok_or(ST_NO_SPACE)?
        } else {
            if want + count > 32 {
                return Err(ST_BAD_ARG);
            }
            if self.used[pio] & (mask << want) != 0 {
                return Err(ST_NO_SPACE);
            }
            want
        };
        let block = block(pio);
        for i in 0..count {
            let mut instr = read_u16(code, i * 2).ok_or(ST_BAD_ARG)?;
            // 上位机按相对地址汇编；JMP 指令（高 3 位为 000）的低 5 位是跳转目标，要加上装载起点。
            if instr & 0xE000 == 0 {
                instr = (instr & !0x1F) | (((instr & 0x1F) + origin as u16) & 0x1F);
            }
            block.instr_mem(origin + i).write(|w| w.set_instr_mem(instr));
        }
        self.used[pio] |= mask << origin;
        self.progs[pio][origin] = Some(Prog {
            count: count as u8,
            wrap_target: (origin + wrap_target) as u8,
            wrap: (origin + wrap) as u8,
        });
        out[0] = origin as u8;
        out[1] = count as u8;
        Ok(2)
    }

    /// OP_PIO_UNLOAD：释放一段程序。还有已使能的状态机在用时拒绝。
    pub fn unload(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pio = pio_index(req)?;
        let origin = *req.get(1).ok_or(ST_BAD_ARG)? as usize;
        let count = *req.get(2).ok_or(ST_BAD_ARG)?;
        let prog = self.progs.get(pio).and_then(|p| p.get(origin)).copied().flatten();
        if prog.map(|p| p.count) != Some(count) {
            return Err(ST_BAD_ARG);
        }
        for sm in 0..4 {
            if self.sm_prog[pio][sm] == Some(origin as u8) {
                if is_enabled(pio, sm) {
                    return Err(ST_BUSY);
                }
                self.sm_prog[pio][sm] = None;
            }
        }
        let mask = if count == 32 { u32::MAX } else { (1u32 << count) - 1 };
        self.used[pio] &= !(mask << origin);
        self.progs[pio][origin] = None;
        Ok(0)
    }

    /// 只检查 OP_PIO_SM_CONFIG 的 body，不改任何状态。通过时返回状态机会用到的全部引脚
    /// （OUT / SET / IN / side-set / JMP）的位图。main.rs 先调用它，确认合法后才释放引脚，
    /// 所以参数不合法时引脚原来的功能不受影响。body 各字节含义见 pio-refactor.md 4.2 节。
    pub fn check_config(&self, req: &[u8]) -> Result<u32, u8> {
        if req.len() < PIO_SM_CONFIG_LEN {
            return Err(ST_BAD_ARG);
        }
        let (pio, _) = target(req)?;
        if self.progs[pio].get(req[2] as usize).copied().flatten().is_none() {
            return Err(ST_BAD_ARG);
        }
        let side_bits = req[13];
        let side_opt = req[14] & PIO_SIDESET_OPT != 0;
        if read_u16(req, 3) == Some(0)
            || side_bits > 5
            || (side_opt && side_bits == 0)
            || req[7] > 32
            || req[9] > 5
            || !(1..=32).contains(&req[15])
            || !(1..=32).contains(&req[16])
            || req[17] > PIO_JOIN_RX
        {
            return Err(ST_BAD_ARG);
        }
        // side-set 位数里含可选位时，真正控制引脚的位数要少 1。
        let side = side_bits - u8::from(side_opt);
        let mut mask = 0u32;
        for (base, count) in [(req[6], req[7]), (req[8], req[9]), (req[10], req[11]), (req[12], side)] {
            for pin in base as u32..base as u32 + count as u32 {
                if pin >= 30 {
                    return Err(ST_BAD_PIN);
                }
                mask |= 1 << pin;
            }
        }
        if req[18] != PIO_NO_PIN {
            if req[18] >= 30 {
                return Err(ST_BAD_PIN);
            }
            mask |= 1 << req[18];
        }
        let dirs = read_u32(req, 20).ok_or(ST_BAD_ARG)?;
        if dirs & !mask != 0 {
            return Err(ST_BAD_ARG);
        }
        Ok(mask)
    }

    /// OP_PIO_SM_CONFIG：配置一个状态机。先用 `check_config` 校验；
    /// 调用前 main.rs 已经把用到的引脚从别的功能上释放了。
    pub fn sm_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin_mask = self.check_config(req)?;
        let (pio, sm) = target(req)?;
        let origin = req[2] as usize;
        let prog = self.progs[pio][origin].ok_or(ST_BAD_ARG)?;
        let clk_int = read_u16(req, 3).ok_or(ST_BAD_ARG)?;
        let clk_frac = req[5];
        let side_bits = req[13];
        let flags = req[14];
        let side_opt = flags & PIO_SIDESET_OPT != 0;
        let pull_thresh = req[15];
        let push_thresh = req[16];
        let join = req[17];
        if self.stream.is_some_and(|s| s.pio == pio && s.sm == sm) {
            self.stop_stream();
        }
        set_enable(pio, sm, false);
        for pin in 0..30u8 {
            if pin_mask & (1 << pin) != 0 {
                crate::pins::function(pin, FUNC_PIO0 + pio as u8);
            }
        }
        let regs = block(pio).sm(sm);
        regs.clkdiv().write(|w| {
            w.set_int(clk_int);
            w.set_frac(clk_frac);
        });
        regs.execctrl().write(|w| {
            w.set_side_en(side_opt);
            w.set_side_pindir(flags & PIO_SIDESET_PINDIRS != 0);
            w.set_jmp_pin(if req[18] == PIO_NO_PIN { 0 } else { req[18] });
            w.set_wrap_top(prog.wrap);
            w.set_wrap_bottom(prog.wrap_target);
        });
        regs.shiftctrl().write(|w| {
            w.set_autopush(flags & PIO_AUTOPUSH != 0);
            w.set_autopull(flags & PIO_AUTOPULL != 0);
            w.set_in_shiftdir(flags & PIO_IN_SHIFT_RIGHT != 0);
            w.set_out_shiftdir(flags & PIO_OUT_SHIFT_RIGHT != 0);
            // 硬件里阈值 32 用 0 表示，所以只取低 5 位。
            w.set_push_thresh(push_thresh & 0x1F);
            w.set_pull_thresh(pull_thresh & 0x1F);
            w.set_fjoin_tx(join == PIO_JOIN_TX);
            w.set_fjoin_rx(join == PIO_JOIN_RX);
        });
        // PIO 的引脚方向只能由状态机自己执行 `set pindirs` 来改：
        // 临时把 SET 引脚指向单个引脚，立即执行一条 `set pindirs, 0/1`（0xE080 | 方向）。
        let dirs = read_u32(req, 20).ok_or(ST_BAD_ARG)?;
        for pin in 0..30u8 {
            if pin_mask & (1 << pin) != 0 {
                regs.pinctrl().write(|w| {
                    w.set_set_base(pin);
                    w.set_set_count(1);
                });
                let out = (dirs >> pin) as u16 & 1;
                regs.instr().write(|w| w.set_instr(0xE080 | out));
            }
        }
        // 方向设好后再写入真正的引脚映射。
        regs.pinctrl().write(|w| {
            w.set_out_base(req[6]);
            w.set_out_count(req[7]);
            w.set_set_base(req[8]);
            w.set_set_count(req[9]);
            w.set_in_base(req[10]);
            w.set_sideset_base(req[12]);
            w.set_sideset_count(side_bits);
        });
        clear_fifos(pio, sm);
        // 立即执行 `jmp origin`（JMP 指令编码就是目标地址本身），让程序从头开始。
        regs.instr().write(|w| w.set_instr(origin as u16));
        self.sm_pins[pio][sm] = pin_mask;
        self.sm_prog[pio][sm] = Some(origin as u8);
        Ok(0)
    }

    /// OP_PIO_SM_ENABLE：使能或停用状态机。没配置过程序的状态机不允许使能。
    pub fn sm_enable(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let on = *req.get(2).ok_or(ST_BAD_ARG)? != 0;
        if on && self.sm_prog[pio][sm].is_none() {
            return Err(ST_BAD_ARG);
        }
        set_enable(pio, sm, on);
        Ok(0)
    }

    /// OP_PIO_SM_RESTART：复位状态机内部状态 / 分频相位，可选清 FIFO。
    pub fn sm_restart(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let flags = *req.get(2).ok_or(ST_BAD_ARG)?;
        let bit = 1u8 << sm;
        // SM_RESTART、CLKDIV_RESTART 是写 1 生效、读回为 0 的脉冲位，用 modify 不会影响其他位。
        block(pio).ctrl().modify(|w| {
            if flags & PIO_RESTART_SM != 0 {
                w.set_sm_restart(bit);
            }
            if flags & PIO_RESTART_CLKDIV != 0 {
                w.set_clkdiv_restart(bit);
            }
        });
        if flags & PIO_RESTART_FIFOS != 0 {
            clear_fifos(pio, sm);
        }
        Ok(0)
    }

    /// OP_PIO_TX_PUSH：往 TX FIFO 写数据，应答实际写入的个数。
    /// FIFO 满时最多等 50 ms，等待期间让出执行器，USB 和其他任务照常运行。
    pub async fn tx_push(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let flags = *req.get(2).ok_or(ST_BAD_ARG)?;
        let count = *req.get(3).ok_or(ST_BAD_ARG)? as usize;
        let data = req.get(4..4 + count * 4).ok_or(ST_BAD_ARG)?;
        let block = block(pio);
        let deadline = Instant::now() + Duration::from_millis(50);
        let mut pushed = 0;
        while pushed < count {
            if block.fstat().read().txfull() & (1 << sm) != 0 {
                if flags & PIO_TX_WAIT == 0 || Instant::now() >= deadline {
                    break;
                }
                Timer::after_micros(20).await;
                continue;
            }
            block.txf(sm).write_value(read_u32(data, pushed * 4).ok_or(ST_BAD_ARG)?);
            pushed += 1;
        }
        out[0] = pushed as u8;
        Ok(1)
    }

    /// OP_PIO_RX_POP：读出 RX FIFO 里现有的数据，最多 max_words 个，可能为空。
    pub fn rx_pop(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let len = (*req.get(2).ok_or(ST_BAD_ARG)? as usize * 4).min(out.len());
        Ok(pop_fifo(pio, sm, &mut out[..len]))
    }

    /// OP_PIO_STREAM_START：开始把某个状态机的 RX 数据流式发给上位机。
    /// 上位机应在使能状态机之前调用，避免启动时 FIFO 先满、PIO 卡住。
    pub fn stream_start(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let flags = *req.get(2).ok_or(ST_BAD_ARG)?;
        let chunk = *req.get(3).ok_or(ST_BAD_ARG)?;
        if self.stream.is_some() {
            return Err(ST_BUSY);
        }
        if self.sm_prog[pio][sm].is_none() {
            return Err(ST_BAD_ARG);
        }
        let sparse = flags & PIO_STREAM_SPARSE != 0;
        reset_ring();
        if !sparse {
            unsafe {
                SRC_ADDR = block(pio).rxf(sm).as_ptr() as u32;
                SRC_TREQ = TREQ_PIO0_RX0 + (pio as u8) * 8 + sm as u8;
            }
            // 通道 1 先装好但不启动，通道 0 立即启动；通道 0 写满后由硬件自动接力到通道 1。
            pac::DMA.ints(0).write_value(0b11);
            arm_dma(1, false);
            arm_dma(0, true);
            unsafe { LIVE = true };
            let inte = pac::DMA.inte(0).read() | 0b11;
            pac::DMA.inte(0).write_value(inte);
            // bind_interrupts! 只登记处理函数，NVIC 里的中断要自己打开。
            unsafe {
                embassy_rp::interrupt::DMA_IRQ_0.unpend();
                embassy_rp::interrupt::DMA_IRQ_0.enable();
            }
        }
        self.stream = Some(Stream {
            pio,
            sm,
            sparse,
            chunk: if chunk == 0 { 255 } else { chunk as usize },
        });
        Ok(0)
    }

    /// OP_PIO_STREAM_STOP：停止流。指定的不是当前流时直接返回成功。
    pub fn stream_stop(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        if self.stream.is_some_and(|s| s.pio == pio && s.sm == sm) {
            self.stop_stream();
        }
        Ok(0)
    }

    /// OP_PIO_STREAM_STATUS：应答 [是否在运行, 环形缓冲未读字数, 已丢弃字数]。
    pub fn stream_status(&mut self, out: &mut [u8]) -> Result<usize, u8> {
        let (used, dropped) = cortex_m::interrupt::free(|_| unsafe { (RING_N as u32, DROPPED) });
        out[0] = u8::from(self.stream.is_some());
        out[1..5].copy_from_slice(&used.to_le_bytes());
        out[5..9].copy_from_slice(&dropped.to_le_bytes());
        Ok(9)
    }

    /// 主循环调用：取出最多一个事件帧的流数据写进 `out`，返回字节数；没有数据返回 0。
    pub fn stream_read(&mut self, out: &mut [u8]) -> usize {
        let Some(stream) = self.stream else {
            return 0;
        };
        let len = (stream.chunk * 4).min(out.len());
        if stream.sparse {
            return pop_fifo(stream.pio, stream.sm, &mut out[..len]);
        }
        cortex_m::interrupt::free(|_| unsafe {
            let ring = &raw const RING_BUF as *const u32;
            let mut n = 0;
            while n + 4 <= len && RING_N > 0 {
                let word = core::ptr::read_volatile(ring.add(RING_R));
                RING_R = (RING_R + 1) % RING;
                RING_N -= 1;
                out[n..n + 4].copy_from_slice(&word.to_le_bytes());
                n += 4;
            }
            n
        })
    }

    /// 引脚被重新分配给其他功能时调用：停掉所有用到这个引脚的状态机（以及它的流）。
    pub fn release_pin(&mut self, pin: u8) {
        for pio in 0..2 {
            for sm in 0..4 {
                if self.sm_pins[pio][sm] & (1 << pin) != 0 {
                    if self.stream.is_some_and(|s| s.pio == pio && s.sm == sm) {
                        self.stop_stream();
                    }
                    set_enable(pio, sm, false);
                    self.sm_pins[pio][sm] = 0;
                }
            }
        }
    }

    /// 上位机断开时调用：停流、停用全部状态机、清空指令内存记录。
    /// 引脚保持原样不动；这样上位机异常退出后再连上，不会被残留程序占满指令内存。
    pub fn reset(&mut self) {
        self.stop_stream();
        for pio in 0..2 {
            for sm in 0..4 {
                set_enable(pio, sm, false);
            }
        }
        *self = Self::new();
    }

    fn stop_stream(&mut self) {
        let Some(stream) = self.stream.take() else {
            return;
        };
        // 必须先停状态机：否则 RX DREQ 一直有效，abort 后 busy 位可能清不掉。
        set_enable(stream.pio, stream.sm, false);
        if !stream.sparse {
            unsafe { LIVE = false };
            let inte = pac::DMA.inte(0).read() & !0b11;
            pac::DMA.inte(0).write_value(inte);
            embassy_rp::interrupt::DMA_IRQ_0.disable();
            pac::DMA.chan_abort().write(|w| w.set_chan_abort(0b11));
            // 有界等待：死等会冻结执行器，USB 会一起失联。
            let mut spins = 0u32;
            while (pac::DMA.ch(0).ctrl_trig().read().busy() || pac::DMA.ch(1).ctrl_trig().read().busy())
                && spins < 200_000
            {
                spins += 1;
                cortex_m::asm::nop();
            }
            pac::DMA.ints(0).write_value(0b11);
        }
        // 引脚不动：仍由 PIO 驱动、保持最后电平，需要确定电平时由上位机用 OP_PIN_CONFIG 设置。
        reset_ring();
    }
}

/// 按编号取 PIO 块的寄存器。
fn block(pio: usize) -> pac::pio::Pio {
    if pio == 0 {
        pac::PIO0
    } else {
        pac::PIO1
    }
}

/// 解析 body 第 0 字节的 PIO 编号（0 或 1）。
fn pio_index(req: &[u8]) -> Result<usize, u8> {
    let pio = *req.first().ok_or(ST_BAD_ARG)? as usize;
    if pio > 1 {
        return Err(ST_BAD_ARG);
    }
    Ok(pio)
}

/// 解析 body 前两个字节的 (PIO 编号, 状态机编号)。
fn target(req: &[u8]) -> Result<(usize, usize), u8> {
    let pio = pio_index(req)?;
    let sm = *req.get(1).ok_or(ST_BAD_ARG)? as usize;
    if sm > 3 {
        return Err(ST_BAD_ARG);
    }
    Ok((pio, sm))
}

fn is_enabled(pio: usize, sm: usize) -> bool {
    block(pio).ctrl().read().sm_enable() & (1 << sm) != 0
}

/// 只改一个状态机的使能位，不影响同一块里的其他状态机。
fn set_enable(pio: usize, sm: usize, on: bool) {
    block(pio).ctrl().modify(|w| {
        let mask = w.sm_enable();
        w.set_sm_enable(if on { mask | (1 << sm) } else { mask & !(1 << sm) });
    });
}

/// 清空 TX / RX FIFO：改动 FJOIN 位会让硬件清 FIFO，翻转两次即可保持原配置。
fn clear_fifos(pio: usize, sm: usize) {
    let shift = block(pio).sm(sm).shiftctrl();
    shift.modify(|w| w.set_fjoin_rx(!w.fjoin_rx()));
    shift.modify(|w| w.set_fjoin_rx(!w.fjoin_rx()));
}

/// 把 RX FIFO 里现有的字读进 `out`（小端），直到 FIFO 空或 `out` 装满。
fn pop_fifo(pio: usize, sm: usize, out: &mut [u8]) -> usize {
    let block = block(pio);
    let mut n = 0;
    while n + 4 <= out.len() && block.fstat().read().rxempty() & (1 << sm) == 0 {
        out[n..n + 4].copy_from_slice(&block.rxf(sm).read().to_le_bytes());
        n += 4;
    }
    n
}

fn reset_ring() {
    cortex_m::interrupt::free(|_| unsafe {
        RING_W = 0;
        RING_R = 0;
        RING_N = 0;
        DROPPED = 0;
        NEXT = 0;
    });
}

/// DMA_IRQ_0 中断处理：某个通道写满半区后触发，把这一半拷进环形缓冲，再重新装填该通道。
/// 另一个通道此时已经接力在写另一半，所以只要在它写满（约 21 ms）之前处理完就不会丢数据。
pub struct StreamDmaHandler;

impl embassy_rp::interrupt::typelevel::Handler<embassy_rp::interrupt::typelevel::DMA_IRQ_0> for StreamDmaHandler {
    unsafe fn on_interrupt() {
        let dma = pac::DMA;
        // 最多处理两次：两个通道可能都已完成。
        for _ in 0..2 {
            let status = dma.ints(0).read() & 0b11;
            if status == 0 {
                return;
            }
            if !LIVE {
                dma.ints(0).write_value(status);
                return;
            }
            // 优先处理按顺序该轮到的通道，保证半区按时间顺序进入环形缓冲。
            let ch = if status & (1 << NEXT) != 0 {
                NEXT
            } else {
                status.trailing_zeros() as u8
            };
            dma.ints(0).write_value(1 << ch);
            copy_half(ch as usize);
            arm_dma(ch as usize, false);
            NEXT = ch ^ 1;
        }
    }
}

/// 把通道 `ch` 刚写满的半区原样拷进环形缓冲；放不下的部分丢弃并计数。
fn copy_half(ch: usize) {
    // 保证读到的是 DMA 写完后的内存内容。
    compiler_fence(Ordering::Acquire);
    cortex_m::interrupt::free(|_| unsafe {
        if !LIVE {
            return;
        }
        let count = HALF.min(RING - RING_N);
        DROPPED = DROPPED.wrapping_add((HALF - count) as u32);
        let src = (&raw const DMA_BUF as *const u32).add(ch * HALF);
        let ring = &raw mut RING_BUF as *mut u32;
        for i in 0..count {
            core::ptr::write(ring.add(RING_W), core::ptr::read_volatile(src.add(i)));
            RING_W = (RING_W + 1) % RING;
        }
        RING_N += count;
    });
}

/// 装填 DMA 通道：从 RX FIFO（地址不递增）搬 HALF 个字到它的半区（地址递增），
/// 由 PIO 的 RX 请求节拍触发，完成后接力到另一个通道并产生中断。
/// `trigger` 为 true 时立即启动，否则只装好参数等待接力（写 AL1_CTRL 不会触发启动）。
fn arm_dma(ch: usize, trigger: bool) {
    let buf = unsafe { (&raw mut DMA_BUF as *mut u32).add(ch * HALF) };
    let regs = pac::DMA.ch(ch);
    regs.read_addr().write_value(unsafe { SRC_ADDR });
    regs.write_addr().write_value(buf as u32);
    regs.trans_count().write(|w| *w = HALF as u32);
    let mut ctrl = pac::dma::regs::CtrlTrig(0);
    ctrl.set_en(true);
    ctrl.set_data_size(DataSize::SIZE_WORD);
    ctrl.set_incr_write(true);
    ctrl.set_chain_to(if ch == 0 { 1 } else { 0 });
    ctrl.set_treq_sel(TreqSel::from_bits(unsafe { SRC_TREQ }));
    ctrl.set_irq_quiet(false);
    if trigger {
        regs.ctrl_trig().write_value(ctrl);
    } else {
        regs.al1_ctrl().write_value(ctrl.0);
    }
}
