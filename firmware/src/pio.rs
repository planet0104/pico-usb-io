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

const HALF: usize = 1024;
const RING: usize = 16384;
const FUNC_PIO0: u8 = 6;
const TREQ_PIO0_RX0: u8 = 4;

static mut DMA_BUF: [u32; HALF * 2] = [0; HALF * 2];
static mut RING_BUF: [u32; RING] = [0; RING];
static mut RING_W: usize = 0;
static mut RING_R: usize = 0;
static mut RING_N: usize = 0;
static mut DROPPED: u32 = 0;
static mut LIVE: bool = false;
static mut NEXT: u8 = 0;
static mut SRC_ADDR: u32 = 0;
static mut SRC_TREQ: u8 = 0;

#[derive(Clone, Copy)]
struct Prog {
    count: u8,
    wrap_target: u8,
    wrap: u8,
}

#[derive(Clone, Copy)]
struct Stream {
    pio: usize,
    sm: usize,
    sparse: bool,
    chunk: usize,
}

pub struct Engine {
    used: [u32; 2],
    progs: [[Option<Prog>; 32]; 2],
    sm_pins: [[u32; 4]; 2],
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

    pub fn sm_config(&mut self, req: &[u8], pin_mask: u32) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let origin = *req.get(2).ok_or(ST_BAD_ARG)? as usize;
        let prog = self.progs[pio].get(origin).copied().flatten().ok_or(ST_BAD_ARG)?;
        let clk_int = read_u16(req, 3).ok_or(ST_BAD_ARG)?;
        let clk_frac = req[5];
        let side_bits = req[13];
        let flags = req[14];
        let side_opt = flags & PIO_SIDESET_OPT != 0;
        let pull_thresh = req[15];
        let push_thresh = req[16];
        let join = req[17];
        if clk_int == 0
            || side_bits > 5
            || (side_opt && side_bits == 0)
            || req[7] > 32
            || req[9] > 5
            || !(1..=32).contains(&pull_thresh)
            || !(1..=32).contains(&push_thresh)
            || join > PIO_JOIN_RX
        {
            return Err(ST_BAD_ARG);
        }
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
            w.set_push_thresh(push_thresh & 0x1F);
            w.set_pull_thresh(pull_thresh & 0x1F);
            w.set_fjoin_tx(join == PIO_JOIN_TX);
            w.set_fjoin_rx(join == PIO_JOIN_RX);
        });
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
        regs.instr().write(|w| w.set_instr(origin as u16));
        self.sm_pins[pio][sm] = pin_mask;
        self.sm_prog[pio][sm] = Some(origin as u8);
        Ok(0)
    }

    pub fn sm_enable(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let on = *req.get(2).ok_or(ST_BAD_ARG)? != 0;
        if on && self.sm_prog[pio][sm].is_none() {
            return Err(ST_BAD_ARG);
        }
        set_enable(pio, sm, on);
        Ok(0)
    }

    pub fn sm_restart(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let flags = *req.get(2).ok_or(ST_BAD_ARG)?;
        let bit = 1u8 << sm;
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

    pub fn rx_pop(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        let len = (*req.get(2).ok_or(ST_BAD_ARG)? as usize * 4).min(out.len());
        Ok(pop_fifo(pio, sm, &mut out[..len]))
    }

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
            pac::DMA.ints(0).write_value(0b11);
            arm_dma(1, false);
            arm_dma(0, true);
            unsafe { LIVE = true };
            let inte = pac::DMA.inte(0).read() | 0b11;
            pac::DMA.inte(0).write_value(inte);
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

    pub fn stream_stop(&mut self, req: &[u8]) -> Result<usize, u8> {
        let (pio, sm) = target(req)?;
        if self.stream.is_some_and(|s| s.pio == pio && s.sm == sm) {
            self.stop_stream();
        }
        Ok(0)
    }

    pub fn stream_status(&mut self, out: &mut [u8]) -> Result<usize, u8> {
        let (used, dropped) = cortex_m::interrupt::free(|_| unsafe { (RING_N as u32, DROPPED) });
        out[0] = u8::from(self.stream.is_some());
        out[1..5].copy_from_slice(&used.to_le_bytes());
        out[5..9].copy_from_slice(&dropped.to_le_bytes());
        Ok(9)
    }

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
        reset_ring();
    }
}

pub fn config_pins(req: &[u8]) -> Result<u32, u8> {
    if req.len() < PIO_SM_CONFIG_LEN {
        return Err(ST_BAD_ARG);
    }
    let side = req[13].saturating_sub(u8::from(req[14] & PIO_SIDESET_OPT != 0));
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

fn block(pio: usize) -> pac::pio::Pio {
    if pio == 0 {
        pac::PIO0
    } else {
        pac::PIO1
    }
}

fn pio_index(req: &[u8]) -> Result<usize, u8> {
    let pio = *req.first().ok_or(ST_BAD_ARG)? as usize;
    if pio > 1 {
        return Err(ST_BAD_ARG);
    }
    Ok(pio)
}

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

fn set_enable(pio: usize, sm: usize, on: bool) {
    block(pio).ctrl().modify(|w| {
        let mask = w.sm_enable();
        w.set_sm_enable(if on { mask | (1 << sm) } else { mask & !(1 << sm) });
    });
}

fn clear_fifos(pio: usize, sm: usize) {
    let shift = block(pio).sm(sm).shiftctrl();
    shift.modify(|w| w.set_fjoin_rx(!w.fjoin_rx()));
    shift.modify(|w| w.set_fjoin_rx(!w.fjoin_rx()));
}

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

pub struct StreamDmaHandler;

impl embassy_rp::interrupt::typelevel::Handler<embassy_rp::interrupt::typelevel::DMA_IRQ_0> for StreamDmaHandler {
    unsafe fn on_interrupt() {
        let dma = pac::DMA;
        for _ in 0..2 {
            let status = dma.ints(0).read() & 0b11;
            if status == 0 {
                return;
            }
            if !LIVE {
                dma.ints(0).write_value(status);
                return;
            }
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

fn copy_half(ch: usize) {
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
