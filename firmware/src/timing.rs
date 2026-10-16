use core::sync::atomic::{compiler_fence, Ordering};

use embassy_rp::gpio::Level;
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_rp::peripherals::{self, PIO0};
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_time::{Duration, Timer};
use fixed::types::extra::U8;
use fixed::FixedU32;
use io_proto::{ST_BAD_ARG, ST_BAD_PIN, ST_NO_SPACE};

pub struct Timing {
    common: Common<'static, PIO0>,
    sm_pulse: StateMachine<'static, PIO0, 0>,
    sm_cap: StateMachine<'static, PIO0, 1>,
    sm_bits: StateMachine<'static, PIO0, 2>,
    pulse_prog: LoadedProgram<'static, PIO0>,
    cap_prog: LoadedProgram<'static, PIO0>,
    cap_pin: Option<Pin<'static, PIO0>>,
    cap_pin_num: Option<u8>,
    cap_level: u8,
    bit_prog: Option<LoadedProgram<'static, PIO0>>,
    sm_i2s: StateMachine<'static, PIO0, 3>,
    i2s_prog: Option<LoadedProgram<'static, PIO0>>,
    i2s_bclk: Option<Pin<'static, PIO0>>,
    i2s_ws: Option<Pin<'static, PIO0>>,
    i2s_data: Option<Pin<'static, PIO0>>,
    i2s_pins: [u8; 3],
    i2s_on: bool,
}

const I2S_HALF: usize = 2048;
const I2S_FRAMES: usize = I2S_HALF / 2;
const I2S_RING: usize = 8192;
static mut I2S_DMA: [u32; I2S_HALF * 2] = [0; I2S_HALF * 2];
static mut I2S_RING_BUF: [u32; I2S_RING] = [0; I2S_RING];
static mut I2S_RING_W: usize = 0;
static mut I2S_RING_R: usize = 0;
static mut I2S_RING_N: usize = 0;
static mut I2S_LIVE: bool = false;
static mut I2S_NEXT: u8 = 0;

pub struct Edge {
    pub pin: u8,
    pub level: u8,
    pub width_us: u32,
}

impl Timing {
    pub fn new(pio: embassy_rp::Peri<'static, PIO0>, irq: impl embassy_rp::interrupt::typelevel::Binding<embassy_rp::interrupt::typelevel::PIO0_IRQ_0, embassy_rp::pio::InterruptHandler<PIO0>> + 'static) -> Self {
        let pio = embassy_rp::pio::Pio::new(pio, irq);
        let mut common = pio.common;
        let pulse_src = pio::pio_asm!(
            ".wrap_target",
            "pull block",
            "out pins, 1",
            "pull block",
            "out x, 32",
            "delay:",
            "jmp x-- delay",
            ".wrap",
        );
        let cap_src = pio::pio_asm!(
            ".wrap_target",
            "mov x, ~null",
            "low_loop:",
            "jmp pin low_done",
            "jmp x-- low_loop",
            "low_done:",
            "mov isr, x",
            "push noblock",
            "mov x, ~null",
            "high_loop:",
            "jmp pin high_cont",
            "jmp high_done",
            "high_cont:",
            "jmp x-- high_loop",
            "high_done:",
            "mov isr, x",
            "push noblock",
            ".wrap",
        );
        let pulse_prog = common.load_program(&pulse_src.program);
        let cap_prog = common.load_program(&cap_src.program);
        Self {
            common,
            sm_pulse: pio.sm0,
            sm_cap: pio.sm1,
            sm_bits: pio.sm2,
            pulse_prog,
            cap_prog,
            cap_pin: None,
            cap_pin_num: None,
            cap_level: 0,
            bit_prog: None,
            sm_i2s: pio.sm3,
            i2s_prog: None,
            i2s_bclk: None,
            i2s_ws: None,
            i2s_data: None,
            i2s_pins: [0; 3],
            i2s_on: false,
        }
    }

    pub async fn pulse_play(&mut self, pin: u8, pulses: &[(u8, u32)]) -> Result<(), u8> {
        if pulses.is_empty() {
            return Err(ST_BAD_ARG);
        }
        self.stop_if_same(pin);
        let pio_pin = take_pin(&mut self.common, pin)?;
        self.sm_pulse.set_enable(false);
        let mut cfg = Config::default();
        cfg.use_program(&self.pulse_prog, &[]);
        cfg.set_out_pins(&[&pio_pin]);
        cfg.clock_divider = FixedU32::<U8>::from_num(125);
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.shift_out.auto_fill = false;
        self.sm_pulse.set_config(&cfg);
        self.sm_pulse.set_pin_dirs(Direction::Out, &[&pio_pin]);
        self.sm_pulse.clear_fifos();
        self.sm_pulse.set_enable(true);
        for (level, us) in pulses {
            self.push_pulse(*level, *us).await;
        }
        while !self.sm_pulse.tx().empty() {
            Timer::after_micros(20).await;
        }
        Timer::after_micros(5).await;
        self.sm_pulse.set_enable(false);
        let last = pulses.last().map(|(l, _)| *l != 0).unwrap_or(false);
        drop(pio_pin);
        crate::pins::set_output(pin, last);
        Ok(())
    }

    async fn push_pulse(&mut self, level: u8, us: u32) {
        for word in [u32::from(level & 1), us] {
            while self.sm_pulse.tx().full() {
                Timer::after_micros(20).await;
            }
            self.sm_pulse.tx().push(word);
        }
    }

    pub fn capture_start(&mut self, pin: u8) -> Result<(), u8> {
        self.capture_stop();
        let pio_pin = take_pin(&mut self.common, pin)?;
        self.cap_pin = Some(pio_pin);
        let pio_pin = self.cap_pin.as_ref().unwrap();
        self.sm_cap.set_enable(false);
        let mut cfg = Config::default();
        cfg.use_program(&self.cap_prog, &[]);
        cfg.set_jmp_pin(pio_pin);
        cfg.clock_divider = FixedU32::<U8>::from_num(125);
        self.sm_cap.set_config(&cfg);
        self.sm_cap.set_pin_dirs(Direction::In, &[pio_pin]);
        self.sm_cap.clear_fifos();
        self.sm_cap.set_enable(true);
        self.cap_pin_num = Some(pin);
        self.cap_level = 0;
        Ok(())
    }

    pub fn capture_stop(&mut self) {
        self.sm_cap.set_enable(false);
        if let Some(pin) = self.cap_pin_num.take() {
            self.cap_pin = None;
            crate::pins::set_input(pin, false, false);
        }
    }

    pub fn poll_capture(&mut self) -> Option<Edge> {
        let pin = self.cap_pin_num?;
        let word = self.sm_cap.rx().try_pull()?;
        let ticks = 0xFFFF_FFFFu32.wrapping_sub(word);
        let level = self.cap_level;
        self.cap_level ^= 1;
        Some(Edge {
            pin,
            level,
            width_us: ticks.saturating_mul(2),
        })
    }

    pub async fn bitstream(
        &mut self,
        pin: u8,
        t0h_ns: u16,
        t0l_ns: u16,
        t1h_ns: u16,
        t1l_ns: u16,
        reset_us: u16,
        data: &[u8],
    ) -> Result<(), u8> {
        if data.is_empty() {
            return Err(ST_BAD_ARG);
        }
        let (t1, t2, t3, div) = fit_ws(t0h_ns, t0l_ns, t1h_ns, t1l_ns).ok_or(ST_BAD_ARG)?;
        self.stop_if_same(pin);
        let pio_pin = take_pin(&mut self.common, pin)?;
        self.load_bits(t1, t2, t3)?;
        self.sm_bits.set_enable(false);
        let mut cfg = Config::default();
        cfg.use_program(self.bit_prog.as_ref().unwrap(), &[&pio_pin]);
        cfg.set_out_pins(&[&pio_pin]);
        cfg.clock_divider = div;
        cfg.shift_out.direction = ShiftDirection::Left;
        cfg.shift_out.auto_fill = true;
        cfg.shift_out.threshold = 8;
        self.sm_bits.set_config(&cfg);
        self.sm_bits.set_pin_dirs(Direction::Out, &[&pio_pin]);
        self.sm_bits.clear_fifos();
        self.sm_bits.set_enable(true);
        for byte in data {
            let word = (*byte as u32) << 24;
            while self.sm_bits.tx().full() {
                Timer::after_micros(10).await;
            }
            self.sm_bits.tx().push(word);
        }
        while !self.sm_bits.tx().empty() {
            Timer::after_micros(10).await;
        }
        Timer::after_micros(20).await;
        self.sm_bits.set_pins(Level::Low, &[&pio_pin]);
        self.sm_bits.set_enable(false);
        drop(pio_pin);
        crate::pins::set_output(pin, false);
        if reset_us > 0 {
            Timer::after(Duration::from_micros(reset_us as u64)).await;
        }
        Ok(())
    }

    pub fn i2s_start(&mut self, bclk: u8, ws: u8, data: u8, frame_hz: u32) -> Result<(), u8> {
        if !(8_000..=48_000).contains(&frame_hz) || ws != bclk + 1 || data == bclk || data == ws {
            return Err(ST_BAD_ARG);
        }
        if !crate::pins::valid(bclk) || !crate::pins::valid(ws) || !crate::pins::valid(data) {
            return Err(ST_BAD_PIN);
        }
        self.i2s_stop();
        if let Some(old) = self.bit_prog.take() {
            unsafe { self.common.free_instr(old.used_memory) };
        }
        let src = pio::pio_asm!(
            ".origin 23",
            ".side_set 2",
            "set x, 30 side 0b00",
            "left_data:",
            "in pins, 1 side 0b01",
            "jmp x-- left_data side 0b00",
            "in pins, 1 side 0b11",
            "set x, 30 side 0b10",
            "right_data:",
            "in pins, 1 side 0b11",
            "jmp x-- right_data side 0b10",
            "in pins, 1 side 0b01",
        );
        let prog = self.common.try_load_program(&src.program).map_err(|_| ST_NO_SPACE)?;
        let data_pin = take_pin(&mut self.common, data)?;
        let bclk_pin = take_pin(&mut self.common, bclk)?;
        let ws_pin = take_pin(&mut self.common, ws)?;
        self.sm_i2s.set_enable(false);
        let mut cfg = Config::default();
        cfg.use_program(&prog, &[&bclk_pin, &ws_pin]);
        cfg.set_in_pins(&[&data_pin]);
        cfg.clock_divider = calculate_pio_clock_divider(frame_hz * 128);
        cfg.shift_in = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        cfg.fifo_join = FifoJoin::RxOnly;
        self.sm_i2s.set_config(&cfg);
        self.sm_i2s.set_pin_dirs(Direction::In, &[&data_pin]);
        self.sm_i2s.set_pin_dirs(Direction::Out, &[&bclk_pin, &ws_pin]);
        self.sm_i2s.clear_fifos();
        self.sm_i2s.set_enable(true);
        self.i2s_prog = Some(prog);
        self.i2s_pins = [bclk, ws, data];
        self.i2s_bclk = Some(bclk_pin);
        self.i2s_ws = Some(ws_pin);
        self.i2s_data = Some(data_pin);
        cortex_m::interrupt::free(|_| unsafe {
            I2S_LIVE = false;
            I2S_RING_W = 0;
            I2S_RING_R = 0;
            I2S_RING_N = 0;
            I2S_NEXT = 0;
        });
        embassy_rp::pac::DMA.ints(0).write_value(0b11);
        arm_dma_ch(1, false);
        arm_dma_ch(0, true);
        unsafe { I2S_LIVE = true };
        let inte = embassy_rp::pac::DMA.inte(0).read() | 0b11;
        embassy_rp::pac::DMA.inte(0).write_value(inte);
        unsafe {
            embassy_rp::interrupt::DMA_IRQ_0.unpend();
            embassy_rp::interrupt::DMA_IRQ_0.enable();
        }
        self.i2s_on = true;
        Ok(())
    }

    pub fn i2s_stop(&mut self) {
        if self.i2s_prog.is_none() && !self.i2s_on {
            return;
        }
        self.i2s_on = false;
        unsafe { I2S_LIVE = false };
        let inte = embassy_rp::pac::DMA.inte(0).read() & !0b11;
        embassy_rp::pac::DMA.inte(0).write_value(inte);
        embassy_rp::interrupt::DMA_IRQ_0.disable();
        embassy_rp::pac::DMA.chan_abort().write(|w| w.set_chan_abort(0b11));
        while embassy_rp::pac::DMA.ch(0).ctrl_trig().read().busy() {}
        while embassy_rp::pac::DMA.ch(1).ctrl_trig().read().busy() {}
        self.sm_i2s.set_enable(false);
        if let Some(prog) = self.i2s_prog.take() {
            unsafe { self.common.free_instr(prog.used_memory) };
        }
        self.i2s_bclk = None;
        self.i2s_ws = None;
        self.i2s_data = None;
        for pin in self.i2s_pins {
            crate::pins::set_hiz(pin);
        }
        embassy_rp::pac::DMA.ints(0).write_value(0b11);
    }

    pub fn i2s_poll(&mut self) {}

    pub fn i2s_read(&mut self, out: &mut [u8]) -> usize {
        cortex_m::interrupt::free(|_| unsafe {
            let ring = &raw const I2S_RING_BUF as *const u32;
            let mut n = 0;
            while n + 4 <= out.len() && I2S_RING_N > 0 {
                let word = core::ptr::read_volatile(ring.add(I2S_RING_R));
                I2S_RING_R += 1;
                if I2S_RING_R == I2S_RING {
                    I2S_RING_R = 0;
                }
                I2S_RING_N -= 1;
                out[n..n + 4].copy_from_slice(&word.to_le_bytes());
                n += 4;
            }
            n
        })
    }

    fn load_bits(&mut self, t1: u8, t2: u8, t3: u8) -> Result<(), u8> {
        self.i2s_stop();
        self.sm_bits.set_enable(false);
        if let Some(old) = self.bit_prog.take() {
            unsafe { self.common.free_instr(old.used_memory) };
        }
        let mut src = pio::pio_asm!(
            ".origin 24",
            ".side_set 1",
            ".wrap_target",
            "bitloop:",
            "out x, 1 side 0",
            "jmp !x do_zero side 1",
            "jmp bitloop side 1",
            "do_zero:",
            "nop side 0",
            ".wrap",
        );
        patch_delay(&mut src.program.code[0], t3);
        patch_delay(&mut src.program.code[1], t1);
        patch_delay(&mut src.program.code[2], t2);
        patch_delay(&mut src.program.code[3], t2);
        self.bit_prog = Some(self.common.load_program(&src.program));
        Ok(())
    }

    fn stop_if_same(&mut self, pin: u8) {
        if self.cap_pin_num == Some(pin) {
            self.capture_stop();
        }
    }
}

pub struct I2sDmaHandler;

impl embassy_rp::interrupt::typelevel::Handler<embassy_rp::interrupt::typelevel::DMA_IRQ_0> for I2sDmaHandler {
    unsafe fn on_interrupt() {
        on_i2s_dma();
    }
}

fn on_i2s_dma() {
    let dma = embassy_rp::pac::DMA;
    for _ in 0..2 {
        let status = dma.ints(0).read() & 0b11;
        if status == 0 {
            return;
        }
        if unsafe { !I2S_LIVE } {
            dma.ints(0).write_value(status);
            return;
        }
        let next = unsafe { I2S_NEXT };
        let ch = if status & (1 << next) != 0 {
            next
        } else {
            status.trailing_zeros() as u8
        };
        dma.ints(0).write_value(1 << ch);
        copy_half(ch as usize);
        arm_dma_ch(ch as usize, false);
        unsafe { I2S_NEXT = ch ^ 1 };
    }
}

fn copy_half(ch: usize) {
    compiler_fence(Ordering::Acquire);
    cortex_m::interrupt::free(|_| unsafe {
        if !I2S_LIVE || I2S_RING_N + I2S_FRAMES > I2S_RING {
            return;
        }
        let dma = &raw const I2S_DMA as *const u32;
        let ring = &raw mut I2S_RING_BUF as *mut u32;
        let start = ch * I2S_HALF;
        let mut w = I2S_RING_W;
        for i in 0..I2S_FRAMES {
            let left = core::ptr::read_volatile(dma.add(start + i * 2));
            let right = core::ptr::read_volatile(dma.add(start + i * 2 + 1));
            core::ptr::write(ring.add(w), (left & 0xFFFF_0000) | (right >> 16));
            w += 1;
            if w == I2S_RING {
                w = 0;
            }
        }
        I2S_RING_W = w;
        I2S_RING_N += I2S_FRAMES;
    });
}

fn arm_dma_ch(ch: usize, trigger: bool) {
        let ptr = unsafe { (&raw mut I2S_DMA as *mut u32).add(ch * I2S_HALF) };
        let p = embassy_rp::pac::DMA.ch(ch);
        p.read_addr().write_value(embassy_rp::pac::PIO0.rxf(3).as_ptr() as u32);
        p.write_addr().write_value(ptr as u32);
        p.trans_count().write(|w| *w = I2S_HALF as u32);
        let mut ctrl = embassy_rp::pac::dma::regs::CtrlTrig(0);
        ctrl.set_en(true);
        ctrl.set_data_size(DataSize::SIZE_WORD);
        ctrl.set_incr_write(true);
        ctrl.set_chain_to(if ch == 0 { 1 } else { 0 });
        ctrl.set_treq_sel(TreqSel::PIO0_RX3);
        ctrl.set_irq_quiet(false);
        if trigger {
            p.ctrl_trig().write_value(ctrl);
        } else {
            p.al1_ctrl().write_value(ctrl.0);
        }
}

fn patch_delay(instr: &mut u16, cycles: u8) {
    let delay = (cycles - 1) as u16;
    *instr = (*instr & !0x0F00) | (delay << 8);
}

fn fit_ws(t0h: u16, t0l: u16, t1h: u16, t1l: u16) -> Option<(u8, u8, u8, FixedU32<U8>)> {
    if t1h <= t0h || t1l == 0 || t0h == 0 {
        return None;
    }
    let t2_ns = t1h - t0h;
    let expect_t0l = t2_ns as u32 + t1l as u32;
    let delta = (expect_t0l as i32 - t0l as i32).unsigned_abs();
    if delta * 5 > t0l as u32 {
        return None;
    }
    for dt in (40..=400).step_by(10) {
        let t1 = div_round(t0h as u32, dt);
        let t2 = div_round(t2_ns as u32, dt);
        let t3 = div_round(t1l as u32, dt);
        if !(1..=16).contains(&t1) || !(1..=16).contains(&t2) || !(1..=16).contains(&t3) {
            continue;
        }
        let div = dt as f32 / 8.0;
        if div < 1.0 {
            continue;
        }
        return Some((t1 as u8, t2 as u8, t3 as u8, FixedU32::<U8>::from_num(div)));
    }
    None
}

fn div_round(num: u32, den: u32) -> u32 {
    (num + den / 2) / den
}

fn take_pin<PIO: Instance>(common: &mut Common<'static, PIO>, pin: u8) -> Result<Pin<'static, PIO>, u8> {
    let made = match pin {
        0 => common.make_pio_pin(unsafe { peripherals::PIN_0::steal() }),
        1 => common.make_pio_pin(unsafe { peripherals::PIN_1::steal() }),
        2 => common.make_pio_pin(unsafe { peripherals::PIN_2::steal() }),
        3 => common.make_pio_pin(unsafe { peripherals::PIN_3::steal() }),
        4 => common.make_pio_pin(unsafe { peripherals::PIN_4::steal() }),
        5 => common.make_pio_pin(unsafe { peripherals::PIN_5::steal() }),
        6 => common.make_pio_pin(unsafe { peripherals::PIN_6::steal() }),
        7 => common.make_pio_pin(unsafe { peripherals::PIN_7::steal() }),
        8 => common.make_pio_pin(unsafe { peripherals::PIN_8::steal() }),
        9 => common.make_pio_pin(unsafe { peripherals::PIN_9::steal() }),
        10 => common.make_pio_pin(unsafe { peripherals::PIN_10::steal() }),
        11 => common.make_pio_pin(unsafe { peripherals::PIN_11::steal() }),
        12 => common.make_pio_pin(unsafe { peripherals::PIN_12::steal() }),
        13 => common.make_pio_pin(unsafe { peripherals::PIN_13::steal() }),
        14 => common.make_pio_pin(unsafe { peripherals::PIN_14::steal() }),
        15 => common.make_pio_pin(unsafe { peripherals::PIN_15::steal() }),
        16 => common.make_pio_pin(unsafe { peripherals::PIN_16::steal() }),
        17 => common.make_pio_pin(unsafe { peripherals::PIN_17::steal() }),
        18 => common.make_pio_pin(unsafe { peripherals::PIN_18::steal() }),
        19 => common.make_pio_pin(unsafe { peripherals::PIN_19::steal() }),
        20 => common.make_pio_pin(unsafe { peripherals::PIN_20::steal() }),
        21 => common.make_pio_pin(unsafe { peripherals::PIN_21::steal() }),
        22 => common.make_pio_pin(unsafe { peripherals::PIN_22::steal() }),
        23 => common.make_pio_pin(unsafe { peripherals::PIN_23::steal() }),
        24 => common.make_pio_pin(unsafe { peripherals::PIN_24::steal() }),
        25 => common.make_pio_pin(unsafe { peripherals::PIN_25::steal() }),
        26 => common.make_pio_pin(unsafe { peripherals::PIN_26::steal() }),
        27 => common.make_pio_pin(unsafe { peripherals::PIN_27::steal() }),
        28 => common.make_pio_pin(unsafe { peripherals::PIN_28::steal() }),
        29 => common.make_pio_pin(unsafe { peripherals::PIN_29::steal() }),
        _ => return Err(io_proto::ST_BAD_PIN),
    };
    Ok(made)
}
