use embassy_rp::pac;

pub const FUNC_PWM: u8 = 4;
pub const FUNC_SIO: u8 = 5;
pub const FUNC_NULL: u8 = 0x1f;

pub fn valid(pin: u8) -> bool {
    pin < 30
}

pub fn function(pin: u8, funcsel: u8) {
    pac::IO_BANK0.gpio(pin as usize).ctrl().write(|w| {
        w.set_funcsel(funcsel);
    });
    pac::PADS_BANK0.gpio(pin as usize).modify(|w| {
        w.set_ie(true);
        w.set_od(false);
    });
}

pub fn pull(pin: u8, up: bool, down: bool) {
    pac::PADS_BANK0.gpio(pin as usize).modify(|w| {
        w.set_ie(true);
        w.set_pue(up);
        w.set_pde(down);
        w.set_od(false);
    });
}

pub fn write(pin: u8, high: bool) {
    let bit = 1u32 << pin;
    if high {
        pac::SIO.gpio_out(0).value_set().write_value(bit);
    } else {
        pac::SIO.gpio_out(0).value_clr().write_value(bit);
    }
}

pub fn read(pin: u8) -> bool {
    pac::SIO.gpio_in(0).read() & (1 << pin) != 0
}

pub fn set_output(pin: u8, high: bool) {
    function(pin, FUNC_SIO);
    write(pin, high);
    pac::SIO.gpio_oe(0).value_set().write_value(1 << pin);
}

pub fn set_input(pin: u8, up: bool, down: bool) {
    function(pin, FUNC_SIO);
    pac::SIO.gpio_oe(0).value_clr().write_value(1 << pin);
    pull(pin, up, down);
}

pub fn set_hiz(pin: u8) {
    function(pin, FUNC_NULL);
    pac::SIO.gpio_oe(0).value_clr().write_value(1 << pin);
    pull(pin, false, false);
}

pub fn pwm_set(pin: u8, freq_hz: u32, duty_permille: u16) -> bool {
    if !valid(pin) || duty_permille > 1000 {
        return false;
    }
    let slice = (pin as usize % 16) / 2;
    let channel_b = pin % 2 == 1;
    let ch = pac::PWM.ch(slice);
    if freq_hz == 0 {
        ch.cc().modify(|w| {
            if channel_b {
                w.set_b(0);
            } else {
                w.set_a(0);
            }
        });
        return true;
    }
    let clk = 125_000_000u32;
    let mut div = 1u32;
    let mut period = clk / freq_hz;
    while period > 65536 && div < 255 {
        div += 1;
        period = clk / (freq_hz * div);
    }
    if period < 2 || div > 255 {
        return false;
    }
    let top = (period - 1) as u16;
    let duty = ((period as u32) * duty_permille as u32 / 1000) as u16;
    function(pin, FUNC_PWM);
    ch.csr().modify(|w| w.set_en(false));
    ch.div().write(|w| {
        w.set_int(div as u8);
        w.set_frac(0);
    });
    ch.top().write(|w| w.set_top(top));
    ch.cc().modify(|w| {
        if channel_b {
            w.set_b(duty.min(top));
        } else {
            w.set_a(duty.min(top));
        }
    });
    ch.csr().write(|w| w.set_en(true));
    true
}
