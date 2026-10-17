//! 直接操作 GPIO 寄存器的底层工具：功能选择、上下拉、读写电平、PWM。
//! 引脚号按运行时数字传入，所以用 PAC 寄存器而不是 embassy 的类型化引脚。

use embassy_rp::pac;

/// IO_BANK0 的功能选择编号（RP2040 数据手册 GPIO 功能表）。
pub const FUNC_PWM: u8 = 4;
pub const FUNC_SIO: u8 = 5;
/// 断开所有外设，引脚悬空。
pub const FUNC_NULL: u8 = 0x1f;

/// GP0..GP29 才是有效引脚。
pub fn valid(pin: u8) -> bool {
    pin < 30
}

/// 把引脚切换到指定外设功能，并打开输入缓冲、允许输出。
pub fn function(pin: u8, funcsel: u8) {
    pac::IO_BANK0.gpio(pin as usize).ctrl().write(|w| {
        w.set_funcsel(funcsel);
    });
    pac::PADS_BANK0.gpio(pin as usize).modify(|w| {
        w.set_ie(true);
        w.set_od(false);
    });
}

/// 设置上拉 / 下拉。
pub fn pull(pin: u8, up: bool, down: bool) {
    pac::PADS_BANK0.gpio(pin as usize).modify(|w| {
        w.set_ie(true);
        w.set_pue(up);
        w.set_pde(down);
        w.set_od(false);
    });
}

/// 通过 SIO 设置输出电平（引脚需已是 SIO 功能且为输出）。
pub fn write(pin: u8, high: bool) {
    let bit = 1u32 << pin;
    if high {
        pac::SIO.gpio_out(0).value_set().write_value(bit);
    } else {
        pac::SIO.gpio_out(0).value_clr().write_value(bit);
    }
}

/// 读取引脚当前电平，任何功能下都能读。
pub fn read(pin: u8) -> bool {
    pac::SIO.gpio_in(0).read() & (1 << pin) != 0
}

/// 设为普通输出：先写好电平再打开输出使能，避免出现一瞬间的错误电平。
pub fn set_output(pin: u8, high: bool) {
    function(pin, FUNC_SIO);
    write(pin, high);
    pac::SIO.gpio_oe(0).value_set().write_value(1 << pin);
}

/// 设为普通输入。
pub fn set_input(pin: u8, up: bool, down: bool) {
    function(pin, FUNC_SIO);
    pac::SIO.gpio_oe(0).value_clr().write_value(1 << pin);
    pull(pin, up, down);
}

/// 设为高阻：断开外设、关输出、去掉上下拉。
pub fn set_hiz(pin: u8) {
    function(pin, FUNC_NULL);
    pac::SIO.gpio_oe(0).value_clr().write_value(1 << pin);
    pull(pin, false, false);
}

/// 用硬件 PWM 在引脚上输出方波。`duty_permille` 是千分比占空比。
/// 频率为 0 时只把该通道占空比清零，不切换引脚功能：引脚已是 PWM 时即输出低。
/// 返回 false 表示参数无法实现（频率太高或太低）。
pub fn pwm_set(pin: u8, freq_hz: u32, duty_permille: u16) -> bool {
    if !valid(pin) || duty_permille > 1000 {
        return false;
    }
    // 每个 PWM slice 管两个相邻引脚：偶数脚是 A 通道，奇数脚是 B 通道；GP16 起编号循环。
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
    // 计数器最大 65536，周期装不下时加大整数分频。
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
