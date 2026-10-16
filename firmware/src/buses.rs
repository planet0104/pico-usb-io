use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals;
use embassy_rp::spi::{self, Phase, Polarity, Spi};
use embassy_rp::uart::{self, Uart};
use io_proto::{ST_BAD_ARG, ST_BAD_PIN};

pub type I2c0 = I2c<'static, peripherals::I2C0, i2c::Blocking>;
pub type I2c1 = I2c<'static, peripherals::I2C1, i2c::Blocking>;
pub type Spi0 = Spi<'static, peripherals::SPI0, spi::Blocking>;
pub type Spi1 = Spi<'static, peripherals::SPI1, spi::Blocking>;
pub type UartBus = Uart<'static, uart::Blocking>;

fn i2c_cfg(hz: u32) -> Result<i2c::Config, u8> {
    if !(10_000..=1_000_000).contains(&hz) { return Err(ST_BAD_ARG); }
    let mut config = i2c::Config::default();
    config.frequency = hz;
    config.sda_pullup = true;
    config.scl_pullup = true;
    Ok(config)
}

pub fn open_i2c0(sda: u8, scl: u8, hz: u32) -> Result<I2c0, u8> {
    let config = i2c_cfg(hz)?;
    match (sda, scl) {
        (0, 1) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_1::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
        (4, 5) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_5::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
        (8, 9) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_9::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
        (12, 13) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_13::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
        (16, 17) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_17::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
        (20, 21) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_21::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
        (24, 25) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_25::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
        (28, 29) => Ok(I2c::new_blocking(unsafe { peripherals::I2C0::steal() }, unsafe { peripherals::PIN_29::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
        _ => Err(ST_BAD_PIN),
    }
}

pub fn open_i2c1(sda: u8, scl: u8, hz: u32) -> Result<I2c1, u8> {
    let config = i2c_cfg(hz)?;
    match (sda, scl) {
        (2, 3) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_2::steal() }, config)),
        (6, 7) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_6::steal() }, config)),
        (10, 11) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_10::steal() }, config)),
        (14, 15) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_14::steal() }, config)),
        (18, 19) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_18::steal() }, config)),
        (22, 23) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_22::steal() }, config)),
        (26, 27) => Ok(I2c::new_blocking(unsafe { peripherals::I2C1::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_26::steal() }, config)),
        _ => Err(ST_BAD_PIN),
    }
}

pub fn open_spi0(sck: u8, mosi: u8, miso: Option<u8>, hz: u32, mode: u8) -> Result<Spi0, u8> {
    if mode > 3 || hz == 0 { return Err(ST_BAD_ARG); }
    let mut config = spi::Config::default();
    config.frequency = hz;
    config.polarity = if mode & 2 == 0 { Polarity::IdleLow } else { Polarity::IdleHigh };
    config.phase = if mode & 1 == 0 { Phase::CaptureOnFirstTransition } else { Phase::CaptureOnSecondTransition };
    if let Some(miso) = miso {
        match (sck, mosi, miso) {
            (2, 3, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (2, 3, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (2, 3, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (2, 3, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (2, 7, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (2, 7, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (2, 7, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (2, 7, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (2, 19, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (2, 19, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (2, 19, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (2, 19, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (2, 23, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (2, 23, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (2, 23, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (2, 23, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (6, 3, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (6, 3, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (6, 3, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (6, 3, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (6, 7, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (6, 7, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (6, 7, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (6, 7, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (6, 19, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (6, 19, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (6, 19, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (6, 19, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (6, 23, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (6, 23, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (6, 23, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (6, 23, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (18, 3, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (18, 3, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (18, 3, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (18, 3, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (18, 7, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (18, 7, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (18, 7, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (18, 7, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (18, 19, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (18, 19, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (18, 19, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (18, 19, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (18, 23, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (18, 23, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (18, 23, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (18, 23, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (22, 3, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (22, 3, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (22, 3, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (22, 3, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_3::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (22, 7, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (22, 7, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (22, 7, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (22, 7, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_7::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (22, 19, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (22, 19, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (22, 19, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (22, 19, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_19::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            (22, 23, 0) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_0::steal() }, config)),
            (22, 23, 4) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_4::steal() }, config)),
            (22, 23, 16) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_16::steal() }, config)),
            (22, 23, 20) => Ok(Spi::new_blocking(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_23::steal() }, unsafe { peripherals::PIN_20::steal() }, config)),
            _ => Err(ST_BAD_PIN),
        }
    } else {
        match (sck, mosi) {
            (2, 3) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_3::steal() }, config)),
            (2, 7) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_7::steal() }, config)),
            (2, 19) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_19::steal() }, config)),
            (2, 23) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_2::steal() }, unsafe { peripherals::PIN_23::steal() }, config)),
            (6, 3) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_3::steal() }, config)),
            (6, 7) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_7::steal() }, config)),
            (6, 19) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_19::steal() }, config)),
            (6, 23) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_6::steal() }, unsafe { peripherals::PIN_23::steal() }, config)),
            (18, 3) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_3::steal() }, config)),
            (18, 7) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_7::steal() }, config)),
            (18, 19) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_19::steal() }, config)),
            (18, 23) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_18::steal() }, unsafe { peripherals::PIN_23::steal() }, config)),
            (22, 3) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_3::steal() }, config)),
            (22, 7) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_7::steal() }, config)),
            (22, 19) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_19::steal() }, config)),
            (22, 23) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI0::steal() }, unsafe { peripherals::PIN_22::steal() }, unsafe { peripherals::PIN_23::steal() }, config)),
            _ => Err(ST_BAD_PIN),
        }
    }
}

pub fn open_spi1(sck: u8, mosi: u8, miso: Option<u8>, hz: u32, mode: u8) -> Result<Spi1, u8> {
    if mode > 3 || hz == 0 { return Err(ST_BAD_ARG); }
    let mut config = spi::Config::default();
    config.frequency = hz;
    config.polarity = if mode & 2 == 0 { Polarity::IdleLow } else { Polarity::IdleHigh };
    config.phase = if mode & 1 == 0 { Phase::CaptureOnFirstTransition } else { Phase::CaptureOnSecondTransition };
    if let Some(miso) = miso {
        match (sck, mosi, miso) {
            (10, 11, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (10, 11, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (10, 11, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (10, 11, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (10, 15, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (10, 15, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (10, 15, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (10, 15, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (10, 27, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (10, 27, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (10, 27, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (10, 27, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (14, 11, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (14, 11, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (14, 11, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (14, 11, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (14, 15, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (14, 15, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (14, 15, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (14, 15, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (14, 27, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (14, 27, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (14, 27, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (14, 27, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (26, 11, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (26, 11, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (26, 11, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (26, 11, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_11::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (26, 15, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (26, 15, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (26, 15, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (26, 15, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_15::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            (26, 27, 8) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_8::steal() }, config)),
            (26, 27, 12) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_12::steal() }, config)),
            (26, 27, 24) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_24::steal() }, config)),
            (26, 27, 28) => Ok(Spi::new_blocking(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_27::steal() }, unsafe { peripherals::PIN_28::steal() }, config)),
            _ => Err(ST_BAD_PIN),
        }
    } else {
        match (sck, mosi) {
            (10, 11) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_11::steal() }, config)),
            (10, 15) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_15::steal() }, config)),
            (10, 27) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_10::steal() }, unsafe { peripherals::PIN_27::steal() }, config)),
            (14, 11) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_11::steal() }, config)),
            (14, 15) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_15::steal() }, config)),
            (14, 27) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_14::steal() }, unsafe { peripherals::PIN_27::steal() }, config)),
            (26, 11) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_11::steal() }, config)),
            (26, 15) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_15::steal() }, config)),
            (26, 27) => Ok(Spi::new_blocking_txonly(unsafe { peripherals::SPI1::steal() }, unsafe { peripherals::PIN_26::steal() }, unsafe { peripherals::PIN_27::steal() }, config)),
            _ => Err(ST_BAD_PIN),
        }
    }
}

pub fn open_uart0(tx: u8, rx: u8, baud: u32) -> Result<UartBus, u8> {
    if !(300..=3_000_000).contains(&baud) { return Err(ST_BAD_ARG); }
    let mut config = uart::Config::default();
    config.baudrate = baud;
    match (tx, rx) {
        (0, 1) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_0::steal() }, unsafe { peripherals::PIN_1::steal() }, config)),
        (0, 13) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_0::steal() }, unsafe { peripherals::PIN_13::steal() }, config)),
        (0, 17) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_0::steal() }, unsafe { peripherals::PIN_17::steal() }, config)),
        (0, 29) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_0::steal() }, unsafe { peripherals::PIN_29::steal() }, config)),
        (12, 1) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_12::steal() }, unsafe { peripherals::PIN_1::steal() }, config)),
        (12, 13) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_12::steal() }, unsafe { peripherals::PIN_13::steal() }, config)),
        (12, 17) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_12::steal() }, unsafe { peripherals::PIN_17::steal() }, config)),
        (12, 29) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_12::steal() }, unsafe { peripherals::PIN_29::steal() }, config)),
        (16, 1) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_16::steal() }, unsafe { peripherals::PIN_1::steal() }, config)),
        (16, 13) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_16::steal() }, unsafe { peripherals::PIN_13::steal() }, config)),
        (16, 17) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_16::steal() }, unsafe { peripherals::PIN_17::steal() }, config)),
        (16, 29) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_16::steal() }, unsafe { peripherals::PIN_29::steal() }, config)),
        (28, 1) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_28::steal() }, unsafe { peripherals::PIN_1::steal() }, config)),
        (28, 13) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_28::steal() }, unsafe { peripherals::PIN_13::steal() }, config)),
        (28, 17) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_28::steal() }, unsafe { peripherals::PIN_17::steal() }, config)),
        (28, 29) => Ok(Uart::new_blocking(unsafe { peripherals::UART0::steal() }, unsafe { peripherals::PIN_28::steal() }, unsafe { peripherals::PIN_29::steal() }, config)),
        _ => Err(ST_BAD_PIN),
    }
}

pub fn open_uart1(tx: u8, rx: u8, baud: u32) -> Result<UartBus, u8> {
    if !(300..=3_000_000).contains(&baud) { return Err(ST_BAD_ARG); }
    let mut config = uart::Config::default();
    config.baudrate = baud;
    match (tx, rx) {
        (4, 5) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_4::steal() }, unsafe { peripherals::PIN_5::steal() }, config)),
        (4, 9) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_4::steal() }, unsafe { peripherals::PIN_9::steal() }, config)),
        (4, 21) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_4::steal() }, unsafe { peripherals::PIN_21::steal() }, config)),
        (4, 25) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_4::steal() }, unsafe { peripherals::PIN_25::steal() }, config)),
        (8, 5) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_8::steal() }, unsafe { peripherals::PIN_5::steal() }, config)),
        (8, 9) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_8::steal() }, unsafe { peripherals::PIN_9::steal() }, config)),
        (8, 21) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_8::steal() }, unsafe { peripherals::PIN_21::steal() }, config)),
        (8, 25) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_8::steal() }, unsafe { peripherals::PIN_25::steal() }, config)),
        (20, 5) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_20::steal() }, unsafe { peripherals::PIN_5::steal() }, config)),
        (20, 9) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_20::steal() }, unsafe { peripherals::PIN_9::steal() }, config)),
        (20, 21) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_20::steal() }, unsafe { peripherals::PIN_21::steal() }, config)),
        (20, 25) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_20::steal() }, unsafe { peripherals::PIN_25::steal() }, config)),
        (24, 5) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_24::steal() }, unsafe { peripherals::PIN_5::steal() }, config)),
        (24, 9) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_24::steal() }, unsafe { peripherals::PIN_9::steal() }, config)),
        (24, 21) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_24::steal() }, unsafe { peripherals::PIN_21::steal() }, config)),
        (24, 25) => Ok(Uart::new_blocking(unsafe { peripherals::UART1::steal() }, unsafe { peripherals::PIN_24::steal() }, unsafe { peripherals::PIN_25::steal() }, config)),
        _ => Err(ST_BAD_PIN),
    }
}
