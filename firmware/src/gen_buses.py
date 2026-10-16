from pathlib import Path

def pin(n):
    return f"peripherals::PIN_{n}"

lines = []
lines.append("use embassy_rp::i2c::{self, I2c};")
lines.append("use embassy_rp::peripherals;")
lines.append("use embassy_rp::spi::{self, Phase, Polarity, Spi};")
lines.append("use embassy_rp::uart::{self, Uart};")
lines.append("use io_proto::{ST_BAD_ARG, ST_BAD_PIN};")
lines.append("")
lines.append("pub type I2c0 = I2c<'static, peripherals::I2C0, i2c::Blocking>;")
lines.append("pub type I2c1 = I2c<'static, peripherals::I2C1, i2c::Blocking>;")
lines.append("pub type Spi0 = Spi<'static, peripherals::SPI0, spi::Blocking>;")
lines.append("pub type Spi1 = Spi<'static, peripherals::SPI1, spi::Blocking>;")
lines.append("pub type UartBus = Uart<'static, uart::Blocking>;")
lines.append("")
lines.append("fn i2c_cfg(hz: u32) -> Result<i2c::Config, u8> {")
lines.append("    if !(10_000..=1_000_000).contains(&hz) { return Err(ST_BAD_ARG); }")
lines.append("    let mut config = i2c::Config::default();")
lines.append("    config.frequency = hz;")
lines.append("    config.sda_pullup = true;")
lines.append("    config.scl_pullup = true;")
lines.append("    Ok(config)")
lines.append("}")
lines.append("")

pairs0 = [(0,1),(4,5),(8,9),(12,13),(16,17),(20,21),(24,25),(28,29)]
pairs1 = [(2,3),(6,7),(10,11),(14,15),(18,19),(22,23),(26,27)]

def i2c_fn(name, peri, pairs):
    lines.append(f"pub fn {name}(sda: u8, scl: u8, hz: u32) -> Result<{ 'I2c0' if peri=='I2C0' else 'I2c1' }, u8> {{")
    lines.append("    let config = i2c_cfg(hz)?;")
    lines.append("    match (sda, scl) {")
    for sda, scl in pairs:
        lines.append(
            f"        ({sda}, {scl}) => Ok(I2c::new_blocking(unsafe {{ peripherals::{peri}::steal() }}, unsafe {{ {pin(scl)}::steal() }}, unsafe {{ {pin(sda)}::steal() }}, config)),"
        )
    lines.append("        _ => Err(ST_BAD_PIN),")
    lines.append("    }")
    lines.append("}")
    lines.append("")

i2c_fn("open_i2c0", "I2C0", pairs0)
i2c_fn("open_i2c1", "I2C1", pairs1)

def spi_fn(name, peri, typ, scks, mosis, misos):
    lines.append(f"pub fn {name}(sck: u8, mosi: u8, miso: Option<u8>, hz: u32, mode: u8) -> Result<{typ}, u8> {{")
    lines.append("    if mode > 3 || hz == 0 { return Err(ST_BAD_ARG); }")
    lines.append("    let mut config = spi::Config::default();")
    lines.append("    config.frequency = hz;")
    lines.append("    config.polarity = if mode & 2 == 0 { Polarity::IdleLow } else { Polarity::IdleHigh };")
    lines.append("    config.phase = if mode & 1 == 0 { Phase::CaptureOnFirstTransition } else { Phase::CaptureOnSecondTransition };")
    lines.append("    if let Some(miso) = miso {")
    lines.append("        match (sck, mosi, miso) {")
    for sck in scks:
        for mosi in mosis:
            for miso in misos:
                if len({sck, mosi, miso}) < 3:
                    continue
                lines.append(
                    f"            ({sck}, {mosi}, {miso}) => Ok(Spi::new_blocking(unsafe {{ peripherals::{peri}::steal() }}, unsafe {{ {pin(sck)}::steal() }}, unsafe {{ {pin(mosi)}::steal() }}, unsafe {{ {pin(miso)}::steal() }}, config)),"
                )
    lines.append("            _ => Err(ST_BAD_PIN),")
    lines.append("        }")
    lines.append("    } else {")
    lines.append("        match (sck, mosi) {")
    for sck in scks:
        for mosi in mosis:
            if sck == mosi:
                continue
            lines.append(
                f"            ({sck}, {mosi}) => Ok(Spi::new_blocking_txonly(unsafe {{ peripherals::{peri}::steal() }}, unsafe {{ {pin(sck)}::steal() }}, unsafe {{ {pin(mosi)}::steal() }}, config)),"
            )
    lines.append("            _ => Err(ST_BAD_PIN),")
    lines.append("        }")
    lines.append("    }")
    lines.append("}")
    lines.append("")

spi_fn("open_spi0", "SPI0", "Spi0", [2,6,18,22], [3,7,19,23], [0,4,16,20])
spi_fn("open_spi1", "SPI1", "Spi1", [10,14,26], [11,15,27], [8,12,24,28])

def uart_fn(name, peri, txs, rxs):
    lines.append(f"pub fn {name}(tx: u8, rx: u8, baud: u32) -> Result<UartBus, u8> {{")
    lines.append("    if !(300..=3_000_000).contains(&baud) { return Err(ST_BAD_ARG); }")
    lines.append("    let mut config = uart::Config::default();")
    lines.append("    config.baudrate = baud;")
    lines.append("    match (tx, rx) {")
    for tx in txs:
        for rx in rxs:
            if tx == rx:
                continue
            lines.append(
                f"        ({tx}, {rx}) => Ok(Uart::new_blocking(unsafe {{ peripherals::{peri}::steal() }}, unsafe {{ {pin(tx)}::steal() }}, unsafe {{ {pin(rx)}::steal() }}, config)),"
            )
    lines.append("        _ => Err(ST_BAD_PIN),")
    lines.append("    }")
    lines.append("}")
    lines.append("")

uart_fn("open_uart0", "UART0", [0,12,16,28], [1,13,17,29])
uart_fn("open_uart1", "UART1", [4,8,20,24], [5,9,21,25])

Path("buses.rs").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("wrote", len(lines))
