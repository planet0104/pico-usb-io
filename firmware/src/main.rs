#![no_std]
#![no_main]

mod buses;
mod pins;
mod timing;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::adc::{Adc, Channel};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler as UsbInterrupt};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;
use io_proto::{
    read_u16, read_u32, Frame, Parser, KIND_EVENT, KIND_REQ, KIND_RESP, MAX_BODY, MAX_FRAME,
    OP_ADC_READ, OP_BITSTREAM, OP_GPIO_READ, OP_GPIO_WRITE, OP_I2C_CONFIG, OP_I2C_TRANSFER,
    OP_PIN_CONFIG, OP_PULSE_CAPTURE_START, OP_PULSE_CAPTURE_STOP, OP_PULSE_PLAY, OP_PWM_SET,
    OP_I2S_CONFIG, OP_I2S_READ, OP_SPI_CONFIG, OP_SPI_TRANSFER, OP_UART_CONFIG, OP_UART_READ,
    OP_UART_WRITE, OP_VERSION, PIN_COUNT, PIN_HIZ, PIN_INPUT, PIN_OUTPUT, PROTO_VERSION, PULL_DOWN,
    PULL_UP, SPI_MISO_NONE, ST_BAD_ARG, ST_BAD_PIN, ST_IO, ST_OK, EVT_PULSE,
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterrupt<USB>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let adc = Adc::new_blocking(p.ADC, Default::default());
    let timing = timing::Timing::new(p.PIO0, Irqs);
    let mut board = Board::new(adc, timing);

    let driver = Driver::new(p.USB, Irqs);
    let mut config = embassy_usb::Config::new(0x2E8A, 0x000A);
    config.manufacturer = Some("pico-usb-io");
    config.product = Some("pico-usb-io");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );
    static USB_STATE: StaticCell<State> = StaticCell::new();
    let mut class = CdcAcmClass::new(&mut builder, USB_STATE.init(State::new()), 64);
    let usb = builder.build();
    spawner.spawn(usb_task(usb).unwrap());

    loop {
        class.wait_connection().await;
        let mut parser = Parser::new();
        loop {
            if class.line_coding().data_rate() == 1200 {
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            }
            board.timing.i2s_poll();
            let _ = drain_events(&mut class, &mut board).await;
            let mut packet = [0u8; 64];
            match select(class.read_packet(&mut packet), Timer::after_millis(2)).await {
                Either::First(Ok(n)) => {
                    let mut frames = heapless_frames();
                    parser.push(&packet[..n], |frame| {
                        if frames.len < frames.buf.len() {
                            frames.buf[frames.len] = OwnedFrame::from_frame(&frame);
                            frames.len += 1;
                        }
                    });
                    for i in 0..frames.len {
                        let req = &frames.buf[i];
                        if req.kind != KIND_REQ {
                            continue;
                        }
                        let mut body = [0u8; MAX_BODY];
                        let (status, n) = board.handle(req.op, &req.body[..req.body_len], &mut body).await;
                        let _ = write_frame(&mut class, KIND_RESP, req.seq, req.op, status, &body[..n]).await;
                    }
                }
                Either::First(Err(_)) => break,
                Either::Second(_) => {}
            }
        }
    }
}

struct OwnedFrame {
    kind: u8,
    seq: u16,
    op: u16,
    body: [u8; MAX_BODY],
    body_len: usize,
}

impl OwnedFrame {
    fn from_frame(frame: &Frame<'_>) -> Self {
        let mut body = [0u8; MAX_BODY];
        let n = frame.body.len().min(MAX_BODY);
        body[..n].copy_from_slice(&frame.body[..n]);
        Self {
            kind: frame.kind,
            seq: frame.seq,
            op: frame.op,
            body,
            body_len: n,
        }
    }
}

struct FrameBuf {
    buf: [OwnedFrame; 4],
    len: usize,
}

fn heapless_frames() -> FrameBuf {
    FrameBuf {
        buf: core::array::from_fn(|_| OwnedFrame {
            kind: 0,
            seq: 0,
            op: 0,
            body: [0; MAX_BODY],
            body_len: 0,
        }),
        len: 0,
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

struct Gone;

impl From<EndpointError> for Gone {
    fn from(value: EndpointError) -> Self {
        match value {
            EndpointError::BufferOverflow => panic!("usb buffer"),
            EndpointError::Disabled => Gone,
        }
    }
}

async fn write_frame<'a, T: Instance + 'a>(
    class: &mut CdcAcmClass<'a, Driver<'a, T>>,
    kind: u8,
    seq: u16,
    op: u16,
    status: u8,
    body: &[u8],
) -> Result<(), Gone> {
    let mut raw = [0u8; MAX_FRAME];
    let Some(n) = io_proto::encode(
        Frame {
            kind,
            seq,
            op,
            status,
            body,
        },
        &mut raw,
    ) else {
        return Ok(());
    };
    let mut sent = 0;
    while sent < n {
        let end = (sent + 64).min(n);
        class.write_packet(&raw[sent..end]).await?;
        sent = end;
    }
    if n % 64 == 0 {
        class.write_packet(&[]).await?;
    }
    Ok(())
}

async fn drain_events<'a, T: Instance + 'a>(
    class: &mut CdcAcmClass<'a, Driver<'a, T>>,
    board: &mut Board,
) -> Result<(), Gone> {
    while let Some(edge) = board.timing.poll_capture() {
        let mut body = [0u8; 6];
        body[0] = edge.pin;
        body[1] = edge.level;
        body[2..6].copy_from_slice(&edge.width_us.to_le_bytes());
        write_frame(class, KIND_EVENT, 0, EVT_PULSE, ST_OK, &body).await?;
    }
    Ok(())
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Role {
    Free,
    Gpio,
    Pwm,
    I2c(u8),
    Spi(u8),
    Uart(u8),
    Capture,
    I2s,
}

struct Board {
    adc: Adc<'static, embassy_rp::adc::Blocking>,
    timing: timing::Timing,
    role: [Role; 30],
    i2c0: Option<buses::I2c0>,
    i2c1: Option<buses::I2c1>,
    spi0: Option<buses::Spi0>,
    spi1: Option<buses::Spi1>,
    uart0: Option<buses::UartBus>,
    uart1: Option<buses::UartBus>,
    i2c_pins: [[Option<u8>; 2]; 2],
    spi_pins: [[Option<u8>; 3]; 2],
    uart_pins: [[Option<u8>; 2]; 2],
}

impl Board {
    fn new(adc: Adc<'static, embassy_rp::adc::Blocking>, timing: timing::Timing) -> Self {
        Self {
            adc,
            timing,
            role: [Role::Free; 30],
            i2c0: None,
            i2c1: None,
            spi0: None,
            spi1: None,
            uart0: None,
            uart1: None,
            i2c_pins: [[None; 2]; 2],
            spi_pins: [[None; 3]; 2],
            uart_pins: [[None; 2]; 2],
        }
    }

    fn release_pin(&mut self, pin: u8) {
        match self.role[pin as usize] {
            Role::I2c(bus) => self.drop_i2c(bus),
            Role::Spi(bus) => self.drop_spi(bus),
            Role::Uart(bus) => self.drop_uart(bus),
            Role::Capture => self.timing.capture_stop(),
            Role::I2s => self.stop_i2s(),
            _ => {}
        }
        if (pin as usize) < self.role.len() {
            self.role[pin as usize] = Role::Free;
        }
    }

    fn drop_i2c(&mut self, bus: u8) {
        if bus == 0 {
            self.i2c0 = None;
        } else {
            self.i2c1 = None;
        }
        for pin in self.i2c_pins[bus as usize] {
            if let Some(pin) = pin {
                self.role[pin as usize] = Role::Free;
                pins::set_hiz(pin);
            }
        }
        self.i2c_pins[bus as usize] = [None, None];
    }

    fn drop_spi(&mut self, bus: u8) {
        if bus == 0 {
            self.spi0 = None;
        } else {
            self.spi1 = None;
        }
        for pin in self.spi_pins[bus as usize] {
            if let Some(pin) = pin {
                self.role[pin as usize] = Role::Free;
                pins::set_hiz(pin);
            }
        }
        self.spi_pins[bus as usize] = [None, None, None];
    }

    fn drop_uart(&mut self, bus: u8) {
        if bus == 0 {
            self.uart0 = None;
        } else {
            self.uart1 = None;
        }
        for pin in self.uart_pins[bus as usize] {
            if let Some(pin) = pin {
                self.role[pin as usize] = Role::Free;
                pins::set_hiz(pin);
            }
        }
        self.uart_pins[bus as usize] = [None, None];
    }

    fn prepare(&mut self, pin: u8) -> Result<(), u8> {
        if !pins::valid(pin) {
            return Err(ST_BAD_PIN);
        }
        self.release_pin(pin);
        Ok(())
    }

    async fn handle(&mut self, op: u16, req: &[u8], out: &mut [u8]) -> (u8, usize) {
        match self.dispatch(op, req, out).await {
            Ok(n) => (ST_OK, n),
            Err(status) => (status, 0),
        }
    }

    async fn dispatch(&mut self, op: u16, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        match op {
            OP_VERSION => {
                out[0..2].copy_from_slice(&PROTO_VERSION.to_le_bytes());
                out[2..4].copy_from_slice(&1u16.to_le_bytes());
                out[4] = PIN_COUNT;
                Ok(5)
            }
            OP_PIN_CONFIG => self.pin_config(req),
            OP_GPIO_WRITE => self.gpio_write(req),
            OP_GPIO_READ => self.gpio_read(req, out),
            OP_PWM_SET => self.pwm_set(req),
            OP_ADC_READ => self.adc_read(req, out),
            OP_I2C_CONFIG => self.i2c_config(req),
            OP_I2C_TRANSFER => self.i2c_transfer(req, out),
            OP_SPI_CONFIG => self.spi_config(req),
            OP_SPI_TRANSFER => self.spi_transfer(req, out),
            OP_UART_CONFIG => self.uart_config(req),
            OP_UART_WRITE => self.uart_write(req),
            OP_UART_READ => self.uart_read(req, out),
            OP_PULSE_PLAY => self.pulse_play(req).await,
            OP_PULSE_CAPTURE_START => self.capture_start(req),
            OP_PULSE_CAPTURE_STOP => self.capture_stop(req),
            OP_BITSTREAM => self.bitstream(req).await,
            OP_I2S_CONFIG => self.i2s_config(req),
            OP_I2S_READ => self.i2s_read(req, out),
            _ => Err(ST_BAD_ARG),
        }
    }

    fn pin_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        let mode = *req.get(1).ok_or(ST_BAD_ARG)?;
        let pull = *req.get(2).ok_or(ST_BAD_ARG)?;
        let level = *req.get(3).unwrap_or(&0) != 0;
        self.prepare(pin)?;
        let (up, down) = match pull {
            0 => (false, false),
            PULL_UP => (true, false),
            PULL_DOWN => (false, true),
            _ => return Err(ST_BAD_ARG),
        };
        match mode {
            PIN_HIZ => pins::set_hiz(pin),
            PIN_INPUT => pins::set_input(pin, up, down),
            PIN_OUTPUT => {
                pins::set_output(pin, level);
                if up || down {
                    pins::pull(pin, up, down);
                }
            }
            _ => return Err(ST_BAD_ARG),
        }
        self.role[pin as usize] = Role::Gpio;
        Ok(0)
    }

    fn gpio_write(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        let level = *req.get(1).ok_or(ST_BAD_ARG)? != 0;
        if !pins::valid(pin) {
            return Err(ST_BAD_PIN);
        }
        if self.role[pin as usize] != Role::Gpio {
            self.prepare(pin)?;
            pins::set_output(pin, level);
            self.role[pin as usize] = Role::Gpio;
        } else {
            pins::write(pin, level);
        }
        Ok(0)
    }

    fn gpio_read(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        if !pins::valid(pin) {
            return Err(ST_BAD_PIN);
        }
        out[0] = u8::from(pins::read(pin));
        Ok(1)
    }

    fn pwm_set(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        let freq = read_u32(req, 1).ok_or(ST_BAD_ARG)?;
        let duty = read_u16(req, 5).ok_or(ST_BAD_ARG)?;
        self.prepare(pin)?;
        if !pins::pwm_set(pin, freq, duty) {
            return Err(ST_BAD_ARG);
        }
        self.role[pin as usize] = Role::Pwm;
        Ok(0)
    }

    fn adc_read(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        self.prepare(pin)?;
        let sample = match pin {
            26 => {
                let mut ch = Channel::new_pin(unsafe { embassy_rp::peripherals::PIN_26::steal() }, Pull::None);
                self.adc.blocking_read(&mut ch)
            }
            27 => {
                let mut ch = Channel::new_pin(unsafe { embassy_rp::peripherals::PIN_27::steal() }, Pull::None);
                self.adc.blocking_read(&mut ch)
            }
            28 => {
                let mut ch = Channel::new_pin(unsafe { embassy_rp::peripherals::PIN_28::steal() }, Pull::None);
                self.adc.blocking_read(&mut ch)
            }
            29 => {
                let mut ch = Channel::new_pin(unsafe { embassy_rp::peripherals::PIN_29::steal() }, Pull::None);
                self.adc.blocking_read(&mut ch)
            }
            _ => return Err(ST_BAD_PIN),
        };
        let sample = sample.map_err(|_| ST_IO)?;
        out[0..2].copy_from_slice(&sample.to_le_bytes());
        self.role[pin as usize] = Role::Gpio;
        Ok(2)
    }

    fn i2c_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let sda = *req.first().ok_or(ST_BAD_ARG)?;
        let scl = *req.get(1).ok_or(ST_BAD_ARG)?;
        let hz = read_u32(req, 2).ok_or(ST_BAD_ARG)?;
        if sda == scl {
            return Err(ST_BAD_ARG);
        }
        let bus = i2c_bus(sda, scl)?;
        self.prepare(sda)?;
        self.prepare(scl)?;
        if bus == 0 {
            self.i2c0 = Some(buses::open_i2c0(sda, scl, hz)?);
        } else {
            self.i2c1 = Some(buses::open_i2c1(sda, scl, hz)?);
        }
        self.role[sda as usize] = Role::I2c(bus);
        self.role[scl as usize] = Role::I2c(bus);
        self.i2c_pins[bus as usize] = [Some(sda), Some(scl)];
        Ok(0)
    }

    fn i2c_transfer(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let bus = *req.first().ok_or(ST_BAD_ARG)?;
        let addr = *req.get(1).ok_or(ST_BAD_ARG)?;
        let wlen = read_u16(req, 2).ok_or(ST_BAD_ARG)? as usize;
        let rlen = read_u16(req, 4).ok_or(ST_BAD_ARG)? as usize;
        let write = req.get(6..6 + wlen).ok_or(ST_BAD_ARG)?;
        if rlen > out.len() || addr > 0x7f {
            return Err(ST_BAD_ARG);
        }
        let read = &mut out[..rlen];
        let result = if bus == 0 {
            let i2c = self.i2c0.as_mut().ok_or(ST_BAD_ARG)?;
            i2c_run(i2c, addr, write, read)
        } else if bus == 1 {
            let i2c = self.i2c1.as_mut().ok_or(ST_BAD_ARG)?;
            i2c_run(i2c, addr, write, read)
        } else {
            Err(ST_BAD_ARG)
        };
        result?;
        Ok(rlen)
    }

    fn spi_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let sck = *req.first().ok_or(ST_BAD_ARG)?;
        let mosi = *req.get(1).ok_or(ST_BAD_ARG)?;
        let miso = *req.get(2).ok_or(ST_BAD_ARG)?;
        let hz = read_u32(req, 3).ok_or(ST_BAD_ARG)?;
        let mode = *req.get(7).ok_or(ST_BAD_ARG)?;
        let miso = if miso == SPI_MISO_NONE { None } else { Some(miso) };
        if Some(sck) == Some(mosi) || miso == Some(sck) || miso == Some(mosi) {
            return Err(ST_BAD_ARG);
        }
        let bus = spi_bus(sck)?;
        self.prepare(sck)?;
        self.prepare(mosi)?;
        if let Some(miso) = miso {
            self.prepare(miso)?;
        }
        if bus == 0 {
            self.spi0 = Some(buses::open_spi0(sck, mosi, miso, hz, mode)?);
        } else {
            self.spi1 = Some(buses::open_spi1(sck, mosi, miso, hz, mode)?);
        }
        self.role[sck as usize] = Role::Spi(bus);
        self.role[mosi as usize] = Role::Spi(bus);
        self.spi_pins[bus as usize][0] = Some(sck);
        self.spi_pins[bus as usize][1] = Some(mosi);
        if let Some(miso) = miso {
            self.role[miso as usize] = Role::Spi(bus);
            self.spi_pins[bus as usize][2] = Some(miso);
        }
        Ok(0)
    }

    fn spi_transfer(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let bus = *req.first().ok_or(ST_BAD_ARG)?;
        let data = req.get(1..).ok_or(ST_BAD_ARG)?;
        if data.is_empty() || data.len() > out.len() {
            return Err(ST_BAD_ARG);
        }
        out[..data.len()].copy_from_slice(data);
        if bus == 0 {
            self.spi0.as_mut().ok_or(ST_BAD_ARG)?.blocking_transfer_in_place(&mut out[..data.len()]).map_err(|_| ST_IO)?;
        } else if bus == 1 {
            self.spi1.as_mut().ok_or(ST_BAD_ARG)?.blocking_transfer_in_place(&mut out[..data.len()]).map_err(|_| ST_IO)?;
        } else {
            return Err(ST_BAD_ARG);
        }
        Ok(data.len())
    }

    fn uart_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let tx = *req.first().ok_or(ST_BAD_ARG)?;
        let rx = *req.get(1).ok_or(ST_BAD_ARG)?;
        let baud = read_u32(req, 2).ok_or(ST_BAD_ARG)?;
        if tx == rx {
            return Err(ST_BAD_ARG);
        }
        let bus = uart_bus(tx)?;
        self.prepare(tx)?;
        self.prepare(rx)?;
        if bus == 0 {
            self.uart0 = Some(buses::open_uart0(tx, rx, baud)?);
        } else {
            self.uart1 = Some(buses::open_uart1(tx, rx, baud)?);
        }
        self.role[tx as usize] = Role::Uart(bus);
        self.role[rx as usize] = Role::Uart(bus);
        self.uart_pins[bus as usize] = [Some(tx), Some(rx)];
        Ok(0)
    }

    fn uart_write(&mut self, req: &[u8]) -> Result<usize, u8> {
        let bus = *req.first().ok_or(ST_BAD_ARG)?;
        let data = req.get(1..).ok_or(ST_BAD_ARG)?;
        if bus == 0 {
            self.uart0.as_mut().ok_or(ST_BAD_ARG)?.blocking_write(data).map_err(|_| ST_IO)?;
        } else if bus == 1 {
            self.uart1.as_mut().ok_or(ST_BAD_ARG)?.blocking_write(data).map_err(|_| ST_IO)?;
        } else {
            return Err(ST_BAD_ARG);
        }
        Ok(0)
    }

    fn uart_read(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let bus = *req.first().ok_or(ST_BAD_ARG)?;
        let max = read_u16(req, 1).ok_or(ST_BAD_ARG)? as usize;
        let max = max.min(out.len());
        let uart = match bus {
            0 => embassy_rp::pac::UART0,
            1 => embassy_rp::pac::UART1,
            _ => return Err(ST_BAD_ARG),
        };
        let mut n = 0;
        while n < max && !uart.uartfr().read().rxfe() {
            out[n] = uart.uartdr().read().data();
            n += 1;
        }
        Ok(n)
    }

    async fn pulse_play(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        let count = read_u16(req, 1).ok_or(ST_BAD_ARG)? as usize;
        if count == 0 || req.len() < 3 + count * 5 {
            return Err(ST_BAD_ARG);
        }
        let mut pulses = [(0u8, 0u32); 48];
        if count > pulses.len() {
            return Err(ST_BAD_ARG);
        }
        for i in 0..count {
            let at = 3 + i * 5;
            pulses[i] = (req[at], read_u32(req, at + 1).ok_or(ST_BAD_ARG)?);
        }
        self.prepare(pin)?;
        self.timing.pulse_play(pin, &pulses[..count]).await?;
        self.role[pin as usize] = Role::Gpio;
        Ok(0)
    }

    fn capture_start(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        self.prepare(pin)?;
        self.timing.capture_start(pin)?;
        self.role[pin as usize] = Role::Capture;
        Ok(0)
    }

    fn capture_stop(&mut self, req: &[u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        if self.role.get(pin as usize) == Some(&Role::Capture) {
            self.timing.capture_stop();
            self.role[pin as usize] = Role::Free;
        }
        Ok(0)
    }

    fn stop_i2s(&mut self) {
        self.timing.i2s_stop();
        for role in &mut self.role {
            if *role == Role::I2s {
                *role = Role::Free;
            }
        }
    }

    fn i2s_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let bclk = *req.first().ok_or(ST_BAD_ARG)?;
        let ws = *req.get(1).ok_or(ST_BAD_ARG)?;
        let data = *req.get(2).ok_or(ST_BAD_ARG)?;
        let hz = read_u32(req, 3).ok_or(ST_BAD_ARG)?;
        if hz == 0 {
            self.stop_i2s();
            return Ok(0);
        }
        self.stop_i2s();
        self.prepare(bclk)?;
        self.prepare(ws)?;
        self.prepare(data)?;
        self.timing.i2s_start(bclk, ws, data, hz)?;
        self.role[bclk as usize] = Role::I2s;
        self.role[ws as usize] = Role::I2s;
        self.role[data as usize] = Role::I2s;
        Ok(0)
    }

    fn i2s_read(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let max = read_u16(req, 0).ok_or(ST_BAD_ARG)? as usize;
        let n = max.min(out.len());
        Ok(self.timing.i2s_read(&mut out[..n]))
    }

    async fn bitstream(&mut self, req: &[u8]) -> Result<usize, u8> {
        self.stop_i2s();
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        let t0h = read_u16(req, 1).ok_or(ST_BAD_ARG)?;
        let t0l = read_u16(req, 3).ok_or(ST_BAD_ARG)?;
        let t1h = read_u16(req, 5).ok_or(ST_BAD_ARG)?;
        let t1l = read_u16(req, 7).ok_or(ST_BAD_ARG)?;
        let reset = read_u16(req, 9).ok_or(ST_BAD_ARG)?;
        let data = req.get(11..).ok_or(ST_BAD_ARG)?;
        self.prepare(pin)?;
        self.timing.bitstream(pin, t0h, t0l, t1h, t1l, reset, data).await?;
        self.role[pin as usize] = Role::Gpio;
        Ok(0)
    }
}

fn i2c_bus(sda: u8, scl: u8) -> Result<u8, u8> {
    if scl != sda + 1 {
        return Err(ST_BAD_PIN);
    }
    match sda % 4 {
        0 => Ok(0),
        2 => Ok(1),
        _ => Err(ST_BAD_PIN),
    }
}

fn spi_bus(sck: u8) -> Result<u8, u8> {
    match sck {
        2 | 6 | 18 | 22 => Ok(0),
        10 | 14 | 26 => Ok(1),
        _ => Err(ST_BAD_PIN),
    }
}

fn uart_bus(tx: u8) -> Result<u8, u8> {
    match tx {
        0 | 12 | 16 | 28 => Ok(0),
        4 | 8 | 20 | 24 => Ok(1),
        _ => Err(ST_BAD_PIN),
    }
}

fn i2c_run<T>(i2c: &mut T, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), u8>
where
    T: I2cIo,
{
    i2c.run(addr, write, read).map_err(|_| ST_IO)
}

trait I2cIo {
    fn run(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), embassy_rp::i2c::Error>;
}

impl<T: embassy_rp::i2c::Instance> I2cIo for embassy_rp::i2c::I2c<'static, T, embassy_rp::i2c::Blocking> {
    fn run(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), embassy_rp::i2c::Error> {
        if write.is_empty() && read.is_empty() {
            return Ok(());
        }
        if write.is_empty() {
            self.blocking_read(addr, read)
        } else if read.is_empty() {
            self.blocking_write(addr, write)
        } else {
            self.blocking_write_read(addr, write, read)
        }
    }
}
