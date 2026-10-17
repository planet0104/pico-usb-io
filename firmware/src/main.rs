#![no_std]
#![no_main]

mod buses;
mod pins;
mod pio;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::adc::{Adc, Channel};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterrupt};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;
use io_proto::{
    read_u16, read_u32, Frame, Parser, KIND_EVENT, KIND_REQ, KIND_RESP, MAX_BODY, MAX_FRAME,
    OP_ADC_READ, OP_GPIO_READ, OP_GPIO_WRITE, OP_I2C_CONFIG, OP_I2C_TRANSFER, OP_PIN_CONFIG,
    OP_PIO_LOAD, OP_PIO_RX_POP, OP_PIO_SM_CONFIG, OP_PIO_SM_ENABLE, OP_PIO_SM_RESTART,
    OP_PIO_STREAM_EVENT, OP_PIO_STREAM_START, OP_PIO_STREAM_STATUS, OP_PIO_STREAM_STOP,
    OP_PIO_TX_PUSH, OP_PIO_UNLOAD, OP_PWM_SET, OP_SPI_CONFIG, OP_SPI_TRANSFER, OP_UART_CONFIG,
    OP_UART_READ, OP_UART_WRITE, OP_VERSION, PIN_COUNT, PIN_HIZ, PIN_INPUT, PIN_OUTPUT,
    PROTO_VERSION, PULL_DOWN, PULL_UP, SPI_MISO_NONE, ST_BAD_ARG, ST_BAD_PIN, ST_IO, ST_OK,
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const FIRMWARE_VERSION: u16 = 2;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterrupt<USB>;
    DMA_IRQ_0 => pio::StreamDmaHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let adc = Adc::new_blocking(p.ADC, Default::default());
    let mut board = Board::new(adc);

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
    let class = CdcAcmClass::new(&mut builder, USB_STATE.init(State::new()), 64);
    let usb = builder.build();
    let (mut tx, rx) = class.split();
    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(usb_reader(rx).unwrap());

    loop {
        tx.wait_connection().await;
        let mut host_ready = false;
        loop {
            if tx.line_coding().data_rate() == 1200 {
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            }
            if !tx.dtr() {
                if host_ready {
                    board.pio.reset();
                    host_ready = false;
                }
                Timer::after(Duration::from_millis(1)).await;
                continue;
            }
            host_ready = true;
            if let Some(cmd) = pop_cmd() {
                let body = resp_body();
                let (status, n) = board.handle(cmd.op, &cmd.body[..cmd.body_len], body).await;
                if write_frame(&mut tx, KIND_RESP, cmd.seq, cmd.op, status, &body[..n])
                    .await
                    .is_err()
                {
                    clear_cmds();
                    break;
                }
                continue;
            }
            let body = resp_body();
            let n = board.pio.stream_read(body);
            if n > 0 {
                if write_frame(&mut tx, KIND_EVENT, 0, OP_PIO_STREAM_EVENT, ST_OK, &body[..n])
                    .await
                    .is_err()
                {
                    clear_cmds();
                    break;
                }
                continue;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}

#[embassy_executor::task]
async fn usb_reader(mut rx: Receiver<'static, MyUsbDriver>) -> ! {
    loop {
        rx.wait_connection().await;
        let mut parser = Parser::new();
        let mut packet = [0u8; 64];
        loop {
            match rx.read_packet(&mut packet).await {
                Ok(n) => {
                    parser.push(&packet[..n], |frame| {
                        if frame.kind == KIND_REQ {
                            push_cmd(&frame);
                        }
                    });
                }
                Err(_) => {
                    clear_cmds();
                    break;
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct OwnedFrame {
    seq: u16,
    op: u16,
    body: [u8; MAX_BODY],
    body_len: usize,
}

impl OwnedFrame {
    const EMPTY: Self = Self {
        seq: 0,
        op: 0,
        body: [0; MAX_BODY],
        body_len: 0,
    };

    fn from_frame(frame: &Frame<'_>) -> Self {
        let mut body = [0u8; MAX_BODY];
        let n = frame.body.len().min(MAX_BODY);
        body[..n].copy_from_slice(&frame.body[..n]);
        Self {
            seq: frame.seq,
            op: frame.op,
            body,
            body_len: n,
        }
    }
}

fn resp_body() -> &'static mut [u8; MAX_BODY] {
    static mut BODY: [u8; MAX_BODY] = [0; MAX_BODY];
    unsafe { &mut *core::ptr::addr_of_mut!(BODY) }
}

struct CmdQ {
    buf: [OwnedFrame; 4],
    head: usize,
    tail: usize,
    len: usize,
}

fn cmd_q() -> *mut CmdQ {
    static mut Q: CmdQ = CmdQ {
        buf: [OwnedFrame::EMPTY; 4],
        head: 0,
        tail: 0,
        len: 0,
    };
    core::ptr::addr_of_mut!(Q)
}

fn push_cmd(frame: &Frame) {
    cortex_m::interrupt::free(|_| unsafe {
        let q = &mut *cmd_q();
        if q.len >= q.buf.len() {
            return;
        }
        q.buf[q.tail] = OwnedFrame::from_frame(frame);
        q.tail = (q.tail + 1) % q.buf.len();
        q.len += 1;
    });
}

fn pop_cmd() -> Option<OwnedFrame> {
    cortex_m::interrupt::free(|_| unsafe {
        let q = &mut *cmd_q();
        if q.len == 0 {
            return None;
        }
        let item = q.buf[q.head];
        q.head = (q.head + 1) % q.buf.len();
        q.len -= 1;
        Some(item)
    })
}

fn clear_cmds() {
    cortex_m::interrupt::free(|_| unsafe {
        let q = &mut *cmd_q();
        q.head = 0;
        q.tail = 0;
        q.len = 0;
    });
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

async fn write_frame(
    class: &mut Sender<'static, MyUsbDriver>,
    kind: u8,
    seq: u16,
    op: u16,
    status: u8,
    body: &[u8],
) -> Result<(), Gone> {
    static mut RAW: [u8; MAX_FRAME] = [0; MAX_FRAME];
    let raw = unsafe { &mut *core::ptr::addr_of_mut!(RAW) };
    let Some(n) = io_proto::encode(
        Frame {
            kind,
            seq,
            op,
            status,
            body,
        },
        raw,
    ) else {
        return Ok(());
    };
    let mut sent = 0;
    while sent < n {
        let end = (sent + 64).min(n);
        write_packet(class, &raw[sent..end]).await?;
        sent = end;
    }
    if n % 64 == 0 {
        write_packet(class, &[]).await?;
    }
    Ok(())
}

async fn write_packet(class: &mut Sender<'static, MyUsbDriver>, data: &[u8]) -> Result<(), Gone> {
    loop {
        if !class.dtr() {
            abort_in_transfers();
            return Err(Gone);
        }
        match select(class.write_packet(data), Timer::after(Duration::from_millis(20))).await {
            Either::First(result) => return result.map_err(Into::into),
            Either::Second(_) => {}
        }
    }
}

fn abort_in_transfers() {
    let usb = embassy_rp::pac::USB;
    usb.ep_abort().write(|w| {
        for n in 1..16 {
            w.set_ep_in(n, true);
        }
    });
    for _ in 0..10000 {
        let done = usb.ep_abort_done().read();
        if (1..16).all(|n| done.ep_in(n)) {
            break;
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Role {
    Free,
    Gpio,
    Pwm,
    I2c(u8),
    Spi(u8),
    Uart(u8),
    Pio,
}

struct Board {
    adc: Adc<'static, embassy_rp::adc::Blocking>,
    pio: pio::Engine,
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
    fn new(adc: Adc<'static, embassy_rp::adc::Blocking>) -> Self {
        Self {
            adc,
            pio: pio::Engine::new(),
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
            Role::Pio => self.pio.release_pin(pin),
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
                out[2..4].copy_from_slice(&FIRMWARE_VERSION.to_le_bytes());
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
            OP_PIO_LOAD => self.pio.load(req, out),
            OP_PIO_UNLOAD => self.pio.unload(req),
            OP_PIO_SM_CONFIG => self.pio_sm_config(req),
            OP_PIO_SM_ENABLE => self.pio.sm_enable(req),
            OP_PIO_SM_RESTART => self.pio.sm_restart(req),
            OP_PIO_TX_PUSH => self.pio.tx_push(req, out).await,
            OP_PIO_RX_POP => self.pio.rx_pop(req, out),
            OP_PIO_STREAM_START => self.pio.stream_start(req),
            OP_PIO_STREAM_STOP => self.pio.stream_stop(req),
            OP_PIO_STREAM_STATUS => self.pio.stream_status(out),
            _ => Err(ST_BAD_ARG),
        }
    }

    fn pio_sm_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let mask = pio::config_pins(req)?;
        for pin in 0..PIN_COUNT {
            if mask & (1 << pin) != 0 {
                self.prepare(pin)?;
            }
        }
        self.pio.sm_config(req, mask)?;
        for pin in 0..PIN_COUNT {
            if mask & (1 << pin) != 0 {
                self.role[pin as usize] = Role::Pio;
            }
        }
        Ok(0)
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
