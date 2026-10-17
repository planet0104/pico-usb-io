//! pico-usb-io 固件：把 RP2040 变成一个"USB 转基础 IO"的通用下位机。
//!
//! 固件只做基础通讯，不包含任何具体用途的逻辑：
//! GPIO / PWM / ADC / I2C / SPI / UART 直接对应芯片外设；
//! 需要精确时序的功能（I2S、WS2812、舵机、红外……）由上位机上传 PIO 程序实现，见 pio.rs。
//!
//! 通讯走 USB CDC 串口，帧格式见 protocol crate。整体结构：
//! `usb_reader` 任务只负责收包、解析请求并放进命令队列；
//! 主循环从队列取命令执行并回应答，空闲时把 PIO 流数据作为事件帧推给上位机。
//! 收和发分开，是为了推流时也能及时收到"停止"之类的命令。

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

/// 固件版本，OP_VERSION 应答里返回；2 表示通用 PIO 引擎版本。
const FIRMWARE_VERSION: u16 = 2;

// 中断绑定：USB 控制器中断交给 embassy 的 USB 驱动，DMA_IRQ_0 交给 PIO 流引擎。
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterrupt<USB>;
    DMA_IRQ_0 => pio::StreamDmaHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let adc = Adc::new_blocking(p.ADC, Default::default());
    let mut board = Board::new(adc);

    // USB 设备描述：沿用树莓派的 VID / Pico 的 PID，Windows 会自动识别为 CDC 串口。
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
    // CDC 串口，bulk 端点每包 64 字节（USB 全速的最大值）。
    static USB_STATE: StaticCell<State> = StaticCell::new();
    let class = CdcAcmClass::new(&mut builder, USB_STATE.init(State::new()), 64);
    let usb = builder.build();
    // 拆成发送端和接收端：接收交给独立任务，主循环只管发送。
    let (mut tx, rx) = class.split();
    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(usb_reader(rx).unwrap());

    loop {
        tx.wait_connection().await;
        let mut host_ready = false;
        loop {
            // 上位机以 1200 波特率打开串口是约定的"进入 BOOTSEL 烧录模式"信号（flash.bat 用）。
            if tx.line_coding().data_rate() == 1200 {
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            }
            // DTR 由上位机打开串口时置位、关闭时清除。DTR 掉了说明上位机已断开，
            // 复位 PIO 引擎，免得流还在推、程序残留在指令内存里。
            if !tx.dtr() {
                if host_ready {
                    board.pio.reset();
                    host_ready = false;
                }
                Timer::after(Duration::from_millis(1)).await;
                continue;
            }
            host_ready = true;
            // 命令优先：有命令就执行并回应答。
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
            // 没有命令时推一帧流数据。
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
            // 什么都没有就让出 1 ms，给其他任务运行。
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}

/// 接收任务：把 USB 收到的字节喂给解析器，解出的请求帧放进命令队列。
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

/// 请求帧的拷贝。解析器回调里的 body 只是借用，放进队列前要复制出来。
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

/// 应答 / 事件 body 的公用缓冲区。放在 static 里而不是栈上，避免 1 KB 数组撑爆任务栈；
/// 只有主循环使用，同一时间只有一处在用。
fn resp_body() -> &'static mut [u8; MAX_BODY] {
    static mut BODY: [u8; MAX_BODY] = [0; MAX_BODY];
    unsafe { &mut *core::ptr::addr_of_mut!(BODY) }
}

/// 接收任务和主循环之间的命令队列（4 格环形）。读写都在临界区内进行。
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

/// 放入一条命令；队列满时丢弃，上位机会因为收不到应答而超时。
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

/// USB 断开时清空队列，旧命令不再执行。
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

/// USB 协议栈任务：处理枚举、控制请求等底层事务。
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

/// 表示 USB 已断开、数据发不出去了。
struct Gone;

impl From<EndpointError> for Gone {
    fn from(value: EndpointError) -> Self {
        match value {
            EndpointError::BufferOverflow => panic!("usb buffer"),
            EndpointError::Disabled => Gone,
        }
    }
}

/// 编码一帧并按 64 字节一包发出。总长正好是 64 的整数倍时补一个空包，
/// 否则 Windows 的 CDC 驱动会一直等下一包，以为这次传输还没结束。
async fn write_frame(
    class: &mut Sender<'static, MyUsbDriver>,
    kind: u8,
    seq: u16,
    op: u16,
    status: u8,
    body: &[u8],
) -> Result<(), Gone> {
    // 编码缓冲同样放在 static 里，避免占用栈。
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

/// 发一个 USB 包。上位机不读数据时 `write_packet` 会一直挂起，
/// 所以每 20 ms 醒来检查一次 DTR，上位机已关闭串口就取消传输并放弃。
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

/// 取消所有 IN 端点（设备→电脑方向）上挂着的传输，让端点回到空闲，下次连接能正常收发。
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

/// 每个引脚当前被谁占用。新功能要用某个引脚时，先把它从旧功能上释放（见 `prepare`），
/// 避免同一个引脚同时被两个外设驱动。
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

/// 板子上所有外设的状态。总线外设打开后放在 Option 里，释放时置 None 即关闭。
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
    /// 每条总线当前用的引脚，释放总线时一起把这些引脚设为高阻。
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

    /// 把引脚从当前功能上释放。总线引脚会关闭整条总线；PIO 引脚会停掉用它的状态机。
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

    /// 检查引脚号并释放旧功能，所有"占用引脚"的命令都先调用它。
    fn prepare(&mut self, pin: u8) -> Result<(), u8> {
        if !pins::valid(pin) {
            return Err(ST_BAD_PIN);
        }
        self.release_pin(pin);
        Ok(())
    }

    /// 执行一条命令，返回 (status, 应答 body 长度)。
    async fn handle(&mut self, op: u16, req: &[u8], out: &mut [u8]) -> (u8, usize) {
        match self.dispatch(op, req, out).await {
            Ok(n) => (ST_OK, n),
            Err(status) => (status, 0),
        }
    }

    /// 按操作码分发，未知操作码返回 ST_BAD_ARG。各命令 body 的布局见 README.md 的「操作码」一节。
    async fn dispatch(&mut self, op: u16, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        match op {
            // 应答 [协议版本 u16, 固件版本 u16, 引脚数 u8]
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

    /// OP_PIO_SM_CONFIG：先完整校验参数（不合法时什么都不动），再逐个释放用到的引脚的旧功能，
    /// 交给 PIO 引擎配置，最后把这些引脚标记为 PIO 占用。
    fn pio_sm_config(&mut self, req: &[u8]) -> Result<usize, u8> {
        let mask = self.pio.check_config(req)?;
        for pin in 0..PIN_COUNT {
            if mask & (1 << pin) != 0 {
                self.prepare(pin)?;
            }
        }
        self.pio.sm_config(req)?;
        for pin in 0..PIN_COUNT {
            if mask & (1 << pin) != 0 {
                self.role[pin as usize] = Role::Pio;
            }
        }
        Ok(0)
    }

    /// OP_PIN_CONFIG：body = [引脚, 模式(高阻/输入/输出), 上下拉, 输出电平]。
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

    /// OP_GPIO_WRITE：body = [引脚, 电平]。引脚还不是 GPIO 时自动切成输出。
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

    /// OP_GPIO_READ：body = [引脚]，应答 [电平]。不改变引脚当前功能。
    fn gpio_read(&mut self, req: &[u8], out: &mut [u8]) -> Result<usize, u8> {
        let pin = *req.first().ok_or(ST_BAD_ARG)?;
        if !pins::valid(pin) {
            return Err(ST_BAD_PIN);
        }
        out[0] = u8::from(pins::read(pin));
        Ok(1)
    }

    /// OP_PWM_SET：body = [引脚, 频率 u32, 千分比占空比 u16]。
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

    /// OP_ADC_READ：body = [引脚]，只支持 GP26..GP29，应答 12 位原始值 u16。
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

    /// OP_I2C_CONFIG：body = [SDA, SCL, 频率 u32]。总线号由引脚决定。
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

    /// OP_I2C_TRANSFER：body = [总线, 7 位地址, 写长度 u16, 读长度 u16, 写数据...]，
    /// 先写后读（中间是重复起始条件），应答读到的数据。
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

    /// OP_SPI_CONFIG：body = [SCK, MOSI, MISO(0xFF 表示不接), 频率 u32, 模式 0..3]。
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

    /// OP_SPI_TRANSFER：body = [总线, 数据...]，全双工收发，应答同样长度的接收数据。
    /// 片选由上位机用 GPIO 自己控制。
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

    /// OP_UART_CONFIG：body = [TX, RX, 波特率 u32]。
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

    /// OP_UART_WRITE：body = [总线, 数据...]，阻塞到全部发完。
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

    /// OP_UART_READ：body = [总线, 最多字节数 u16]。直接读硬件接收 FIFO（32 字节深），
    /// 只返回已经收到的，不等待；上位机需要定期轮询。
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

/// 由引脚推出 I2C 总线号：SDA 必须是偶数脚且 SCL = SDA + 1；GP0/1、GP4/5… 属于 I2C0，
/// GP2/3、GP6/7… 属于 I2C1（每 4 个引脚一循环）。
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

/// 由 SCK 引脚推出 SPI 总线号。
fn spi_bus(sck: u8) -> Result<u8, u8> {
    match sck {
        2 | 6 | 18 | 22 => Ok(0),
        10 | 14 | 26 => Ok(1),
        _ => Err(ST_BAD_PIN),
    }
}

/// 由 TX 引脚推出 UART 编号。
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

/// I2C0 和 I2C1 是不同的类型，用这个 trait 让同一段收发代码对两者都能用。
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
