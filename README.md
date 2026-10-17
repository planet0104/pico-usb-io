# pico-usb-io

把 RP2040 开发板变成一个通过 USB 串口控制的通用 IO 下位机。

固件只做基础通讯，不包含任何具体用途的逻辑：

- **基础外设**：GPIO、PWM、ADC、I2C、SPI、UART，一条命令对应一次硬件操作。
- **通用 PIO**：需要精确时序的功能（I2S 麦克风、WS2812 灯、舵机、红外接收……）由电脑上传 PIO 程序实现。固件只负责装载程序、配置状态机、收发数据，时序由 RP2040 的 PIO 硬件保证，不受电脑调度抖动影响。

所以新增一种外设通常不需要改固件，只要在上位机写一段 PIO 程序。

---

## 目录

| 路径 | 内容 |
|---|---|
| `firmware/` | RP2040 固件（Rust + embassy） |
| `protocol/` | 固件和上位机共用的帧协议与常量 |
| `host/` | Rust 上位机库 `host` 和示例程序 |
| `*.bat` | Windows 下一键编译、烧录、运行示例的脚本 |
| `pio-refactor.md` | 通用 PIO 引擎的设计文档和协议细节 |

---

## 快速开始

### 1. 准备环境

```text
rustup target add thumbv6m-none-eabi
```

烧录脚本需要 `elf2uf2-rs`，没有时 `flash.bat` 会自动 `cargo install`。

### 2. 烧录固件

```text
.\flash.bat COM13
```

脚本会编译固件、生成 `firmware.uf2`，再找名为 `RPI-RP2` 的 U 盘并复制进去。

- **第一次烧录**（或板子里不是本固件）：按住 BOOTSEL 再插 USB，板子会出现 `RPI-RP2` 盘，然后运行 `flash.bat`。
- **已经是本固件**：脚本会用 1200 波特率打开一次串口，固件收到后自动进入 BOOTSEL，无需按键。
- **只编译不烧录**：`.\flash.bat build`。

### 3. 找到串口号

插上板子后在设备管理器的「端口 (COM 和 LPT)」里查看。所有示例不带参数时默认用 `COM5`，端口不对会报 `version: Timeout(1)`。Linux 下默认 `/dev/ttyACM0`。

### 4. 运行示例

| 命令 | 作用 | 接线 |
|---|---|---|
| `.\blink.bat COM13` | GP25 闪 3 次（官方 Pico 的板载 LED） | 无 |
| `.\ws2812.bat COM13 [引脚]` | 板载 RGB 灯依次红、绿、蓝、灭 | 默认 GP16 |
| `.\record.bat COM13` | INMP441 麦克风录 15 秒，写 `record.wav` | SCK→GP10，WS→GP11，SD→GP12，L/R→GND |
| `.\servo.bat COM13 [引脚] [角度]` | 舵机转到指定角度；不给角度则来回扫动 | 信号线默认 GP15，舵机用独立 5 V 供电并共地 |
| `.\ir.bat COM13 [引脚] [秒数]` | 监听红外遥控器，解码 NEC 按键 | 接收头 OUT 默认 GP15，需上拉到 3V3，默认监听 15 秒 |

`.bat` 实际执行的是 `cargo run -p host --bin <名字> -- <参数>`，Linux / macOS 下直接用这条命令。

---

## 在自己的 Rust 程序里使用

### 引入依赖

```toml
[dependencies]
host = { path = "路径/pico-usb-io/host" }
pio = "0.3"          # 只有用到 PIO 时才需要
```

### 基础外设

```rust
use host::Board;

fn main() -> Result<(), host::Error> {
    let mut board = Board::open("COM13")?;
    let v = board.version()?;
    println!("protocol {} firmware {}", v.protocol, v.firmware);

    board.pin_output(25, false)?;               // GP25 设为输出低
    board.gpio_write(25, true)?;                // 写高
    let level = board.gpio_read(7)?;            // 读 GP7

    board.pwm_set(15, 1_000, 250)?;             // GP15：1 kHz，占空比 25.0%
    let raw = board.adc_read(26)?;              // GP26 的 12 位 ADC 值

    board.i2c_config(4, 5, 100_000)?;           // SDA=GP4, SCL=GP5 → 总线 0
    let id = board.i2c_transfer(0, 0x68, &[0x75], 1)?;   // 写寄存器地址后读 1 字节

    board.spi_config(18, 19, Some(16), 1_000_000, 0)?;   // SCK, MOSI, MISO, 频率, 模式 → 总线 0
    board.gpio_write(17, false)?;               // 片选自己用 GPIO 控制
    let rx = board.spi_transfer(0, &[0x9F, 0, 0, 0])?;
    board.gpio_write(17, true)?;

    board.uart_config(8, 9, 115_200)?;          // TX=GP8, RX=GP9 → UART1
    board.uart_write(1, b"hello")?;
    let got = board.uart_read(1, 64)?;          // 只取已收到的，不等待

    board.release();                            // 关闭串口（Drop 时也会自动关闭）
    Ok(())
}
```

每个方法都会发一条命令并等待应答（最多 3 秒），失败返回 `host::Error`：

| 错误 | 含义 |
|---|---|
| `Error::Status(n)` | 固件拒绝了命令，`n` 见下文「状态码」 |
| `Error::Timeout(seq)` | 超时没收到应答，通常是端口号不对或板子没在运行本固件 |
| `Error::Serial` / `Error::Io` | 串口打不开或读写出错 |
| `Error::Short` | 应答长度不对（固件和上位机版本不匹配） |

### 引脚规则

每个引脚同一时间只能有一个功能。给引脚设置新功能时，固件会自动释放旧功能：

- 引脚原来属于 I2C、SPI、UART 时，会关闭整条总线。
- 引脚原来属于 PIO 时，会停掉用它的状态机。

| 外设 | 可用引脚 |
|---|---|
| ADC | GP26 ~ GP29 |
| I2C | SDA 为偶数脚、SCL = SDA + 1；SDA % 4 == 0 是总线 0（GP0/1、4/5、8/9…），== 2 是总线 1（GP2/3、6/7…） |
| SPI0 | SCK ∈ {2, 6, 18, 22}，MOSI ∈ {3, 7, 19, 23}，MISO ∈ {0, 4, 16, 20} 或不接 |
| SPI1 | SCK ∈ {10, 14, 26}，MOSI ∈ {11, 15, 27}，MISO ∈ {8, 12, 24, 28} 或不接 |
| UART0 | TX ∈ {0, 12, 16, 28}，RX ∈ {1, 13, 17, 29} |
| UART1 | TX ∈ {4, 8, 20, 24}，RX ∈ {5, 9, 21, 25} |
| PWM | 任意引脚，频率约 8 Hz ~ 数十 MHz |

使用 PWM 时注意：相邻两个引脚共用一个 PWM 单元，例如 GP14/15、GP16/17，GP16 起编号循环。改其中一个引脚的频率，另一个也会跟着变，占空比则各自独立。

### 用 PIO 实现自定义时序

RP2040 有两个 PIO 块（`pio` = 0 或 1），每块 4 个状态机（`sm` = 0~3），共享 32 条指令。

使用流程固定为以下几步：

1. `pio_load`：上传程序。
2. `pio_sm_config`：配置状态机。
3. `pio_sm_restart`：复位状态机。
4. 按需开始收发：输出数据用 `pio_tx_push`；连续接收先 `pio_stream_start`。
5. `pio_sm_enable(on)`：启动状态机。
6. 结束：停流、停状态机、用 `pin_output` / `pin_input` 把引脚设到确定状态，最后 `pio_unload`。

下面这个例子在 GP15 上输出 1 kHz 方波：

```rust
use host::{Board, SmConfig, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM};
use std::{thread, time::Duration};

fn main() -> Result<(), host::Error> {
    let pin = 15;
    let mut board = Board::open("COM13")?;

    // 高 32 周期、低 32 周期，一圈 64 个 PIO 周期。
    let program = pio::pio_asm!(
        ".wrap_target",
        "    set pins, 1 [31]",
        "    set pins, 0 [31]",
        ".wrap",
    )
    .program;
    let prog = board.pio_load(0, &program)?;

    let cfg = SmConfig {
        clkdiv: host::clkdiv(64 * 1_000),   // PIO 时钟 64 kHz → 方波 1 kHz
        set_pins: (pin, 1),                 // `set pins` 控制 GP15
        out_pindirs: 1 << pin,              // GP15 设为输出
        ..Default::default()
    };
    board.pio_sm_config(0, &prog, &cfg)?;
    board.pio_sm_restart(0, 0, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)?;
    board.pio_sm_enable(0, 0, true)?;

    thread::sleep(Duration::from_secs(3));

    board.pio_sm_enable(0, 0, false)?;
    board.pin_output(pin, false)?;          // 停止后引脚保持最后电平，这里明确拉低
    board.pio_unload(prog)?;
    Ok(())
}
```

**往状态机送数据**：`board.pio_tx_push(pio, sm, &words)`。它会自动分帧；FIFO 满时固件会等待，每帧最多等 50 ms。参考 `ws2812.rs`、`servo.rs`。

**从状态机收数据**：

- 偶尔读一次：`pio_rx_pop(pio, sm, 最多字数)`。
- 持续接收：用流，同一时间只能有一个流。

```rust
board.pio_stream_start(0, 0, false, 255)?;   // 必须在使能状态机之前
board.pio_sm_enable(0, 0, true)?;
loop {
    let words: Vec<u32> = board.read_stream(Duration::from_secs(2))?;
    // 处理数据……
}
// 结束时：
board.pio_stream_stop(0, 0)?;                // 固件会同时停用该状态机
let st = board.pio_stream_status()?;         // st.dropped_words > 0 说明读得太慢丢过数据
```

流有两种模式：

- **连续模式**（`sparse = false`）：用 DMA 搬运，板上有约 64 KB 缓冲，适合 I2S 这类高速数据，参考 `record.rs`。
- **稀疏模式**（`sparse = true`）：有数据才发，适合红外、按键边沿这类偶发数据，参考 `ir.rs`。

`SmConfig` 各字段的含义见 `host/src/lib.rs` 里的注释；分频用 `host::clkdiv(目标频率)` 计算，系统时钟为 125 MHz。

#### PIO 使用注意事项

- **引脚范围**：side-set 和 OUT / SET 的引脚必须连续。例如 I2S 的 WS 必须是 SCK 的下一号脚；`set_pins` 最多 5 个。
- **指令内存**：每个 PIO 块共 32 条指令，多个程序共用；用完要 `pio_unload`。
- **停止后的引脚**：状态机停止后引脚保持最后电平，不会自动变成高阻。需要确定电平时自己用 `pin_output` / `pin_input` 设置；比如 INMP441 在时钟线悬空时会出异常。
- **连续流的速度**：上位机要持续读取。48 kHz 每帧 4 字节（192 KB/s）的 I2S 已实测连续 15 秒无丢失；更高速率需自行测试，并用 `pio_stream_status` 检查丢弃计数。

### 断开与异常退出

- 上位机打开串口时会置 DTR，固件据此判断已连接。
- 程序退出或崩溃后 DTR 消失，固件会自动停掉所有状态机和流，并清空 PIO 指令内存；重新运行程序即可，不需要重插 USB。
- 同一个串口同一时间只能被一个程序打开。

---

## 用其他语言直接调用（串口协议）

任何能打开串口的语言都可以控制板子，Rust 库只是协议的一种封装。

### 串口设置

- **连接方式**：USB CDC 虚拟串口，波特率可以随便填，但**不要用 1200**（固件会进入烧录模式）。
- **DTR**：打开后必须置 DTR（pyserial 默认会置），否则固件不处理命令。
- **持续读串口**：固件推流时会不停发数据，程序要一直读串口，否则电脑端缓冲满了，你发的命令也会被卡住。

### 帧格式

所有多字节字段都是小端：

```text
A5 5A | kind(u8) seq(u16) op(u16) status(u8) len(u16) | body(len 字节) | crc16(u16)
```

| 字段 | 说明 |
|---|---|
| `kind` | 1 = 请求（电脑→板），2 = 应答（板→电脑），3 = 事件（板主动发） |
| `seq` | 请求序号，应答带回同一个值；事件固定为 0 |
| `op` | 操作码，见下表 |
| `status` | 请求填 0；应答里是状态码 |
| `len` | body 长度，最大 1024 |
| `crc16` | CRC-16/MODBUS（初值 0xFFFF，多项式 0xA001 反射），覆盖 kind 到 body 末尾 |

### Python 示例

```python
import struct
import serial  # pip install pyserial

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def make_frame(seq, op, body=b""):
    head = struct.pack("<BHHBH", 1, seq, op, 0, len(body))
    return b"\xA5\x5A" + head + body + struct.pack("<H", crc16(head + body))

def read_frame(port):
    while True:
        if port.read(1) != b"\xA5" or port.read(1) != b"\x5A":
            continue
        head = port.read(8)
        kind, seq, op, status, n = struct.unpack("<BHHBH", head)
        body = port.read(n)
        (crc,) = struct.unpack("<H", port.read(2))
        if crc == crc16(head + body):
            return kind, seq, op, status, body

def call(port, seq, op, body=b""):
    port.write(make_frame(seq, op, body))
    while True:
        kind, rseq, rop, status, rbody = read_frame(port)
        if kind == 2 and rseq == seq:
            if status != 0:
                raise RuntimeError(f"firmware status {status}")
            return rbody

port = serial.Serial("COM13", 115200, timeout=1)
proto, fw, pins = struct.unpack("<HHB", call(port, 1, 1))          # OP_VERSION
print("protocol", proto, "firmware", fw, "pins", pins)
call(port, 2, 2, bytes([25, 2, 0, 0]))                             # GP25 设为输出低
call(port, 3, 3, bytes([25, 1]))                                   # GP25 写高
```

### 操作码

基础外设（body 中 u16 / u32 均为小端）：

| op | 名称 | 请求 body | 应答 body |
|---|---|---|---|
| 1 | VERSION | 空 | 协议版本 u16，固件版本 u16，引脚数 u8 |
| 2 | PIN_CONFIG | 引脚，模式（0 高阻 / 1 输入 / 2 输出），上下拉（0 无 / 1 上拉 / 2 下拉），输出电平 | 空 |
| 3 | GPIO_WRITE | 引脚，电平 | 空 |
| 4 | GPIO_READ | 引脚 | 电平 |
| 5 | PWM_SET | 引脚，频率 u32，占空比千分比 u16 | 空 |
| 6 | ADC_READ | 引脚（26~29） | 12 位值 u16 |
| 7 | I2C_CONFIG | SDA，SCL，频率 u32（10 k~1 M） | 空 |
| 8 | I2C_TRANSFER | 总线，7 位地址，写长度 u16，读长度 u16，写数据 | 读到的数据 |
| 9 | SPI_CONFIG | SCK，MOSI，MISO（0xFF 不接），频率 u32，模式 0~3 | 空 |
| 10 | SPI_TRANSFER | 总线，数据 | 同样长度的接收数据 |
| 11 | UART_CONFIG | TX，RX，波特率 u32（300~3 M） | 空 |
| 12 | UART_WRITE | 总线，数据 | 空 |
| 13 | UART_READ | 总线，最多字节数 u16 | 已收到的数据（可能为空） |

通用 PIO（pio 为 0/1，sm 为 0~3）：

| op | 名称 | 请求 body | 应答 body |
|---|---|---|---|
| 20 | PIO_LOAD | pio，flags（bit0 自动分配地址），起始地址，wrap_target，wrap，条数，指令 u16 × 条数 | 实际起始地址，条数 |
| 21 | PIO_UNLOAD | pio，起始地址，条数 | 空 |
| 22 | PIO_SM_CONFIG | 固定 24 字节，见 `pio-refactor.md` 4.2 节 | 空 |
| 23 | PIO_SM_ENABLE | pio，sm，0/1 | 空 |
| 24 | PIO_SM_RESTART | pio，sm，flags（bit0 分频相位，bit1 清 FIFO，bit2 状态机内部状态） | 空 |
| 25 | PIO_TX_PUSH | pio，sm，flags（bit0 满时等待），个数（1~255），u32 × 个数 | 实际写入个数 |
| 26 | PIO_RX_POP | pio，sm，最多个数 | u32 序列 |
| 27 | PIO_STREAM_START | pio，sm，flags（bit0 稀疏模式），每帧最多个数（0 表示 255） | 空 |
| 28 | PIO_STREAM_STOP | pio，sm | 空 |
| 29 | PIO_STREAM_STATUS | 空 | 运行中 u8，缓冲未读字数 u32，已丢弃字数 u32 |
| 30 | PIO_STREAM_EVENT | 仅用于事件帧（kind = 3），body 是流数据 u32 序列 | — |

PIO 程序按相对地址汇编上传，固件会把 JMP 目标和 wrap 加上实际装载地址。PIO 指令编码见 RP2040 数据手册第 3.4 节，也可以用 [pioasm](https://github.com/raspberrypi/pico-sdk/tree/master/tools/pioasm) 或 Rust 的 `pio` crate 生成。

### 状态码

| 值 | 名称 | 常见原因 |
|---|---|---|
| 0 | OK | |
| 1 | BAD_FRAME | 保留，当前固件不会返回；CRC 错误的帧会被直接丢弃，表现为等不到应答 |
| 2 | BAD_PIN | 引脚号无效，或该引脚组合不支持这个外设 |
| 3 | BAD_ARG | 参数超范围、长度不对、未知操作码、总线未打开 |
| 4 | BUSY | 已有流在运行；卸载的程序仍被运行中的状态机使用 |
| 5 | IO | 外设操作失败（I2C 无应答、ADC 读取失败等） |
| 6 | NO_SPACE | PIO 指令内存不够 |

Rust 库在电脑端还会自己产生两种状态码：`pio_tx_push` 一个字都写不进去时返回 BUSY（通常是状态机没在运行）；请求 body 超过 1024 字节时返回 NO_SPACE。

---

## 常见问题

**报 `version: Timeout(1)`**
端口号不对（默认是 COM5），或板子里不是本固件。在设备管理器确认端口，再把端口作为第一个参数传入。

**串口完全没反应，重新运行也不行**
先关掉所有占用该串口的程序，再重新插拔 USB。

**录音有断续**
运行结束时会打印「板上缓冲丢弃 N 个字」。N > 0 说明电脑读得不够快，避免在读流的循环里做耗时操作。

**固件和上位机版本**
`version()` 返回的协议版本为 2、固件版本为 2。旧版固件（含 I2S / pulse / bitstream 专用命令）与当前上位机不兼容，两边需要一起更新。
