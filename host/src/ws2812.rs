//! WS2812 示例：让板载 RGB 灯依次显示红、绿、蓝，然后熄灭。
//!
//! WS2812 用单根数据线、按高电平宽度区分 0 和 1，时序要求在百纳秒级，
//! 所以用 PIO 程序产生波形，上位机只需往 TX FIFO 推颜色数据。
//!
//! 用法：`ws2812.bat COM13`，第二个参数可改数据脚（默认 GP16）。

use std::thread;
use std::time::Duration;

use host::{Board, Program, SmConfig, PIO_JOIN_TX, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM};

/// 使用 PIO0 的状态机 0。
const PIO: u8 = 0;
const SM: u8 = 0;

/// pico-examples 的 ws2812 程序：PIO 时钟 8 MHz，每位 10 个周期即 800 kHz。
/// 0 码高 250 ns 低 1000 ns，1 码高 875 ns 低 375 ns；FIFO 空时停在 out，引脚保持低电平即复位。
/// `[n]` 表示这条指令执行后再多等 n 个周期。
fn ws2812_program() -> Program {
    pio::pio_asm!(
        ".side_set 1",
        ".wrap_target",
        "bitloop:",
        "    out x, 1        side 0 [2]", // 取一位到 x，数据线低 3 个周期
        "    jmp !x do_zero  side 1 [1]", // 数据线高 2 个周期；这一位是 0 就跳走
        "    jmp bitloop     side 1 [4]", // 1 码：继续保持高 5 个周期
        "do_zero:",
        "    nop             side 0 [4]", // 0 码：提前拉低 5 个周期
        ".wrap",
    )
    .program
}

fn main() {
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  板载 WS2812 接 GP16，第二个参数可以改数据脚");
    let port = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let pin = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(16u8);
    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    let prog = board.pio_load(PIO, &ws2812_program()).expect("load");
    let cfg = SmConfig {
        clkdiv: host::clkdiv(8_000_000),
        // 数据线由 side-set 驱动。
        sideset_base: pin,
        // 每个灯 24 位颜色：OSR 移出 24 位后自动取下一个字；默认左移，从字的最高位开始发。
        autopull: true,
        pull_thresh: 24,
        // 只发不收，把 RX FIFO 并给 TX，变成 8 级。
        fifo_join: PIO_JOIN_TX,
        out_pindirs: 1 << pin,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg).expect("config");
    board
        .pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)
        .expect("restart");
    board.pio_sm_enable(PIO, SM, true).expect("enable");

    // 标准 WS2812 是 GRB 顺序，但这块板子的灯实测是 RGB 顺序，所以按 [R, G, B] 写。
    let colors: &[(&str, [u8; 3])] = &[
        ("red", [0x20, 0x00, 0x00]),
        ("green", [0x00, 0x20, 0x00]),
        ("blue", [0x00, 0x00, 0x20]),
        ("off", [0x00, 0x00, 0x00]),
    ];
    for (name, [a, b, c]) in colors {
        // 三个颜色字节放在 bit31..8，低 8 位不会被发出。
        let word = (u32::from(*a) << 24) | (u32::from(*b) << 16) | (u32::from(*c) << 8);
        board.pio_tx_push(PIO, SM, &[word]).expect("ws2812");
        println!("GP{pin} {name}");
        thread::sleep(Duration::from_millis(400));
    }

    // 收尾：停状态机、数据线拉低、释放指令内存。
    board.pio_sm_enable(PIO, SM, false).expect("disable");
    board.pin_output(pin, false).expect("pin low");
    board.pio_unload(prog).expect("unload");
}
