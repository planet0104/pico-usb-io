use std::thread;
use std::time::Duration;

use host::{Board, Program, SmConfig, PIO_JOIN_TX, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM};

const PIO: u8 = 0;
const SM: u8 = 0;

/// pico-examples 的 ws2812 程序：PIO 时钟 8 MHz，每位 10 个周期即 800 kHz。
/// 0 码高 250 ns 低 1000 ns，1 码高 875 ns 低 375 ns；FIFO 空时停在 out，引脚保持低电平即复位。
fn ws2812_program() -> Program {
    pio::pio_asm!(
        ".side_set 1",
        ".wrap_target",
        "bitloop:",
        "    out x, 1        side 0 [2]",
        "    jmp !x do_zero  side 1 [1]",
        "    jmp bitloop     side 1 [4]",
        "do_zero:",
        "    nop             side 0 [4]",
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
        sideset_base: pin,
        autopull: true,
        pull_thresh: 24,
        fifo_join: PIO_JOIN_TX,
        out_pindirs: 1 << pin,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg).expect("config");
    board
        .pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)
        .expect("restart");
    board.pio_sm_enable(PIO, SM, true).expect("enable");

    let colors: &[(&str, [u8; 3])] = &[
        ("red", [0x20, 0x00, 0x00]),
        ("green", [0x00, 0x20, 0x00]),
        ("blue", [0x00, 0x00, 0x20]),
        ("off", [0x00, 0x00, 0x00]),
    ];
    for (name, [a, b, c]) in colors {
        let word = (u32::from(*a) << 24) | (u32::from(*b) << 16) | (u32::from(*c) << 8);
        board.pio_tx_push(PIO, SM, &[word]).expect("ws2812");
        println!("GP{pin} {name}");
        thread::sleep(Duration::from_millis(400));
    }

    board.pio_sm_enable(PIO, SM, false).expect("disable");
    board.pin_output(pin, false).expect("pin low");
    board.pio_unload(prog).expect("unload");
}
