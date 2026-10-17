use std::thread;
use std::time::{Duration, Instant};

use host::{
    Board, Program, SmConfig, PIO_JOIN_NONE, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};

const PIO: u8 = 0;
const SM: u8 = 0;
const DEFAULT_PIN: u8 = 15;

/// 舵机脉宽范围：0.5ms(0°) ~ 2.5ms(180°)，周期 20ms 即 50Hz。
const MIN_US: u32 = 500;
const MAX_US: u32 = 2500;
const PERIOD_US: u32 = 20_000;

/// PIO 时钟 1 MHz，即 1 个 PIO 周期 = 1 µs，所以计数值可直接当微秒用。
const PIO_HZ: u32 = 1_000_000;

/// 舵机程序：每个周期从 TX FIFO 取两个数——先输出 `脉宽` 个周期的高电平，
/// 再输出 `间隔`(= 周期 - 脉宽) 个周期的低电平。上位机自己更新脉宽即改变角度。
/// FIFO 空时 `pull` 会阻塞停住，引脚保持低电平（舵机保持不上力），安全。
fn servo_program() -> Program {
    pio::pio_asm!(
        ".wrap_target",
        "    pull            ",
        "    mov y, osr      ",
        "    set pins, 1     ",
        "width_loop:",
        "    jmp y-- width_loop",
        "    set pins, 0     ",
        "    pull            ",
        "    mov y, osr      ",
        "gap_loop:",
        "    jmp y-- gap_loop",
        ".wrap",
    )
    .program
}

/// 角度(0..180) → 脉宽(µs)
fn angle_to_us(angle_deg: u32) -> u32 {
    let a = angle_deg.min(180);
    MIN_US + (MAX_US - MIN_US) * a / 180
}

/// 在 `duration` 内持续按同一脉宽推 TX FIFO，让舵机保持一个角度。
/// 每轮推一对 [脉宽, 间隔]；推得比舵机消耗略快，FIFO 满时固件会做有界等待。
fn hold(board: &mut Board, width_us: u32, duration: Duration) {
    let gap_us = PERIOD_US - width_us;
    let words = [width_us, gap_us];
    let start = Instant::now();
    while start.elapsed() < duration {
        if let Err(err) = board.pio_tx_push(PIO, SM, &words) {
            eprintln!("推 TX FIFO 失败（忽略继续）: {err}");
        }
        thread::sleep(Duration::from_millis(19));
    }
}

fn main() {
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  舵机信号线接 GP{DEFAULT_PIN}（第二个参数可改），第三个参数是可选的固定角度(0-180)");
    println!("  ⚠ 舵机电源不要接 Pico 的 3V3/5V 引脚，用独立 5V 供电，务必共地");

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
        .unwrap_or(DEFAULT_PIN);
    let angle = std::env::args().nth(3).and_then(|s| s.parse::<u32>().ok());

    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    let prog = board.pio_load(PIO, &servo_program()).expect("load");
    let cfg = SmConfig {
        clkdiv: host::clkdiv(PIO_HZ),
        set_pins: (pin, 1),
        fifo_join: PIO_JOIN_NONE,
        out_pindirs: 1 << pin,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg).expect("config");
    board
        .pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)
        .expect("restart");
    board.pio_sm_enable(PIO, SM, true).expect("enable");

    match angle {
        // 给定角度：转到该角度并保持 2 秒
        Some(a) => {
            println!("转到 {a}°（脉宽 {}µs）", angle_to_us(a));
            hold(&mut board, angle_to_us(a), Duration::from_millis(2000));
        }
        // 未给定角度：扫一遍 0°→180°→0°，最后停在 90°
        None => {
            println!("扫动 0° → 180° → 0°");
            for a in (0..=180).step_by(5) {
                hold(&mut board, angle_to_us(a), Duration::from_millis(40));
            }
            for a in (0..=180).rev().step_by(5) {
                hold(&mut board, angle_to_us(a), Duration::from_millis(40));
            }
            println!("停在 90°");
            hold(&mut board, angle_to_us(90), Duration::from_millis(500));
        }
    }

    board.pio_sm_enable(PIO, SM, false).expect("disable");
    board.pin_output(pin, false).expect("pin low");
    board.pio_unload(prog).expect("unload");
}
