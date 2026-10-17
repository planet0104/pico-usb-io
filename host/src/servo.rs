//! 舵机示例：用 PIO 产生 50 Hz 舵机脉冲，转到指定角度或来回扫动。
//!
//! 用法：`servo.bat COM13 [信号脚] [角度]`，不给角度时 0°→180°→0° 扫一遍后停在 90°。

use std::thread;
use std::time::Duration;

use host::{
    Board, Program, SmConfig, PIO_JOIN_NONE, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};

/// 使用 PIO0 的状态机 0。
const PIO: u8 = 0;
const SM: u8 = 0;
const DEFAULT_PIN: u8 = 15;

/// 舵机脉宽范围：0.5ms(0°) ~ 2.5ms(180°)，周期 20ms 即 50Hz。
const MIN_US: u32 = 500;
const MAX_US: u32 = 2500;
const PERIOD_US: u32 = 20_000;

/// PIO 时钟 1 MHz，即 1 个 PIO 周期 = 1 µs，所以计数值可直接当微秒用。
const PIO_HZ: u32 = 1_000_000;

/// 舵机程序：每个周期取一个字，低 16 位是高电平计数，高 16 位是低电平计数。
/// `pull noblock` 在 FIFO 空时把 X（上一次的字）复制进 OSR，所以上位机只需在角度变化时推一次，
/// 电脑定时抖动不会让脉冲中断。启动前必须先推一个字，否则 X=0 会输出几微秒周期的窄脉冲。
/// 每周期 8 条额外指令：高电平 = 计数 + 3 µs，低电平 = 计数 + 5 µs。
fn servo_program() -> Program {
    pio::pio_asm!(
        ".wrap_target",
        "    pull noblock",        // 有新数据就取新的，没有就复用 X 里的上一个字
        "    mov x, osr",          // 保存这个字，供下一周期复用
        "    out y, 16",           // 低 16 位：高电平计数
        "    set pins, 1",
        "width_loop:",
        "    jmp y-- width_loop",  // 每圈 1 µs
        "    out y, 16",           // 高 16 位：低电平计数
        "    set pins, 0",
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

/// 把脉宽打包成程序要的一个字：扣掉程序本身的指令开销，保证脉宽和 20 ms 周期都准确。
fn pulse_word(width_us: u32) -> u32 {
    let high = width_us - 3;
    let low = PERIOD_US - width_us - 5;
    (low << 16) | high
}

/// 推一次新脉宽，PIO 会一直重复它，然后等待 `duration`。
fn hold(board: &mut Board, width_us: u32, duration: Duration) {
    if let Err(err) = board.pio_tx_push(PIO, SM, &[pulse_word(width_us)]) {
        eprintln!("推 TX FIFO 失败: {err}");
    }
    thread::sleep(duration);
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
        // 信号线由 `set pins` 驱动。
        set_pins: (pin, 1),
        // 右移：`out y, 16` 先取低 16 位，再取高 16 位。
        out_shift_right: true,
        fifo_join: PIO_JOIN_NONE,
        out_pindirs: 1 << pin,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg).expect("config");
    board
        .pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)
        .expect("restart");
    // 使能前必须先推一个字，否则第一次 `pull noblock` 拿到的是 X = 0，会输出极窄的脉冲。
    let first = angle_to_us(angle.unwrap_or(0));
    board.pio_tx_push(PIO, SM, &[pulse_word(first)]).expect("first pulse");
    board.pio_sm_enable(PIO, SM, true).expect("enable");

    match angle {
        // 给定角度：转到该角度并保持 2 秒
        Some(a) => {
            println!("转到 {a}°（脉宽 {first}µs）");
            thread::sleep(Duration::from_millis(2000));
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

    // 收尾：停状态机（可能停在高电平中途），所以再把信号线拉低，最后释放指令内存。
    board.pio_sm_enable(PIO, SM, false).expect("disable");
    board.pin_output(pin, false).expect("pin low");
    board.pio_unload(prog).expect("unload");
}
