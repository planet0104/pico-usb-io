//! 步进电机示例：28BYJ-48 + ULN2003 驱动板，正转 90°、反转 90°，来回两次。
//!
//! 8 个半拍直接写在 PIO 程序里循环执行，不占用 FIFO；转速只由分频决定：
//! 每个半拍 32 个 PIO 周期，转速 = 125 MHz / (32 × 分频)。
//!
//! 用法：`stepper.bat COM13 [IN1 引脚] [每秒半拍数]`，IN1..IN4 接连续的四个引脚。

use std::thread;
use std::time::Duration;

use host::{Board, Program, SmConfig, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM};

const PIO: u8 = 0;
const SM: u8 = 0;
/// 每个半拍的 PIO 周期数（1 条 set + 31 拍延时）。
const CYCLES_PER_STEP: u32 = 32;
/// 28BYJ-48 转一圈 4096 个半拍（内部减速比约 1:64）。
const STEPS_PER_REV: u32 = 4096;
/// 每段走 1/4 圈。
const LEG_STEPS: u32 = STEPS_PER_REV / 4;
/// 分频最大 65535，对应最慢约 60 半拍/秒。
const MIN_STEPS_PER_SEC: u32 = 60;

/// 8 拍半步序列，bit0 = IN1；`reverse` 为 true 时倒序执行即反转。
fn program(reverse: bool) -> Program {
    if reverse {
        pio::pio_asm!(
            ".wrap_target",
            "    set pins, 9  [31]",
            "    set pins, 8  [31]",
            "    set pins, 12 [31]",
            "    set pins, 4  [31]",
            "    set pins, 6  [31]",
            "    set pins, 2  [31]",
            "    set pins, 3  [31]",
            "    set pins, 1  [31]",
            ".wrap",
        )
        .program
    } else {
        pio::pio_asm!(
            ".wrap_target",
            "    set pins, 1  [31]", // IN1
            "    set pins, 3  [31]", // IN1 + IN2
            "    set pins, 2  [31]", // IN2
            "    set pins, 6  [31]", // IN2 + IN3
            "    set pins, 4  [31]", // IN3
            "    set pins, 12 [31]", // IN3 + IN4
            "    set pins, 8  [31]", // IN4
            "    set pins, 9  [31]", // IN4 + IN1
            ".wrap",
        )
        .program
    }
}

/// 转一段：启动状态机，按转速睡够 LEG_STEPS 个半拍的时间，再停下并给线圈断电。
fn run_leg(board: &mut Board, base: u8, rate: u32, reverse: bool) -> Result<(), host::Error> {
    let prog = board.pio_load(PIO, &program(reverse))?;
    let cfg = SmConfig {
        clkdiv: host::clkdiv(rate * CYCLES_PER_STEP),
        set_pins: (base, 4),
        out_pindirs: 0xF << base,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg)?;
    board.pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)?;
    board.pio_sm_enable(PIO, SM, true)?;
    thread::sleep(Duration::from_micros(LEG_STEPS as u64 * 1_000_000 / rate as u64));
    board.pio_sm_enable(PIO, SM, false)?;
    board.pio_unload(prog)?;
    // 停下时某一相还通着电，拉低避免线圈持续发热。
    for pin in base..base + 4 {
        board.pin_output(pin, false)?;
    }
    Ok(())
}

fn main() {
    let port = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let base: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2);
    let rate: u32 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(400)
        .max(MIN_STEPS_PER_SEC);

    println!("连接：IN1..IN4 -> GP{base}..GP{}，VCC -> 5V（VBUS），GND -> GND", base + 3);
    println!("转速 {rate} 半拍/秒，每段 90°");

    let mut board = Board::open(&port).expect("open serial");
    for reverse in [false, true, false, true] {
        println!("{}", if reverse { "反转 90°" } else { "正转 90°" });
        run_leg(&mut board, base, rate, reverse).expect("stepper");
        thread::sleep(Duration::from_millis(400));
    }
    println!("完成，线圈已断电");
}
