//! 步进电机示例：用 PIO 自动循环输出 28BYJ-48 的四相半拍时序，经 ULN2003 板驱动。
//!
//! 8 个半拍模式直接写死在 PIO 指令内存里循环执行，**完全不占用 FIFO**：
//! 上位机只需装载程序、设好分频（决定转速）、开关状态机，PIO 会一直转下去。
//! 每个半拍固定占 32 个 PIO 周期，所以 转速 = 125MHz / (32 × 分频)。
//!
//! 用法：`stepper.bat COM13 [起始脚] [每秒半拍数 | phase]`

use std::thread;
use std::time::Duration;

use host::{
    Board, Program, SmConfig, PIO_JOIN_NONE, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};

const PIO: u8 = 0;
const SM: u8 = 0;
/// 每个半拍占用的 PIO 周期数（1 条 set + 31 拍空转）
const CYCLES_PER_STEP: u32 = 32;
/// 28BYJ-48 标称 4096 个半拍转一圈（内部减速比约 1:64）
const STEPS_PER_REV: u32 = 4096;
/// 每段走的半拍数
const LEG_STEPS: u32 = STEPS_PER_REV / 4;
/// 分频上限 65535 对应的最慢速度；比这更慢只能靠程序里的空转
const MIN_STEPS_PER_SEC: u32 = 60;

/// 正向 8 拍半步序列（bit0 = IN1）。相邻两拍只变一相，走起来最顺。
fn forward_program() -> Program {
    pio::pio_asm!(
        ".wrap_target",
        "    set pins, 1  [31]", // IN1
        "    set pins, 3  [31]", // IN1+IN2
        "    set pins, 2  [31]", // IN2
        "    set pins, 6  [31]", // IN2+IN3
        "    set pins, 4  [31]", // IN3
        "    set pins, 12 [31]", // IN3+IN4
        "    set pins, 8  [31]", // IN4
        "    set pins, 9  [31]", // IN4+IN1
        ".wrap",
    )
    .program
}

/// 反向：把同样的 8 拍倒过来执行。8 拍是个闭环，倒着走就是反转。
fn reverse_program() -> Program {
    pio::pio_asm!(
        ".wrap_target",
        "    set pins, 9  [31]", // IN4+IN1
        "    set pins, 8  [31]", // IN4
        "    set pins, 12 [31]", // IN3+IN4
        "    set pins, 4  [31]", // IN3
        "    set pins, 6  [31]", // IN2+IN3
        "    set pins, 2  [31]", // IN2
        "    set pins, 3  [31]", // IN1+IN2
        "    set pins, 1  [31]", // IN1
        ".wrap",
    )
    .program
}

/// 走一段：装载程序 → 设好分频 → 开状态机 → 睡够时间 → 关掉、释放引脚。
///
/// 转速由分频精确决定，所以"走 N 个半拍"就等于"转 N/转速 秒"，没必要在 PIO 里数步数。
/// 睡够以后直接停状态机，引脚停在某一相上，这时立刻把它们拉低断电，免得线圈一直通电发热。
fn run_leg(board: &mut Board, base: u8, rate: u32, forward: bool) {
    let prog = if forward {
        board.pio_load(PIO, &forward_program()).expect("load")
    } else {
        board.pio_load(PIO, &reverse_program()).expect("load")
    };
    let cfg = SmConfig {
        clkdiv: host::clkdiv(rate * CYCLES_PER_STEP),
        set_pins: (base, 4),
        fifo_join: PIO_JOIN_NONE,
        out_pindirs: 0xF_u32 << base,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg).expect("config");
    board
        .pio_sm_restart(
            PIO,
            SM,
            PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM,
        )
        .expect("restart");
    board.pio_sm_enable(PIO, SM, true).expect("enable");

    thread::sleep(Duration::from_micros(
        LEG_STEPS as u64 * 1_000_000 / rate as u64,
    ));

    board.pio_sm_enable(PIO, SM, false).expect("disable");
    board.pio_unload(prog).expect("unload");
    for pin in base..base + 4 {
        board.pin_output(pin, false).expect("pin low");
    }
}

/// 逐相慢速通电（不经过 PIO，直接写 GPIO）：每相单独通电 2 秒，按 IN1→IN2→IN3→IN4 再倒回来。
///
/// 单相通电只会让轴移动约 0.18°（28BYJ-48 有减速箱，肉眼看不出位移），
/// 但每换一相，转子都应该有"咔"一下的跳变、并且通电期间轴被磁力吸住、手感有阻力。
/// 某一相通电时轴完全没有反应 → 那一路（线 / ULN2003 通道 / 电机内部线圈）没通。
fn phase_scan(board: &mut Board, base: u8) {
    for p in [1u8, 2, 4, 8, 4, 2, 1, 2, 4, 8] {
        for bit in 0..4u8 {
            board
                .pin_output(base + bit, p & (1 << bit) != 0)
                .expect("pin");
        }
        let name = match p {
            1 => "IN1",
            2 => "IN2",
            4 => "IN3",
            _ => "IN4",
        };
        println!("  {name} 单独通电 2 秒 —— 感受转子有没有一下跳变/被吸住");
        thread::sleep(Duration::from_secs(2));
    }
    for bit in 0..4u8 {
        board.pin_output(base + bit, false).expect("pin");
    }
    println!("逐相扫描结束（已全部断电）。");
}

fn main() {
    let port = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let base: u8 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);
    let arg3 = std::env::args().nth(3).unwrap_or_default();
    // 第三个参数是 "phase" 时进入逐相通电诊断，否则当每秒半拍数
    let phase_mode = arg3 == "phase";
    let rate: u32 = arg3.parse().unwrap_or(400).max(MIN_STEPS_PER_SEC);

    println!("连接（28BYJ-48 四相五线 + ULN2003 驱动板）:");
    println!("  板 IN1 -> GP{base}，IN2 -> GP{}，IN3 -> GP{}，IN4 -> GP{}", base + 1, base + 2, base + 3);
    println!("  板 VCC -> 5V（Pico 第 40 脚 VBUS，或外接 5V 电源并共地）");
    println!("  板 GND -> GND");
    println!("  ⚠ 四个 IN 必须接在连续的四个脚上、且按 IN1..IN4 顺序");
    println!("  ⚠ 电机要 5V 供电，用 3V3 力矩不够，通常只会嗡嗡响不转");

    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    if phase_mode {
        println!("逐相诊断模式：每相单独通电 2 秒");
        phase_scan(&mut board, base);
        return;
    }

    let deg = LEG_STEPS as f64 * 360.0 / STEPS_PER_REV as f64;
    let secs = LEG_STEPS as f64 / rate as f64;
    println!(
        "  每半拍 {CYCLES_PER_STEP} 个周期，分频 {:?} → {rate} 半拍/秒（{:.3}° / 半拍）",
        host::clkdiv(rate * CYCLES_PER_STEP),
        360.0 / STEPS_PER_REV as f64
    );

    for (i, forward) in [true, false, true, false].iter().enumerate() {
        println!(
            "  第 {} 段：{} {deg:.1}°（{LEG_STEPS} 半拍，约 {secs:.1} 秒）",
            i + 1,
            if *forward { "正向" } else { "反向" },
        );
        run_leg(&mut board, base, rate, *forward);
        thread::sleep(Duration::from_millis(400));
    }
    println!("完成；线圈已断电，电机现在可以自由转动。");
}
