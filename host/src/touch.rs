//! 触摸按钮示例：轮询 GPIO 电平，检测"按下 / 松开"事件（覆盖固件的 GPIO 读 op）。
//!
//! 常见模块（TTP223 一类）：SIG -> 信号脚，VCC -> 3V3，GND -> GND。
//! 出厂默认"触摸时 SIG 输出高"；若你的模块被焊成低有效，第三个参数写 low。
//!
//! 用法：touch.bat COM13 [信号脚] [low]

use std::thread;
use std::time::{Duration, Instant};

use host::Board;

/// 空闲引脚（GP15/16 已被舵机、红外、WS2812 占用）
const PIN_DEFAULT: u8 = 14;
/// 轮询间隔
const POLL_MS: u64 = 10;
/// 电平要连续稳定这么多个采样才认账，用于消抖
const DEBOUNCE: u32 = 3;

fn main() {
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
        .unwrap_or(PIN_DEFAULT);
    let active_low = matches!(std::env::args().nth(3).as_deref(), Some("low"));

    println!("连接:");
    println!("  触摸模块 SIG -> GP{pin}（第二个参数可改），VCC -> 3V3，GND -> GND");
    println!("  出厂默认触摸时输出高；若是低有效，加第三个参数 low");

    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    // 配成输入（模块是推挽输出，无需上下拉）
    board.pin_input(pin).expect("pin input");

    // 用首个读数当初始状态，免得一启动就误报一次"按下"
    let mut pressed = match board.gpio_read(pin) {
        Ok(v) => v != active_low,
        Err(e) => {
            println!("读取失败: {e}");
            return;
        }
    };

    println!(
        "开始轮询 GP{pin}，每 {POLL_MS} ms 一次；Ctrl-C 结束。初始电平: {}",
        if pressed { "有效(已按下)" } else { "无效(未按下)" }
    );

    let mut stable = 0u32;
    let mut count = 0u32;
    let start = Instant::now();

    loop {
        let raw = match board.gpio_read(pin) {
            Ok(v) => v,
            Err(e) => {
                println!("读取失败: {e}");
                break;
            }
        };
        let active = raw != active_low;

        if active == pressed {
            stable = 0;
        } else {
            stable += 1;
            if stable >= DEBOUNCE {
                pressed = active;
                stable = 0;
                let t = start.elapsed().as_secs_f32();
                if pressed {
                    count += 1;
                    println!("[{t:7.2}s] 按下   (#{count})");
                } else {
                    println!("[{t:7.2}s] 松开");
                }
            }
        }

        thread::sleep(Duration::from_millis(POLL_MS));
    }
}
