//! 电池电压 / 剩电量估算示例：0–25V 检测模块（1:5 分压）接 RP2040 ADC。
//!
//! 直接用两个批处理即可：
//!   battery_li.bat  COM13 [ADC引脚]   —— 单节锂电池（3.0–4.2V）
//!   battery_dry.bat COM13 [ADC引脚]   —— 两节干电池串联（约 2.0–3.2V）
//!
//! 本程序 `battery` 的第一个参数是模式 li|dry，由批处理填好。
//! 电压法只是粗估剩电：必须空载、静置后测量，否则读到的是带载压降后的偏低电压。

use std::thread;
use std::time::{Duration, Instant};

use host::Board;

const ADC_FULL: f32 = 4095.0;
const ADC_VREF: f32 = 3.3;
/// 0–25V 检测模块是 1:5 分压，被测电压 = ADC 端电压 × 5
const DIVIDER: f32 = 5.0;
const PIN_DEFAULT: u8 = 26;
/// 每次取一串样本（丢掉第一个不准的），再平均
const BURST: usize = 16;
/// 测量时长
const SECONDS: u64 = 126;

/// 开路电压(OCV) -> 剩电百分比，按电压降序排列
type Curve = &'static [(f32, f32)];

/// 单节锂电池
const LI: Curve = &[
    (4.20, 100.0),
    (4.00, 85.0),
    (3.85, 70.0),
    (3.70, 50.0),
    (3.60, 30.0),
    (3.45, 15.0),
    (3.30, 5.0),
    (3.00, 0.0),
];

/// 单节碱性干电池
const DRY: Curve = &[
    (1.60, 100.0),
    (1.50, 90.0),
    (1.40, 70.0),
    (1.30, 50.0),
    (1.20, 25.0),
    (1.10, 10.0),
    (0.90, 0.0),
];

/// 干电池脚本按 2 节串联折算单节
const DRY_CELLS: f32 = 2.0;

/// 在对照曲线上做线性插值，超出两端就取端点。
fn soc(v: f32, curve: Curve) -> f32 {
    if v >= curve[0].0 {
        return curve[0].1;
    }
    let last = curve[curve.len() - 1];
    if v <= last.0 {
        return last.1;
    }
    for w in curve.windows(2) {
        let (vh, ph) = w[0];
        let (vl, pl) = w[1];
        if v <= vh && v >= vl {
            return pl + (ph - pl) * (v - vl) / (vh - vl);
        }
    }
    0.0
}

/// 采一串样本并平均，返回 (被测电压 V, 原始值下限, 原始值上限)。
/// 第一个样本受 RP2040 ADC 通道切换影响不准，丢掉。
fn measure(board: &mut Board, pin: u8) -> Result<(f32, u16, u16), host::Error> {
    let mut sum = 0u32;
    let mut n = 0u32;
    let mut lo = u16::MAX;
    let mut hi = 0u16;
    for i in 0..BURST {
        let raw = board.adc_read(pin)?;
        if i == 0 {
            continue;
        }
        sum += raw as u32;
        n += 1;
        lo = lo.min(raw);
        hi = hi.max(raw);
    }
    let avg = (sum / n) as u16;
    let v = avg as f32 / ADC_FULL * ADC_VREF * DIVIDER;
    Ok((v, lo, hi))
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "li".into());
    let port = std::env::args().nth(2).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let pin = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(PIN_DEFAULT);

    let is_dry = mode == "dry";
    let (name, curve, cells) = if is_dry {
        ("两节干电池串联", DRY, DRY_CELLS)
    } else {
        ("单节锂电池", LI, 1.0)
    };

    println!("电池电压 / 剩电量估算（0–25V 检测模块，1:5 分压）");
    println!("  {name}｜串口 {port}｜ADC 引脚 GP{pin}｜采样 {SECONDS} 秒");
    println!("  ⚠ 请空载测量：电池脱离负载、静置几分钟，否则读到的是带载压降后的偏低电压");

    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    let deadline = Instant::now() + Duration::from_secs(SECONDS);
    let mut all = 0f32;
    let mut count = 0u32;

    while Instant::now() < deadline {
        match measure(&mut board, pin) {
            Ok((v, lo, hi)) => {
                let per_cell = v / cells;
                println!(
                    "  {v:.3} V   单节 {per_cell:.3} V   剩电约 {:.0}%   (原始 {lo}..{hi})",
                    soc(per_cell, curve)
                );
                all += v;
                count += 1;
            }
            Err(e) => {
                println!("  ADC 读取失败: {e}");
                break;
            }
        }
        thread::sleep(Duration::from_millis(500));
    }

    if count > 0 {
        let avg = all / count as f32;
        let per_cell = avg / cells;
        println!(
            "结果：平均 {avg:.3} V，单节 {per_cell:.3} V，剩电估算约 {:.0}%",
            soc(per_cell, curve)
        );
        if is_dry {
            println!("（按两节串联、折算到单节 {per_cell:.3} V 估算碱性干电池剩电）");
        } else {
            println!("（锂电池放电平台平坦，电压法误差可能较大，仅供参考）");
        }
    }
}
