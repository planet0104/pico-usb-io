//! GPS 定位测试：读取 DX-GP22-A 输出的 NMEA 语句，每秒打印一次定位状态和经纬度。
//!
//! 接线见 pinout.md：W -> GP2，P -> GP3，R -> GP4（UART1 TX），T -> GP5（UART1 RX），
//! V -> 5V（VBUS），G -> GND。
//!
//! 用法：`gps.bat COM13 [秒数] [波特率]`。不给波特率时依次试 9600 和 115200。

use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};

use host::{Board, Error};

/// 模块 W 脚（ON/OFF，低电平关断），拉高让模块工作。
const PIN_EN: u8 = 2;
const PIN_TX: u8 = 4;
const PIN_RX: u8 = 5;
/// TX 接 GP4 时固件用的是 UART1。
const UART: u8 = 1;
const BAUDS: [u32; 2] = [9600, 115_200];
/// 固件 UART 只有 32 字节硬件缓冲，9600 波特率约 33 ms 填满，要比这更勤地读。
const POLL: Duration = Duration::from_millis(10);
/// 开头原样打印几条语句，方便确认波特率和数据格式。
const RAW_LINES: usize = 5;

/// 从 NMEA 语句里汇总出的当前状态。
#[derive(Default)]
struct Fix {
    /// UTC 时间 hhmmss
    time: String,
    valid: bool,
    lat: Option<f64>,
    lon: Option<f64>,
    speed_kmh: Option<f64>,
    used: u32,
    alt: Option<f64>,
    /// 每个卫星系统（GP=GPS、BD/GB=北斗、GL=GLONASS、GA=伽利略）可见的卫星数
    in_view: HashMap<String, u32>,
}

/// 把串口收到的字节拼成整行（NMEA 每条语句以 \r\n 结尾）。
struct LineReader {
    buf: Vec<u8>,
}

impl LineReader {
    fn poll(&mut self, board: &mut Board) -> Result<Vec<String>, Error> {
        self.buf.extend(board.uart_read(UART, 256)?);
        let mut lines = Vec::new();
        while let Some(i) = self.buf.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.buf.drain(..=i).collect();
            lines.push(String::from_utf8_lossy(&line).trim().to_string());
        }
        Ok(lines)
    }
}

/// 校验 `$...*hh`：`$` 和 `*` 之间所有字节异或等于 hh。通过时返回中间的内容。
fn checked(line: &str) -> Option<&str> {
    let body = line.strip_prefix('$')?;
    let (data, sum) = body.split_once('*')?;
    let want = u8::from_str_radix(sum.get(..2)?, 16).ok()?;
    let got = data.bytes().fold(0u8, |a, b| a ^ b);
    (got == want).then_some(data)
}

/// NMEA 坐标是"度分"格式（纬度 ddmm.mmmm，经度 dddmm.mmmm），转成十进制度；南纬、西经为负。
fn coord(value: &str, hemi: &str) -> Option<f64> {
    let v: f64 = value.parse().ok()?;
    let deg = (v / 100.0).floor();
    let d = deg + (v - deg * 100.0) / 60.0;
    Some(if hemi == "S" || hemi == "W" { -d } else { d })
}

fn update(fix: &mut Fix, data: &str) {
    let f: Vec<&str> = data.split(',').collect();
    let Some(id) = f.first().filter(|id| id.len() == 5) else {
        return;
    };
    let (talker, kind) = id.split_at(2);
    match kind {
        // RMC：时间、是否有效、经纬度、速度（节）
        "RMC" if f.len() > 7 => {
            fix.time = f[1].get(..6).unwrap_or("").to_string();
            fix.valid = f[2] == "A";
            fix.lat = coord(f[3], f[4]);
            fix.lon = coord(f[5], f[6]);
            fix.speed_kmh = f[7].parse::<f64>().ok().map(|knots| knots * 1.852);
        }
        // GGA：参与定位的卫星数、海拔
        "GGA" if f.len() > 9 => {
            fix.used = f[7].parse().unwrap_or(0);
            fix.alt = f[9].parse().ok();
        }
        // GSV：该卫星系统可见卫星总数
        "GSV" if f.len() > 3 => {
            if let Ok(n) = f[3].parse() {
                fix.in_view.insert(talker.to_string(), n);
            }
        }
        _ => {}
    }
}

/// UTC hhmmss 转成北京时间 hh:mm:ss。
fn beijing(time: &str) -> String {
    let Some(h) = time.get(..2).and_then(|h| h.parse::<u32>().ok()) else {
        return "--:--:--".into();
    };
    format!("{:02}:{}:{}", (h + 8) % 24, &time[2..4], &time[4..6])
}

/// 依次试每个波特率，2.5 秒内收到校验正确的语句就认为对了。
fn detect_baud(board: &mut Board, reader: &mut LineReader) -> Result<Option<u32>, Error> {
    for baud in BAUDS {
        board.uart_config(PIN_TX, PIN_RX, baud)?;
        reader.buf.clear();
        let deadline = Instant::now() + Duration::from_millis(2500);
        while Instant::now() < deadline {
            if reader.poll(board)?.iter().any(|l| checked(l).is_some()) {
                return Ok(Some(baud));
            }
            thread::sleep(POLL);
        }
        println!("  {baud} 波特率下没有收到有效语句");
    }
    Ok(None)
}

fn main() -> Result<(), Error> {
    let port = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(120);
    let baud: Option<u32> = std::env::args().nth(3).and_then(|s| s.parse().ok());

    println!("接线：W->GP{PIN_EN}，R->GP{PIN_TX}，T->GP{PIN_RX}，V->5V，G->GND（P 秒脉冲不用）");
    let mut board = Board::open(&port)?;
    board.pin_output(PIN_EN, true)?;
    let mut reader = LineReader { buf: Vec::new() };

    let baud = match baud {
        Some(b) => {
            board.uart_config(PIN_TX, PIN_RX, b)?;
            b
        }
        None => match detect_baud(&mut board, &mut reader)? {
            Some(b) => b,
            None => {
                println!("没有收到 GPS 数据：检查 T->GP{PIN_RX} 接线、V 是否接 5V、W 是否被拉低");
                return Ok(());
            }
        },
    };
    println!("串口 UART{UART} {baud} 波特率，监听 {secs} 秒（天线朝上、放窗边或室外）");

    let start = Instant::now();
    let mut fix = Fix::default();
    let mut raw = 0;
    let mut last_print = Instant::now();
    let mut last_data = Instant::now();
    let mut first_fix: Option<Duration> = None;

    while start.elapsed() < Duration::from_secs(secs) {
        for line in reader.poll(&mut board)? {
            let Some(data) = checked(&line) else {
                continue;
            };
            last_data = Instant::now();
            if raw < RAW_LINES {
                println!("  原始: {line}");
                raw += 1;
            }
            update(&mut fix, data);
        }

        if last_print.elapsed() >= Duration::from_secs(1) {
            last_print = Instant::now();
            let view: u32 = fix.in_view.values().sum();
            if last_data.elapsed() > Duration::from_secs(3) {
                println!("超过 3 秒没有收到数据，检查接线和供电");
            } else if fix.valid {
                if first_fix.is_none() {
                    first_fix = Some(start.elapsed());
                    println!("首次定位用时 {:.0} 秒", start.elapsed().as_secs_f64());
                }
                println!(
                    "{} 已定位  纬度 {:.6}  经度 {:.6}  海拔 {} m  速度 {:.1} km/h  卫星 {}/{}",
                    beijing(&fix.time),
                    fix.lat.unwrap_or(0.0),
                    fix.lon.unwrap_or(0.0),
                    fix.alt.map_or("-".into(), |a| format!("{a:.1}")),
                    fix.speed_kmh.unwrap_or(0.0),
                    fix.used,
                    view,
                );
            } else {
                println!("{} 搜星中  可见卫星 {view}  参与定位 {}", beijing(&fix.time), fix.used);
            }
        }
        thread::sleep(POLL);
    }

    match (first_fix, fix.lat, fix.lon) {
        (Some(t), Some(lat), Some(lon)) => println!(
            "结束：首次定位 {:.0} 秒，最后位置 {lat:.6}, {lon:.6}（WGS84 坐标）",
            t.as_secs_f64()
        ),
        _ => println!("结束：未能定位。室内通常搜不到星，请把天线放到窗边或室外再试"),
    }
    Ok(())
}
