use std::fs::File;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};

use host::Board;

const RATE: u32 = 48_000;
const SECONDS: usize = 15;
const GAIN: i32 = 8;
/// 一阶高通（DC 阻断）系数，截止频率约 40 Hz。
const HP_ALPHA: f32 = 0.995;
const BCLK: u8 = 10;
const WS: u8 = 11;
const DATA: u8 = 12;

/// 带时间戳的阶段日志，用于定位卡点。
fn step(t0: Instant, msg: impl AsRef<str>) {
    println!("[{:7.3}s] {}", t0.elapsed().as_secs_f64(), msg.as_ref());
}

fn main() {
    let t0 = Instant::now();
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  INMP441 VDD -> 3V3");
    println!("  INMP441 GND -> GND");
    println!("  INMP441 L/R -> GND，麦克风在左声道");
    println!("  INMP441 SCK -> GP{BCLK}");
    println!("  INMP441 WS  -> GP{WS}，必须是 SCK 的下一号脚");
    println!("  INMP441 SD  -> GP{DATA}");
    println!("每个 I2S 帧是一个小端 u32：高 16 位是左声道，低 16 位是右声道");
    println!("录音 {SECONDS} 秒，采样率 {RATE} Hz，左声道高通滤波（约 40Hz）后放大 {GAIN} 倍，写入 record.wav");

    let path = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });

    step(t0, format!("[1/8] 打开串口 {path} ..."));
    let mark = Instant::now();
    let mut board = match Board::open(&path) {
        Ok(board) => board,
        Err(err) => {
            step(t0, format!("[1/8] 打开串口失败 ({:?}): {err}", mark.elapsed()));
            std::process::exit(1);
        }
    };
    step(t0, format!("[1/8] 串口已打开 (+{} ms)", mark.elapsed().as_millis()));

    step(t0, "[2/8] 查询固件版本 ...");
    let mark = Instant::now();
    let version = match board.version() {
        Ok(version) => version,
        Err(err) => {
            step(t0, format!("[2/8] 查询版本失败/超时 ({:?}): {err}", mark.elapsed()));
            std::process::exit(1);
        }
    };
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );
    step(t0, format!("[2/8] 版本已获取 (+{} ms)", mark.elapsed().as_millis()));

    step(t0, "[3/8] 下发 I2S 启动配置 ...");
    let mark = Instant::now();
    if let Err(err) = board.i2s_config(BCLK, WS, DATA, RATE) {
        step(t0, format!("[3/8] I2S 启动失败/超时 ({:?}): {err}", mark.elapsed()));
        std::process::exit(1);
    }
    step(t0, format!("[3/8] I2S 已启动 (+{} ms)", mark.elapsed().as_millis()));

    println!("等待麦克风启动");
    let mark = Instant::now();
    let primed = discard_until_live(&mut board, t0);
    step(
        t0,
        format!(
            "[4/8] 麦克风预热结束，预录 {} 采样 (+{} ms)",
            primed.len(),
            mark.elapsed().as_millis()
        ),
    );

    println!("开始录音");
    let need = RATE as usize * SECONDS;
    let mut hp = HighPass::new();
    let mut left = Vec::with_capacity(need);
    // 预录段(raw)也走高通+增益，避免开机瞬态的直流台阶进入文件。
    left.extend(primed.iter().map(|raw| gain(hp.process(*raw as f32))));
    let started = Instant::now();
    let mut timed_out = false;
    let mut read_errors = 0usize;
    let mut raw_peak = 0f32;
    let mut clipped = 0usize;
    while left.len() < need {
        if started.elapsed() > Duration::from_secs(SECONDS as u64 + 4) {
            timed_out = true;
            break;
        }
        let mark = Instant::now();
        let bytes = match board.read_i2s_push() {
            Ok(bytes) => bytes,
            Err(err) => {
                read_errors += 1;
                step(
                    t0,
                    format!(
                        "[5/8] 读取音频帧失败 (#{read_errors}, 阻塞 {:?}): {err}，已采集 {} 采样",
                        mark.elapsed(),
                        left.len()
                    ),
                );
                // 连续失败说明固件/串口已经不再流数据，直接退出循环写文件。
                if read_errors >= 3 {
                    break;
                }
                continue;
            }
        };
        if bytes.len() < 4 {
            thread::sleep(Duration::from_millis(1));
            continue;
        }
        let before = left.len() / RATE as usize;
        for chunk in bytes.chunks_exact(4) {
            let word = u32::from_le_bytes(chunk.try_into().unwrap());
            let raw = (word >> 16) as i16 as f32;
            raw_peak = raw_peak.max(raw.abs());
            let out = gain(hp.process(raw));
            if out.saturating_abs() >= 32760 {
                clipped += 1;
            }
            left.push(out);
            if left.len() == need {
                break;
            }
        }
        if left.len() / RATE as usize != before {
            println!("已采集 {} / {need}", left.len());
        }
    }
    step(
        t0,
        format!(
            "[5/8] 录音循环结束：{} 采样，用时 {:.2}s{}",
            left.len(),
            started.elapsed().as_secs_f64(),
            if timed_out { "（达到超时上限退出）" } else { "" }
        ),
    );
    let peak = left.iter().map(|s| (*s as i32).abs()).max().unwrap_or(0);
    let nonzero = left.iter().filter(|s| **s != 0).count();
    let mean = if left.is_empty() {
        0
    } else {
        left.iter().map(|s| *s as i64).sum::<i64>() / left.len() as i64
    };
    step(
        t0,
        format!(
            "[5/8] 音频统计：峰值 {peak}，均值 {mean}，非零采样 {nonzero}/{}，削顶 {clipped} 个，原始峰值 {raw_peak:.0}，前 8 个样本 {:?}",
            left.len(),
            &left[..left.len().min(8)]
        ),
    );

    step(t0, "[6/8] 停止 I2S（下发 hz=0）...");
    let mark = Instant::now();
    if let Err(err) = board.i2s_config(BCLK, WS, DATA, 0) {
        step(t0, format!("[6/8] 停止 I2S 失败/超时 ({:?}): {err}", mark.elapsed()));
    } else {
        step(t0, format!("[6/8] I2S 已停止 (+{} ms)", mark.elapsed().as_millis()));
    }

    step(t0, "[7/8] 写入 record.wav ...");
    let mark = Instant::now();
    write_wav("record.wav", &left).expect("write wav");
    step(t0, format!("[7/8] 写文件完成 (+{} ms)", mark.elapsed().as_millis()));
    println!("写好 record.wav，{} 个采样，{:.2} 秒", left.len(), left.len() as f32 / RATE as f32);

    step(t0, "[8/8] 释放串口 / 关闭后台读线程 ...");
    let mark = Instant::now();
    board.release();
    step(t0, format!("[8/8] 释放完成 (+{} ms)", mark.elapsed().as_millis()));
    step(t0, "全部完成，正常退出");
}

fn discard_until_live(board: &mut Board, t0: Instant) -> Vec<i16> {
    let deadline = Instant::now() + Duration::from_secs(2);
    let mute_need = RATE as usize / 50;
    let live_need = RATE as usize / 100;
    let mut mute = 0usize;
    let mut kept = Vec::new();
    let mut saving = false;
    let mut done = false;
    let mut empty_reads = 0usize;
    while Instant::now() < deadline {
        let Ok(bytes) = board.read_i2s_push() else {
            empty_reads += 1;
            continue;
        };
        if bytes.len() < 4 {
            thread::sleep(Duration::from_millis(1));
            continue;
        }
        if !saving && empty_reads > 0 {
            step(
                t0,
                format!("[4/8] 预热期间有 {empty_reads} 次读取超时（固件可能未在推送音频）"),
            );
            empty_reads = 0;
        }
        for chunk in bytes.chunks_exact(4) {
            let word = u32::from_le_bytes(chunk.try_into().unwrap());
            let raw = (word >> 16) as i16;
            let amp = amplify(raw);
            if saving {
                if !done && amp.saturating_abs() < 16 {
                    saving = false;
                    mute = 1;
                    kept.clear();
                    continue;
                }
                kept.push(raw);
                done = kept.len() >= live_need;
                continue;
            }
            if amp.saturating_abs() < 16 {
                mute += 1;
                continue;
            }
            if mute < mute_need {
                mute = 0;
                continue;
            }
            saving = true;
            kept.push(raw);
            done = kept.len() >= live_need;
        }
        if done {
            return kept;
        }
    }
    step(
        t0,
        format!("[4/8] 2s 内未检测到有效声音，按空预录继续（超时读取 {empty_reads} 次）"),
    );
    Vec::new()
}

/// 一阶高通（DC 阻断）滤波器，去除直流偏置与低频轰鸣。
struct HighPass {
    prev_in: f32,
    prev_out: f32,
}

impl HighPass {
    fn new() -> Self {
        Self {
            prev_in: 0.0,
            prev_out: 0.0,
        }
    }

    fn process(&mut self, x: f32) -> f32 {
        let y = x - self.prev_in + HP_ALPHA * self.prev_out;
        self.prev_in = x;
        self.prev_out = y;
        y
    }
}

/// 高通滤波后的浮点样本乘以增益，并饱和到 i16。
fn gain(sample: f32) -> i16 {
    (sample * GAIN as f32).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

fn amplify(sample: i16) -> i16 {
    (sample as i32)
        .saturating_mul(GAIN)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

fn write_wav(path: &str, samples: &[i16]) -> std::io::Result<()> {
    let data_len = (samples.len() * 2) as u32;
    let mut bytes = Vec::with_capacity(44 + samples.len() * 2);
    bytes.extend_from_slice(b"RIFF");
    bytes.extend_from_slice(&(36 + data_len).to_le_bytes());
    bytes.extend_from_slice(b"WAVEfmt ");
    bytes.extend_from_slice(&16u32.to_le_bytes());
    bytes.extend_from_slice(&1u16.to_le_bytes());
    bytes.extend_from_slice(&1u16.to_le_bytes());
    bytes.extend_from_slice(&RATE.to_le_bytes());
    bytes.extend_from_slice(&(RATE * 2).to_le_bytes());
    bytes.extend_from_slice(&2u16.to_le_bytes());
    bytes.extend_from_slice(&16u16.to_le_bytes());
    bytes.extend_from_slice(b"data");
    bytes.extend_from_slice(&data_len.to_le_bytes());
    for sample in samples {
        bytes.extend_from_slice(&sample.to_le_bytes());
    }
    let mut file = File::create(path)?;
    file.write_all(&bytes)?;
    Ok(())
}
