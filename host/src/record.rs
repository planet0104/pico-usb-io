use std::fs::File;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};

use host::Board;

const RATE: u32 = 48_000;
const SECONDS: usize = 5;
const GAIN: i32 = 64;
const BCLK: u8 = 10;
const WS: u8 = 11;
const DATA: u8 = 12;

fn main() {
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  INMP441 VDD -> 3V3");
    println!("  INMP441 GND -> GND");
    println!("  INMP441 L/R -> GND，麦克风在左声道");
    println!("  INMP441 SCK -> GP{BCLK}");
    println!("  INMP441 WS  -> GP{WS}，必须是 SCK 的下一号脚");
    println!("  INMP441 SD  -> GP{DATA}");
    println!("每个 I2S 帧是一个小端 u32：高 16 位是左声道，低 16 位是右声道");
    println!("录音 {SECONDS} 秒，采样率 {RATE} Hz，左声道放大 {GAIN} 倍，写入 record.wav");

    let path = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let mut board = Board::open(&path).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );
    board.i2s_config(BCLK, WS, DATA, RATE).expect("i2s config");
    println!("等待麦克风启动");
    let primed = discard_until_live(&mut board);
    println!("开始录音");

    let need = RATE as usize * SECONDS;
    let mut left = Vec::with_capacity(need);
    left.extend_from_slice(&primed);
    let started = Instant::now();
    while left.len() < need {
        if started.elapsed() > Duration::from_secs((SECONDS as u64) + 4) {
            break;
        }
        let bytes = board.i2s_read(256).expect("i2s read");
        if bytes.len() < 4 {
            thread::sleep(Duration::from_millis(1));
            continue;
        }
        let mark = left.len() / RATE as usize;
        for chunk in bytes.chunks_exact(4) {
            let word = u32::from_le_bytes(chunk.try_into().unwrap());
            left.push(amplify((word >> 16) as i16));
            if left.len() == need {
                break;
            }
        }
        if left.len() / RATE as usize != mark {
            println!("已采集 {} / {need}", left.len());
        }
    }
    board.i2s_config(BCLK, WS, DATA, 0).expect("i2s stop");
    write_wav("record.wav", &left).expect("write wav");
    println!("写好 record.wav，{} 个采样，{:.2} 秒", left.len(), left.len() as f32 / RATE as f32);
}

fn discard_until_live(board: &mut Board) -> Vec<i16> {
    let deadline = Instant::now() + Duration::from_secs(2);
    let mute_need = RATE as usize / 50;
    let live_need = RATE as usize / 100;
    let mut mute = 0usize;
    let mut kept = Vec::new();
    let mut saving = false;
    let mut done = false;
    while Instant::now() < deadline {
        let Ok(bytes) = board.i2s_read(256) else {
            continue;
        };
        if bytes.len() < 4 {
            thread::sleep(Duration::from_millis(1));
            continue;
        }
        for chunk in bytes.chunks_exact(4) {
            let word = u32::from_le_bytes(chunk.try_into().unwrap());
            let sample = amplify((word >> 16) as i16);
            if saving {
                if !done && sample.saturating_abs() < 16 {
                    saving = false;
                    mute = 1;
                    kept.clear();
                    continue;
                }
                kept.push(sample);
                done = kept.len() >= live_need;
                continue;
            }
            if sample.saturating_abs() < 16 {
                mute += 1;
                continue;
            }
            if mute < mute_need {
                mute = 0;
                continue;
            }
            saving = true;
            kept.push(sample);
            done = kept.len() >= live_need;
        }
        if done {
            return kept;
        }
    }
    Vec::new()
}

fn amplify(sample: i16) -> i16 {
    (sample as i32)
        .saturating_mul(GAIN)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

fn write_wav(path: &str, samples: &[i16]) -> std::io::Result<()> {
    let mut file = File::create(path)?;
    let data_len = (samples.len() * 2) as u32;
    let mut hdr = Vec::with_capacity(44);
    hdr.extend_from_slice(b"RIFF");
    hdr.extend_from_slice(&(36 + data_len).to_le_bytes());
    hdr.extend_from_slice(b"WAVEfmt ");
    hdr.extend_from_slice(&16u32.to_le_bytes());
    hdr.extend_from_slice(&1u16.to_le_bytes());
    hdr.extend_from_slice(&1u16.to_le_bytes());
    hdr.extend_from_slice(&RATE.to_le_bytes());
    hdr.extend_from_slice(&(RATE * 2).to_le_bytes());
    hdr.extend_from_slice(&2u16.to_le_bytes());
    hdr.extend_from_slice(&16u16.to_le_bytes());
    hdr.extend_from_slice(b"data");
    hdr.extend_from_slice(&data_len.to_le_bytes());
    file.write_all(&hdr)?;
    for sample in samples {
        file.write_all(&sample.to_le_bytes())?;
    }
    Ok(())
}
