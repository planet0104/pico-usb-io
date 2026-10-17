use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};

use host::{
    Board, Program, SmConfig, PIO_JOIN_RX, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};

const RATE: u32 = 48_000;
const SECONDS: usize = 15;
const GAIN: i32 = 8;
/// 一阶高通（DC 阻断）系数，截止频率约 40 Hz。
const HP_ALPHA: f32 = 0.995;
const BCLK: u8 = 10;
const WS: u8 = 11;
const DATA: u8 = 12;
const PIO: u8 = 0;
const SM: u8 = 0;

/// 每声道只采前 16 个 SCK 的数据，后 16 个只出时钟；每帧 128 个 PIO 周期，
/// autopush 32 位，所以每帧一个 u32：高 16 位左声道，低 16 位右声道。
/// side-set bit0 = SCK，bit1 = WS。
fn i2s_program() -> Program {
    pio::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        "    set x, 15          side 0b00",
        "left_in:",
        "    in pins, 1         side 0b01",
        "    jmp x-- left_in    side 0b00",
        "    set x, 13          side 0b01",
        "left_skip:",
        "    nop                side 0b00",
        "    jmp x-- left_skip  side 0b01",
        "    nop                side 0b00",
        "    nop                side 0b11",
        "    set x, 15          side 0b10",
        "right_in:",
        "    in pins, 1         side 0b11",
        "    jmp x-- right_in   side 0b10",
        "    set x, 13          side 0b11",
        "right_skip:",
        "    nop                side 0b10",
        "    jmp x-- right_skip side 0b11",
        "    nop                side 0b10",
        "    nop                side 0b01",
        ".wrap",
    )
    .program
}

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
    println!("录音 {SECONDS} 秒，采样率 {RATE} Hz，左声道高通滤波（约 40Hz）后放大 {GAIN} 倍，写入 record.wav");
    assert_eq!(WS, BCLK + 1, "side-set 引脚必须相邻：WS 必须是 SCK 的下一号脚");

    let path = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });

    let mut board = match Board::open(&path) {
        Ok(board) => board,
        Err(err) => {
            step(t0, format!("[1/7] 打开串口 {path} 失败: {err}"));
            std::process::exit(1);
        }
    };
    step(t0, format!("[1/7] 串口 {path} 已打开"));

    let version = match board.version() {
        Ok(version) => version,
        Err(err) => {
            step(t0, format!("[2/7] 查询版本失败: {err}"));
            std::process::exit(1);
        }
    };
    step(
        t0,
        format!(
            "[2/7] protocol {} firmware {} pins {}",
            version.protocol, version.firmware, version.pins
        ),
    );

    let prog = match start_i2s(&mut board) {
        Ok(prog) => prog,
        Err(err) => {
            step(t0, format!("[3/7] I2S 启动失败: {err}"));
            std::process::exit(1);
        }
    };
    step(t0, format!("[3/7] I2S 已启动，程序装在 PIO{PIO} 地址 {}", prog.origin));

    println!("等待麦克风启动");
    let primed = discard_until_live(&mut board, t0);
    step(t0, format!("[4/7] 麦克风预热结束，预录 {} 采样", primed.len()));

    println!("开始录音");
    let need = RATE as usize * SECONDS;
    let mut hp = HighPass::new();
    let mut left = Vec::with_capacity(need);
    left.extend(primed.iter().map(|raw| gain(hp.process(*raw as f32))));
    let started = Instant::now();
    let mut read_errors = 0usize;
    let mut clipped = 0usize;
    while left.len() < need && started.elapsed() < Duration::from_secs(SECONDS as u64 + 4) {
        let words = match board.read_stream(Duration::from_secs(2)) {
            Ok(words) => words,
            Err(err) => {
                read_errors += 1;
                step(t0, format!("[5/7] 读取音频失败 (#{read_errors}): {err}，已采集 {} 采样", left.len()));
                if read_errors >= 3 {
                    break;
                }
                continue;
            }
        };
        let before = left.len() / RATE as usize;
        for word in words {
            let out = gain(hp.process((word >> 16) as i16 as f32));
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
    let peak = left.iter().map(|s| (*s as i32).abs()).max().unwrap_or(0);
    step(
        t0,
        format!(
            "[5/7] 录音结束：{} 采样，用时 {:.2}s，峰值 {peak}，削顶 {clipped} 个",
            left.len(),
            started.elapsed().as_secs_f64()
        ),
    );
    if let Ok(status) = board.pio_stream_status() {
        step(t0, format!("[5/7] 板上缓冲丢弃 {} 个字", status.dropped_words));
    }

    match stop_i2s(&mut board, prog) {
        Ok(()) => step(t0, "[6/7] I2S 已停止"),
        Err(err) => step(t0, format!("[6/7] 停止 I2S 失败: {err}")),
    }

    write_wav("record.wav", &left).expect("write wav");
    step(
        t0,
        format!(
            "[7/7] 写好 record.wav，{} 个采样，{:.2} 秒",
            left.len(),
            left.len() as f32 / RATE as f32
        ),
    );
    board.release();
}

fn start_i2s(board: &mut Board) -> Result<host::PioProgram, host::Error> {
    let prog = board.pio_load(PIO, &i2s_program())?;
    let cfg = SmConfig {
        clkdiv: host::clkdiv(RATE * 128),
        in_pins: (DATA, 1),
        sideset_base: BCLK,
        autopush: true,
        push_thresh: 32,
        fifo_join: PIO_JOIN_RX,
        out_pindirs: (1 << BCLK) | (1 << WS),
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg)?;
    board.pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)?;
    board.pio_stream_start(PIO, SM, false, 255)?;
    board.pio_sm_enable(PIO, SM, true)?;
    Ok(prog)
}

/// INMP441 在 SCK/WS 悬空时会进入异常状态，必须断电才能恢复，所以停止后把时钟线拉低。
fn stop_i2s(board: &mut Board, prog: host::PioProgram) -> Result<(), host::Error> {
    board.pio_stream_stop(PIO, SM)?;
    board.pio_sm_enable(PIO, SM, false)?;
    board.pin_output(BCLK, false)?;
    board.pin_output(WS, false)?;
    board.pin_input(DATA)?;
    board.pio_unload(prog)
}

fn discard_until_live(board: &mut Board, t0: Instant) -> Vec<i16> {
    let deadline = Instant::now() + Duration::from_secs(2);
    let mute_need = RATE as usize / 50;
    let live_need = RATE as usize / 100;
    let mut mute = 0usize;
    let mut kept = Vec::new();
    let mut saving = false;
    let mut done = false;
    while Instant::now() < deadline {
        let Ok(words) = board.read_stream(Duration::from_millis(500)) else {
            continue;
        };
        for word in words {
            let raw = (word >> 16) as i16;
            let quiet = amplify(raw).saturating_abs() < 16;
            if saving {
                if !done && quiet {
                    saving = false;
                    mute = 1;
                    kept.clear();
                    continue;
                }
                kept.push(raw);
                done = kept.len() >= live_need;
                continue;
            }
            if quiet {
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
    step(t0, "[4/7] 2s 内未检测到有效声音，按空预录继续");
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
    File::create(path)?.write_all(&bytes)
}
