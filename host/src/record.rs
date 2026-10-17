//! I2S 录音示例：从 INMP441 数字麦克风录一段 WAV。
//!
//! 固件不认识 I2S，这里自己写一个 PIO 程序产生 SCK / WS 时钟并采样 SD 数据线，
//! 用连续流把采样实时传回电脑，再做高通滤波、放大，写成 16 位单声道 WAV。
//!
//! 用法：`record.bat COM13`，或 `cargo run -p host --bin record -- COM13`。

use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};

use host::{
    Board, Program, SmConfig, PIO_JOIN_RX, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};

/// 采样率。
const RATE: u32 = 48_000;
/// 录音时长（秒）。
const SECONDS: usize = 15;
/// 数字增益。INMP441 灵敏度较低，说话声原始幅度很小；增益太大会把底噪一起放大。
const GAIN: i32 = 8;
/// 一阶高通（DC 阻断）系数，截止频率约 40 Hz。
const HP_ALPHA: f32 = 0.995;
/// 麦克风接线：SCK、WS 由 side-set 驱动，必须是相邻引脚。
const BCLK: u8 = 10;
const WS: u8 = 11;
const DATA: u8 = 12;
/// 使用 PIO0 的状态机 0。
const PIO: u8 = 0;
const SM: u8 = 0;

/// I2S 接收程序。PIO 时钟 = 采样率 × 128，每个 SCK 周期占 2 个 PIO 周期，
/// 每声道 32 个 SCK，一帧（左 + 右）共 128 个 PIO 周期。
///
/// INMP441 输出 24 位数据，我们只要高 16 位：每声道只在前 16 个 SCK 采样，
/// 后 16 个只出时钟不采样。autopush 阈值 32，所以每帧正好一个 u32：
/// 高 16 位是左声道，低 16 位是右声道，带宽只有每声道 32 位时的一半。
///
/// side-set 2 位：bit0 = SCK，bit1 = WS。在 SCK 为高时采样（麦克风在 SCK 下降沿更新数据）。
/// WS 比声道数据提前一个 SCK 切换，这是 I2S 标准时序。
fn i2s_program() -> Program {
    pio::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        // 左声道（WS = 0）：采 16 位
        "    set x, 15          side 0b00",
        "left_in:",
        "    in pins, 1         side 0b01", // SCK 高，采一位
        "    jmp x-- left_in    side 0b00", // SCK 低
        // 左声道剩下的 16 个 SCK：只出时钟
        "    set x, 13          side 0b01",
        "left_skip:",
        "    nop                side 0b00",
        "    jmp x-- left_skip  side 0b01",
        "    nop                side 0b00",
        "    nop                side 0b11", // 左声道最后一个 SCK，WS 提前变高
        // 右声道（WS = 1）：同样采 16 位
        "    set x, 15          side 0b10",
        "right_in:",
        "    in pins, 1         side 0b11",
        "    jmp x-- right_in   side 0b10",
        "    set x, 13          side 0b11",
        "right_skip:",
        "    nop                side 0b10",
        "    jmp x-- right_skip side 0b11",
        "    nop                side 0b10",
        "    nop                side 0b01", // 右声道最后一个 SCK，WS 提前变低
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

    // 麦克风上电 / 唤醒后会先输出一段全 0，有的模块还会先"哒"一声，跳过这段再开始计时。
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
    // 多给 4 秒余量，防止流异常时无限等待。
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
            // 高 16 位是左声道（L/R 接 GND 的麦克风在左声道），按有符号数解释。
            let out = gain(hp.process((word >> 16) as i16 as f32));
            if out.saturating_abs() >= 32760 {
                clipped += 1;
            }
            left.push(out);
            if left.len() == need {
                break;
            }
        }
        // 每满一秒打印一次进度。
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

/// 装载 I2S 程序并开始推流。
fn start_i2s(board: &mut Board) -> Result<host::PioProgram, host::Error> {
    let prog = board.pio_load(PIO, &i2s_program())?;
    let cfg = SmConfig {
        clkdiv: host::clkdiv(RATE * 128),
        // SD 数据线作为 `in pins` 的输入。
        in_pins: (DATA, 1),
        // side-set 从 SCK 开始，2 位即 SCK、WS。
        sideset_base: BCLK,
        // 攒够 32 位（一帧）自动推入 RX FIFO；左移，先采的位在高位。
        autopush: true,
        push_thresh: 32,
        // 只收不发，把 TX FIFO 并给 RX，变成 8 级。
        fifo_join: PIO_JOIN_RX,
        out_pindirs: (1 << BCLK) | (1 << WS),
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg)?;
    board.pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)?;
    // 先开流再使能状态机，否则 FIFO 满了 PIO 会停下来，开头的时钟就不连续。
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

/// 丢掉麦克风启动阶段的数据：先要看到连续 20 ms 的静音（启动时的全 0 输出），
/// 再看到连续 10 ms 的有效声音，才认为麦克风已正常工作，返回从有效声音开始到
/// 当前这批数据结束的原始采样（至少 10 ms）。
/// 2 秒内没等到就返回空，直接开始录。
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
                // 有效声音还没攒够 10 ms 就又出现静音，说明是启动杂音，重新等。
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
            // 之前的静音不够长，这次的声音不算数。
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

    /// y[n] = x[n] - x[n-1] + α·y[n-1]
    fn process(&mut self, x: f32) -> f32 {
        let y = x - self.prev_in + HP_ALPHA * self.prev_out;
        self.prev_in = x;
        self.prev_out = y;
        y
    }
}

/// 浮点样本乘增益并限幅到 i16，超出范围的会被削顶。
fn gain(sample: f32) -> i16 {
    (sample * GAIN as f32).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// 整数版增益，只用于启动检测时判断静音。
fn amplify(sample: i16) -> i16 {
    (sample as i32)
        .saturating_mul(GAIN)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// 写 16 位单声道 PCM WAV。先在内存里拼好再一次性写盘，几十万个采样逐个写会非常慢。
fn write_wav(path: &str, samples: &[i16]) -> std::io::Result<()> {
    let data_len = (samples.len() * 2) as u32;
    let mut bytes = Vec::with_capacity(44 + samples.len() * 2);
    // 44 字节标准文件头
    bytes.extend_from_slice(b"RIFF");
    bytes.extend_from_slice(&(36 + data_len).to_le_bytes());
    bytes.extend_from_slice(b"WAVEfmt ");
    bytes.extend_from_slice(&16u32.to_le_bytes()); // fmt 块长度
    bytes.extend_from_slice(&1u16.to_le_bytes()); // PCM
    bytes.extend_from_slice(&1u16.to_le_bytes()); // 单声道
    bytes.extend_from_slice(&RATE.to_le_bytes()); // 采样率
    bytes.extend_from_slice(&(RATE * 2).to_le_bytes()); // 每秒字节数
    bytes.extend_from_slice(&2u16.to_le_bytes()); // 每个采样的字节数
    bytes.extend_from_slice(&16u16.to_le_bytes()); // 位深
    bytes.extend_from_slice(b"data");
    bytes.extend_from_slice(&data_len.to_le_bytes());
    for sample in samples {
        bytes.extend_from_slice(&sample.to_le_bytes());
    }
    File::create(path)?.write_all(&bytes)
}
