use std::collections::VecDeque;
use std::time::{Duration, Instant};

use host::{
    Board, Program, SmConfig, PIO_JOIN_NONE, PIO_RESTART_CLKDIV, PIO_RESTART_FIFOS, PIO_RESTART_SM,
};

const PIO: u8 = 0;
const SM: u8 = 0;
const DEFAULT_PIN: u8 = 15;

/// PIO 跑在 125 MHz 系统时钟上，1 个 PIO 周期 = 8 ns。
const PIO_HZ: u32 = 125_000_000;

/// 接收头输出是开集电极：有 38kHz 载波时拉低，空闲时（靠上拉）为高。
///
/// 同时测量高、低两段，两段每轮都是 2 周期，换算比例一致。
/// 输出流按 高,低,高,低… 严格交替；解码只用高电平段（NEC 的位值编码在间隔里）。
fn ir_program() -> Program {
    pio::pio_asm!(
        ".wrap_target",
        "start:",
        "    wait 1 pin 0        ",  // 进入高电平段
        "    mov x, !null        ",
        "high_count:",
        "    jmp x-- high_test   ",
        "high_test:",
        "    jmp pin high_count  ",  // 仍高 -> 继续计数
        "    mov isr, x          ",  // 变低 -> 本次高电平段结束
        "    push                ",
        "    wait 0 pin 0        ",  // 进入低电平段
        "    mov x, !null        ",
        "low_loop:",
        "    jmp pin low_done    ",  // 变高 -> 本次低电平段结束
        "    jmp x-- low_loop    ",  // 仍低 -> 继续计数（每轮 2 周期）
        "low_done:",
        "    mov isr, x          ",
        "    push                ",
        "    jmp start           ",
        ".wrap",
    )
    .program
}

/// 把 PIO 计数换算成微秒：每轮 2 个周期，x 从 0xFFFFFFFF 递减。
fn duration_us(word: u32) -> u64 {
    let iters = u32::MAX.wrapping_sub(word) as u64;
    iters * 2 / (PIO_HZ as u64 / 1_000_000)
}

/// NEC 命令字节 -> 按键名（按你手上的 21 键遥控器实测标定）
fn nec_name(cmd: u8) -> Option<&'static str> {
    Some(match cmd {
        0x16 => "数字 0",
        0x0C => "数字 1",
        0x18 => "数字 2",
        0x5E => "数字 3",
        0x08 => "数字 4",
        0x1C => "数字 5",
        0x5A => "数字 6",
        0x42 => "数字 7",
        0x52 => "数字 8",
        0x4A => "数字 9",
        0x19 => "100+",
        0x0D => "200+",
        0x15 => "音量 +",
        0x07 => "音量 -",
        0x45 => "频道 -",
        0x46 => "频道",
        0x47 => "频道 +",
        0x44 => "上一曲",
        0x40 => "下一曲",
        0x43 => "播放/暂停",
        0x09 => "EQ",
        _ => return None,
    })
}

struct Decoder {
    /// 只保留高电平段（间隔）
    spaces: VecDeque<u64>,
    last: Option<(u8, Instant)>,
}

impl Decoder {
    fn new() -> Self {
        Self {
            spaces: VecDeque::new(),
            last: None,
        }
    }

    /// 喂入一个高电平段时长（µs）
    fn feed(&mut self, us: u64) {
        // 空闲 / 帧间隔：一段很长的高电平 -> 结算当前帧
        if us > 5_500 {
            let frame = std::mem::take(&mut self.spaces);
            if let Some(cmd) = Self::decode(&frame) {
                self.report(cmd);
            }
            return;
        }
        // NEC 引导间隔（~4.5ms）：作为新帧的首个间隔
        if us >= 2_800 {
            self.spaces.clear();
            self.spaces.push_back(us);
            return;
        }
        self.spaces.push_back(us);
        while self.spaces.len() > 33 {
            self.spaces.pop_front();
        }
    }

    /// NEC：引导间隔 ~4.5ms + 32 个位间隔（~0.56ms = 0，~1.69ms = 1），
    /// 地址与命令都带反码，用反码校验挡掉误识别。
    fn decode(spaces: &VecDeque<u64>) -> Option<u8> {
        if spaces.len() != 33 || !(3_500..=5_500).contains(&spaces[0]) {
            return None;
        }
        let mut code = 0u32;
        for (i, &s) in spaces.iter().skip(1).enumerate() {
            let bit = if s < 1_100 {
                0
            } else if s < 2_800 {
                1
            } else {
                return None;
            };
            code |= (bit as u32) << i;
        }
        let b = code.to_le_bytes(); // [地址, ~地址, 命令, ~命令]
        if (b[0] ^ b[1]) == 0xFF && (b[2] ^ b[3]) == 0xFF {
            return Some(b[2]);
        }
        None
    }

    fn report(&mut self, cmd: u8) {
        // NEC 长按会重复发同一帧，短时间内相同码只报一次
        if let Some((last, t)) = self.last {
            if last == cmd && t.elapsed() < Duration::from_millis(700) {
                return;
            }
        }
        self.last = Some((cmd, Instant::now()));
        match nec_name(cmd) {
            Some(n) => println!("NEC 按键: {n}"),
            None => println!("NEC 按键: 未登记 (命令 0x{cmd:02X})"),
        }
    }
}

fn main() {
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  红外接收头 OUT 接 GP{DEFAULT_PIN}（第二个参数可改），第三个参数是监听秒数");
    println!("  ⚠ 接收头输出需 10kΩ 上拉到 3V3（成品模块通常已内置）");

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
    let seconds: u64 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(30);

    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    // 关键：先把引脚强制回输入，清掉舵机 demo 遗留的输出低状态。
    board.pin_input(pin).expect("pin input");

    let prog = board.pio_load(PIO, &ir_program()).expect("load");
    let cfg = SmConfig {
        clkdiv: host::clkdiv(PIO_HZ),
        in_pins: (pin, 1),
        jmp_pin: Some(pin),
        fifo_join: PIO_JOIN_NONE,
        push_thresh: 32,
        ..Default::default()
    };
    board.pio_sm_config(SM, &prog, &cfg).expect("config");
    board
        .pio_sm_restart(PIO, SM, PIO_RESTART_CLKDIV | PIO_RESTART_FIFOS | PIO_RESTART_SM)
        .expect("restart");

    board
        .pio_stream_start(PIO, SM, true, 255)
        .expect("stream start");
    board.pio_sm_enable(PIO, SM, true).expect("enable");

    println!("对准接收头按遥控器按键，监听 {seconds} 秒...");

    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut decoder = Decoder::new();
    // 输出流按 高,低,高,低… 交替，只用高电平段解码
    let mut is_high = true;

    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        if let Ok(words) = board.read_stream(left.min(Duration::from_millis(200))) {
            for word in words {
                if is_high {
                    decoder.feed(duration_us(word));
                }
                is_high = !is_high;
            }
        }
    }

    board.pio_stream_stop(PIO, SM).expect("stream stop");
    board.pio_sm_enable(PIO, SM, false).expect("disable");
    board.pio_rx_pop(PIO, SM, 255).expect("rx pop");
    board.pin_input(pin).expect("pin input");
    board.pio_unload(prog).expect("unload");
}
