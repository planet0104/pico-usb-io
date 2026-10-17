//! I2C OLED 示例：用固件的 I2C 总线驱动一块 SSD1306 屏（覆盖 I2C 配置 + 读写 op）。
//!
//! 固件对引脚有要求：SCL 必须紧邻 SDA（scl = sda + 1），且 SDA 落在 4 的倍数（I2C0）
//! 或 4k+2（I2C1）上。默认 GP8(SDA) / GP9(SCL)，走 I2C0。
//!
//! 用法：oled.bat COM13 [SDA脚] [SCL脚] [屏高 64|32] [秒数] [旋转 180|0]

use std::thread;
use std::time::{Duration, Instant};

use host::Board;

const SDA_DEFAULT: u8 = 8;
const SCL_DEFAULT: u8 = 9;
/// SSD1306 支持 400 kHz
const HZ: u32 = 400_000;
/// SSD1306 的两个常见地址
const ADDR_A: u8 = 0x3C;
const ADDR_B: u8 = 0x3D;
const W: usize = 128;
const FRAME_MS: u64 = 1;

struct Oled {
    /// 显存：每页 8 行、128 列，共 pages * 128 字节
    fb: Vec<u8>,
    pages: usize,
    /// 整屏旋转 180°，取决于面板的安装方向
    rot: bool,
}

impl Oled {
    fn new(pages: usize, rot: bool) -> Self {
        Self {
            fb: vec![0; W * pages],
            pages,
            rot,
        }
    }

    fn height(&self) -> i32 {
        (self.pages * 8) as i32
    }

    fn clear(&mut self) {
        for b in self.fb.iter_mut() {
            *b = 0;
        }
    }

    fn px(&mut self, x: i32, y: i32) {
        let (x, y) = if self.rot {
            (W as i32 - 1 - x, self.height() - 1 - y)
        } else {
            (x, y)
        };
        if x < 0 || y < 0 || x >= W as i32 || y >= self.height() {
            return;
        }
        self.fb[(y as usize / 8) * W + x as usize] |= 1 << (y as usize % 8);
    }

    fn hline(&mut self, x0: i32, x1: i32, y: i32) {
        for x in x0..=x1 {
            self.px(x, y);
        }
    }

    fn vline(&mut self, x: i32, y0: i32, y1: i32) {
        for y in y0..=y1 {
            self.px(x, y);
        }
    }

    fn rect(&mut self, x0: i32, y0: i32, x1: i32, y1: i32) {
        self.hline(x0, x1, y0);
        self.hline(x0, x1, y1);
        self.vline(x0, y0, y1);
        self.vline(x1, y0, y1);
    }

    fn fill(&mut self, x0: i32, y0: i32, x1: i32, y1: i32) {
        for y in y0..=y1 {
            self.hline(x0, x1, y);
        }
    }
}

/// 发一串命令：首字节 0x00 是"后面都是命令"的控制字节。
fn cmd(board: &mut Board, bus: u8, addr: u8, cmds: &[u8]) -> Result<(), host::Error> {
    let mut buf = Vec::with_capacity(cmds.len() + 1);
    buf.push(0x00);
    buf.extend_from_slice(cmds);
    board.i2c_transfer(bus, addr, &buf, 0)?;
    Ok(())
}

/// 刷新整屏。整屏窗口已在 init 里设过一次：水平寻址模式下写满整个窗口后，
/// 地址指针会自动绕回窗口开头，所以每帧只要把 1024 字节显存发下去就行。
///
/// 这样每帧只有 2 次串口往返（逐页写要 16 次）；往返延迟是本 demo 的主要开销，所以要尽量少。
fn flush(board: &mut Board, bus: u8, addr: u8, oled: &Oled) -> Result<(), host::Error> {
    // 0x40 之后是显存数据；单帧 body 上限 1024 字节，所以每块最多带 1000 字节
    const CHUNK: usize = 1000;
    for part in oled.fb.chunks(CHUNK) {
        let mut buf = Vec::with_capacity(part.len() + 1);
        buf.push(0x40);
        buf.extend_from_slice(part);
        board.i2c_transfer(bus, addr, &buf, 0)?;
    }
    Ok(())
}

/// SSD1306 上电初始化。mux 和 COM 引脚配置随屏高不同。
fn init(board: &mut Board, bus: u8, addr: u8, pages: usize) -> Result<(), host::Error> {
    let mux = (pages * 8 - 1) as u8;
    let compins: u8 = if pages == 4 { 0x02 } else { 0x12 };
    cmd(
        board,
        bus,
        addr,
        &[
            0xAE, // 关闭显示
            0xD5, 0x80, // 时钟分频
            0xA8, mux, // 多路复用比 = 屏高
            0xD3, 0x00, // 显示偏移 0
            0x40, // 起始行 0
            0x8D, 0x14, // 使能内部电荷泵
            0x20, 0x00, // 水平寻址模式（整屏连续写）
            0xA1, // 段重映射
            0xC8, // COM 反向扫描
            0xDA, compins, // COM 引脚配置
            0x81, 0xCF, // 对比度
            0xD9, 0xF1, // 预充电
            0xDB, 0x40, // VCOMH
            0xA4, // 显示跟随显存
            0xA6, // 正常显示（非反色）
            0x2E, // 关闭滚动
            0xAF, // 打开显示
            0x21, 0x00, (W - 1) as u8, // 列窗口 0..127（整屏，只设一次）
            0x22, 0x00, (pages - 1) as u8, // 页窗口 0..pages-1
        ],
    )?;
    Ok(())
}

/// 三角波：让 t 在 0..=limit 之间来回走，到端点反向。
/// 用它替代取模，方块才会"撞边反弹"而不是瞬移回另一侧。
fn bounce(t: i32, limit: i32) -> i32 {
    if limit <= 0 {
        return 0;
    }
    let period = limit * 2;
    let m = t.rem_euclid(period);
    if m <= limit {
        m
    } else {
        period - m
    }
}

fn arg<T: std::str::FromStr>(n: usize, default: T) -> T {
    std::env::args()
        .nth(n)
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

fn main() {
    let port = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let sda = arg(2, SDA_DEFAULT);
    let scl = arg(3, SCL_DEFAULT);
    let height: usize = arg(4, 64usize);
    let secs: u64 = arg(5, 12u64);
    let rot = arg(6, 0u16) != 0;

    if sda.checked_add(1) != Some(scl) || !matches!(sda % 4, 0 | 2) {
        println!("SDA/SCL 组合不合法：固件要求 scl = sda + 1，且 sda % 4 为 0 或 2");
        println!("合法组合 I2C0: 0/1 4/5 8/9 12/13 16/17 20/21 24/25 28/29");
        println!("合法组合 I2C1: 2/3 6/7 10/11 14/15 18/19 22/23 26/27");
        return;
    }
    let pages = if height <= 32 { 4 } else { 8 };
    let bus = if sda % 4 == 0 { 0 } else { 1 };

    println!("连接（SSD1306 I2C 屏，4 线）:");
    println!("  模块 GND -> GND，VCC -> 3V3，SDA -> GP{sda}，SCL -> GP{scl}");
    println!("  屏高 {height}（{pages} 页），I2C{bus}，{HZ} Hz，旋转 {}", if rot { 180 } else { 0 });

    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    board.i2c_config(sda, scl, HZ).expect("i2c config");

    // 0x3C 没应答就试 0x3D（NACK 会被固件报成 ST_IO）
    let mut addr = ADDR_A;
    if let Err(err) = init(&mut board, bus, ADDR_A, pages) {
        println!("0x3C 无应答（{err}），改试 0x3D");
        addr = ADDR_B;
        if let Err(err) = init(&mut board, bus, ADDR_B, pages) {
            println!("0x3C / 0x3D 都无应答（{err}）——检查接线、屏是否为 SSD1306、供电是否 3V3");
            return;
        }
    }
    println!("找到屏，地址 0x{addr:02X}；开始刷新画面 {secs} 秒...");
    println!("  ⚠ 左上角应有一个空心方框（固定不动，是方向标记）；若它在右下角，把最后一个参数改成 180");

    let mut oled = Oled::new(pages, rot);
    let start = Instant::now();
    let mut frame = 0u32;
    let h = oled.height();

    while start.elapsed() < Duration::from_secs(secs) {
        oled.clear();
        oled.rect(0, 0, W as i32 - 1, h - 1); // 外框
        oled.rect(3, 3, 14, 14); // 左上角空心方框：方向标记，始终不动
        for x in 0..W as i32 {
            let d = 2 + x * (h - 4) / W as i32;
            oled.px(x, d); // 对角线
            oled.px(x, h - 3 - x * (h - 4) / W as i32); // 反对角线
        }
        // 经典"弹球"路线：横竖同速（约 45° 斜走），撞到边就反弹
        let bx = 4 + bounce(frame as i32 * 2, W as i32 - 22);
        let by = 6 + bounce(frame as i32 * 2, h - 20);
        oled.fill(bx, by, bx + 17, by + 11); // 移动的方块（撞边反弹，不瞬移）
        let bar = (frame as i32 * 3) % (W as i32 - 8);
        oled.fill(4, h - 5, 4 + bar, h - 3); // 底部进度条

        match flush(&mut board, bus, addr, &oled) {
            Ok(()) => {}
            Err(err) => {
                println!("写入失败（第 {frame} 帧）: {err}");
                return;
            }
        }

        frame += 1;
        if frame % 25 == 0 {
            println!("  已刷新 {frame} 帧（{:.1}s）", start.elapsed().as_secs_f32());
        }
        thread::sleep(Duration::from_millis(FRAME_MS));
    }

    println!("结束：共 {frame} 帧，屏幕保持最后一帧。");
}
