//! ST7789 240×240 SPI TFT 示例：用固件的 SPI 总线写屏（覆盖 SPI 配置 + 传输 op）。
//!
//! 屏幕是只写的，所以 SPI 用 txonly（不接 MISO）；CS / DC / RST 三个脚由上位机当普通
//! GPIO 控制。SCK 决定走哪条 SPI 总线（GP2/6/18/22 是 SPI0，GP10/14/26 是 SPI1），
//! 且 MOSI 必须是该总线上合法的引脚组合（本 demo 默认 GP10 + GP11 走 SPI1）。
//!
//! 用法：st7789.bat COM13 [SCK脚] [MOSI脚] [DC脚] [RST脚] [CS脚|none] [旋转 0..3] [秒数]
//! 默认 GP10(SCK) / GP11(MOSI) / GP12(DC) / GP13(RES)，CS 不接、把模块的 CS 接到 GND。
//!
//! ST7789 控制器的显存是 240×320，而面板只有 240×240，所以写窗口时要按旋转方向补一个
//! 偏移（见 `Panel::xoff/yoff`）：偏移错了会整屏空白或错位。

use std::thread;
use std::time::{Duration, Instant};

use host::{Board, Error};

const W: usize = 240;
const H: usize = 240;
/// ST7789 最高可到 62.5 MHz；这里取 24 MHz，长杜邦线也稳。
const SPI_HZ: u32 = 24_000_000;
/// 单次 SPI 传输最多带多少数据字节：协议 body 上限 1024，扣掉 1 字节总线号。
const CHUNK: usize = 1000;
/// 弹球方块边长（像素）和每帧移动距离
const BOX: i32 = 32;
const SPEED: i32 = 3;

const BLACK: u16 = 0x0000;
const WHITE: u16 = 0xFFFF;

/// RGB888 → RGB565
const fn rgb(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

/// 一块已经初始化好的屏。xoff/yoff 是写窗口时的固定偏移（ST7789 显存多出的那 80 行）。
struct Panel {
    bus: u8,
    /// 数据/命令选择脚：低 = 命令，高 = 数据。
    dc: u8,
    xoff: u16,
    yoff: u16,
}

impl Panel {
    /// 发命令（含参数）：DC 拉低、发完再拉高。
    fn cmd(&self, board: &mut Board, bytes: &[u8]) -> Result<(), Error> {
        board.gpio_write(self.dc, false)?;
        board.spi_transfer(self.bus, bytes)?;
        board.gpio_write(self.dc, true)?;
        Ok(())
    }

    /// 发像素数据：DC 已经拉高，分块送完（每块 ≤ CHUNK 字节）。
    fn push(&self, board: &mut Board, data: &[u8]) -> Result<(), Error> {
        for part in data.chunks(CHUNK) {
            board.spi_transfer(self.bus, part)?;
        }
        Ok(())
    }

    /// 设置写窗口并进入写显存状态（CASET / RASET / RAMWR 一次发完）。
    fn set_window(&self, board: &mut Board, x: i32, y: i32, w: i32, h: i32) -> Result<(), Error> {
        let xs = x as u16 + self.xoff;
        let ys = y as u16 + self.yoff;
        let xe = xs + w as u16 - 1;
        let ye = ys + h as u16 - 1;
        let mut c = [0u8; 11];
        c[0] = 0x2A;
        c[1] = (xs >> 8) as u8;
        c[2] = xs as u8;
        c[3] = (xe >> 8) as u8;
        c[4] = xe as u8;
        c[5] = 0x2B;
        c[6] = (ys >> 8) as u8;
        c[7] = ys as u8;
        c[8] = (ye >> 8) as u8;
        c[9] = ye as u8;
        c[10] = 0x2C;
        self.cmd(board, &c)
    }

    /// 往一个矩形区域写数据（调用前先 set_window）。
    fn blit(&self, board: &mut Board, x: i32, y: i32, w: i32, h: i32, src: &[u8]) -> Result<(), Error> {
        self.set_window(board, x, y, w, h)?;
        self.push(board, src)
    }
}

/// 上电复位 + 初始化。时序取自 ST7789 常见 240×240 模块的通用初始化序列。
fn init(panel: &Panel, board: &mut Board, rst: u8) -> Result<(), Error> {
    // 硬复位：拉低 20 ms 再拉高
    board.pin_output(rst, true)?;
    thread::sleep(Duration::from_millis(10));
    board.pin_output(rst, false)?;
    thread::sleep(Duration::from_millis(20));
    board.pin_output(rst, true)?;
    thread::sleep(Duration::from_millis(120));

    let steps: &[(u8, &[u8])] = &[
        (0x01, &[]),                                        // SWRESET 软复位
        (0x11, &[]),                                        // SLPOUT 退出睡眠
        (0x3A, &[0x55]),                                    // COLMOD：16 位色
        (0xB2, &[0x0C, 0x0C, 0x00, 0x33, 0x33]),            // PORCTRL 前后沿
        (0xB7, &[0x35]),                                    // GCTRL 门极控制
        (0xBB, &[0x19]),                                    // VCOMS 公共电压
        (0xC0, &[0x2C]),                                    // LCMCTRL
        (0xC2, &[0x01]),                                    // VDVVRHEN
        (0xC3, &[0x12]),                                    // VRHS
        (0xC4, &[0x20]),                                    // VDVS
        (0xC6, &[0x0F]),                                    // FRCTRL2 帧率
        (0xD0, &[0xA4, 0xA1]),                              // PWCTRL1
        (0xE0, &[0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23]), // 正极性伽马
        (0xE1, &[0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23]), // 负极性伽马
        (0x21, &[]),                                        // INVON：IPS 屏要反色
    ];
    for (c, d) in steps {
        let mut buf = Vec::with_capacity(1 + d.len());
        buf.push(*c);
        buf.extend_from_slice(d);
        panel.cmd(board, &buf)?;
        // 这两条命令之后芯片需要时间，别的命令不用等。
        match *c {
            0x01 => thread::sleep(Duration::from_millis(150)),
            0x11 => thread::sleep(Duration::from_millis(120)),
            _ => {}
        }
    }
    panel.cmd(board, &[0x13])?; // NORON 正常显示模式
    thread::sleep(Duration::from_millis(10));
    panel.cmd(board, &[0x29])?; // DISPON 开显示
    thread::sleep(Duration::from_millis(100));
    Ok(())
}

/// 三角波：t 在 0..2*limit 之间来回，替代取模，避免方块绕一圈"瞬移"。
fn bounce(t: i32, limit: i32) -> i32 {
    if limit <= 0 {
        return 0;
    }
    let m = t.rem_euclid(2 * limit);
    if m < limit {
        m
    } else {
        2 * limit - m
    }
}

fn px(fb: &mut [u8], x: i32, y: i32, c: u16) {
    if x < 0 || y < 0 || x >= W as i32 || y >= H as i32 {
        return;
    }
    let i = (y as usize * W + x as usize) * 2;
    fb[i] = (c >> 8) as u8;
    fb[i + 1] = c as u8;
}

fn fill_rect(fb: &mut [u8], x0: i32, y0: i32, x1: i32, y1: i32, c: u16) {
    for y in y0..=y1 {
        for x in x0..=x1 {
            px(fb, x, y, c);
        }
    }
}

/// 静态背景：8 条横向色带 + 白色边框。擦除方块时从这里取像素。
fn background(fb: &mut [u8]) {
    const BARS: [u16; 8] = [
        rgb(255, 0, 0),
        rgb(255, 128, 0),
        rgb(255, 255, 0),
        rgb(0, 255, 0),
        rgb(0, 255, 255),
        rgb(0, 128, 255),
        rgb(128, 0, 255),
        rgb(255, 0, 255),
    ];
    for (i, c) in BARS.iter().enumerate() {
        let y = i as i32 * 30;
        fill_rect(fb, 0, y, W as i32 - 1, y + 29, *c);
    }
    let (w, h) = (W as i32 - 1, H as i32 - 1);
    fill_rect(fb, 0, 0, w, 1, WHITE);
    fill_rect(fb, 0, h - 1, w, h, WHITE);
    fill_rect(fb, 0, 0, 1, h, WHITE);
    fill_rect(fb, w - 1, 0, w, h, WHITE);
}

/// 从显存里裁一个矩形，拼成一行接一行的连续字节流。
fn crop(fb: &[u8], x: i32, y: i32, w: i32, h: i32) -> Vec<u8> {
    let mut out = Vec::with_capacity((w * h * 2) as usize);
    for row in 0..h {
        let start = ((y + row) as usize * W + x as usize) * 2;
        out.extend_from_slice(&fb[start..start + w as usize * 2]);
    }
    out
}

/// 纯色矩形。
fn solid(c: u16, w: i32, h: i32) -> Vec<u8> {
    let mut row = Vec::with_capacity(w as usize * 2);
    for _ in 0..w {
        row.extend_from_slice(&[(c >> 8) as u8, c as u8]);
    }
    let mut out = Vec::with_capacity((w * h * 2) as usize);
    for _ in 0..h {
        out.extend_from_slice(&row);
    }
    out
}

fn main() -> Result<(), Error> {
    let port = std::env::args().nth(1).unwrap_or_else(|| {
        if cfg!(windows) {
            "COM5".into()
        } else {
            "/dev/ttyACM0".into()
        }
    });
    let arg = |n: usize, d: u8| -> u8 { std::env::args().nth(n).and_then(|s| s.parse().ok()).unwrap_or(d) };
    let sck = arg(2, 10);
    let mosi = arg(3, 11);
    let dc = arg(4, 12);
    let rst = arg(5, 13);
    let cs: Option<u8> = match std::env::args().nth(6).as_deref() {
        None | Some("none") => None,
        Some(s) => s.parse().ok(),
    };
    let rot: u32 = std::env::args().nth(7).and_then(|s| s.parse().ok()).unwrap_or(0);
    let secs: u64 = std::env::args().nth(8).and_then(|s| s.parse().ok()).unwrap_or(15);

    println!("接线（ST7789 240×240）：");
    println!("  屏 SCL/SCK -> GP{sck}，SDA/MOSI -> GP{mosi}，DC -> GP{dc}，RES -> GP{rst}");
    match cs {
        Some(cs) => println!("  屏 CS -> GP{cs}（本 demo 一直拉低）"),
        None => println!("  屏无 CS 脚（或已接地），程序不控制 CS"),
    }
    println!("  屏 VCC -> 3V3，GND -> GND，BLK -> 3V3（背光）");
    println!("  ⚠ SCK/MOSI 必须是同一总线的合法组合：SPI0 = SCK(GP2|GP6) + MOSI(GP3|GP7)；");
    println!("    SPI1 = SCK(GP10|GP14|GP26) + MOSI(GP11|GP15|GP27)");

    let mut board = Board::open(&port)?;
    let v = board.version()?;
    println!("protocol {} firmware {} pins {}", v.protocol, v.firmware, v.pins);

    // SCK 推出总线号（与固件 spi_bus() 一致）
    let bus: u8 = match sck {
        2 | 6 | 18 | 22 => 0,
        _ => 1,
    };
    board.spi_config(sck, mosi, None, SPI_HZ, 0)?;
    if let Some(cs) = cs {
        board.pin_output(cs, false)?;
    }

    // 旋转决定 MADCTL 和写窗口偏移（240×240 面板只看显存里的 240 行，要补 80 行）
    let (madctl, xoff, yoff) = match rot {
        0 => (0x00u8, 0u16, 80u16),
        1 => (0x60, 80, 0),
        2 => (0xC0, 0, 80),
        _ => (0xA0, 80, 0),
    };
    let panel = Panel {
        bus,
        dc,
        xoff,
        yoff,
    };

    init(&panel, &mut board, rst)?;
    panel.cmd(&mut board, &[0x36, madctl])?;
    println!("初始化完成：SPI {bus} @ {} MHz，旋转 {rot}，窗口偏移 ({xoff},{yoff})", SPI_HZ / 1_000_000);

    // 全屏画一次测试图案，验证整屏写窗口 + 分块传输
    let mut fb = vec![0u8; W * H * 2];
    background(&mut fb);
    let t0 = Instant::now();
    panel.blit(&mut board, 0, 0, W as i32, H as i32, &fb)?;
    let ms = t0.elapsed().as_millis();
    println!(
        "整屏 {W}×{H} 刷新完成：{} 字节 / {} 次往返 / {ms} ms",
        fb.len(),
        fb.len().div_ceil(CHUNK)
    );

    // 弹球动画：只更新新旧两个方块区域，避免每帧整屏重传
    println!("弹球动画 {secs} 秒……");
    let mut prev = (0i32, 0i32);
    let mut frame = 0u64;
    let bg_box = (BOX, BOX);
    let solid_box = solid(BLACK, BOX, BOX);
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(secs) {
        let bx = bounce(frame as i32 * SPEED, W as i32 - BOX - 2);
        let by = bounce(frame as i32 * SPEED * 2 / 3, H as i32 - BOX - 2);
        // 先擦掉旧位置（从背景显存取像素），再画新位置
        let old = crop(&fb, prev.0, prev.1, bg_box.0, bg_box.1);
        panel.blit(&mut board, prev.0, prev.1, bg_box.0, bg_box.1, &old)?;
        panel.blit(&mut board, bx, by, BOX, BOX, &solid_box)?;
        prev = (bx, by);
        frame += 1;
    }
    let secs_f = t0.elapsed().as_secs_f64();
    println!(
        "动画结束：{frame} 帧 / {secs_f:.1} s = {:.1} fps（每帧 2 个区域、约 12 次往返）",
        frame as f64 / secs_f
    );
    println!("关机完成（CS 拉低不影响显示，屏会保持最后一帧）");
    Ok(())
}
