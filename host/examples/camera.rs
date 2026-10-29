//! 电脑摄像头实时预览到 ST7789 屏（经 pico-usb-io），测显示链路能跑到多快。
//!
//! 用法：cargo run -p host --example camera --release -- COM5 [摄像头编号] [秒数] [raw|lz4|jpeg|jpeg2] [JPEG 质量]
//! 默认：摄像头 0、30 秒、raw、质量 85。接线同 st7789.rs：SCK GP10，MOSI GP11，DC GP12，RES GP13，屏无 CS。
//!
//! - raw：直接发 240×240 的 RGB565（115200 字节）
//! - lz4：RGB565 用 LZ4 压缩后发，固件解压
//! - jpeg：整帧发一张 JPEG，固件解码后写屏
//! - jpeg2：上下切成两张 JPEG，上半张交给 RP2040 的第二个核，两个核同时解码
//!
//! 采集编码和刷屏分两个线程：后台线程不停把最新一帧准备好，主线程只管往屏上写，
//! 所以打印出来的帧率就是显示链路本身的上限。每 30 帧打印一次各段平均耗时。

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use host::{Board, Error};
use image::codecs::jpeg::JpegEncoder;
use image::imageops::{self, FilterType};
use image::{DynamicImage, ImageFormat, RgbImage};
use nokhwa::pixel_format::RgbFormat;
use nokhwa::utils::{
    CameraFormat, CameraIndex, FrameFormat, RequestedFormat, RequestedFormatType, Resolution,
};
use nokhwa::{Buffer, Camera};

const SIDE: u32 = 240;
const BUS: u8 = 1;
const SCK: u8 = 10;
const MOSI: u8 = 11;
const DC: u8 = 12;
const RST: u8 = 13;
const SPI_HZ: u32 = 24_000_000;
const SPI_MODE: u8 = 3;
const STATS: u32 = 30;
/// jpeg2 模式的切分行：16 的倍数，正好是 4:2:0 MCU 的边界，两张图都不用补行。
const SPLIT: u32 = 128;

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Raw,
    Lz4,
    Jpeg(u8),
    Jpeg2(u8),
}

/// 准备好的一帧（RGB565 像素，或一两张 JPEG）和准备它各段用的时间。
struct Prepared {
    parts: Vec<Vec<u8>>,
    grab: Duration,
    decode: Duration,
    convert: Duration,
}

type Latest = Arc<Mutex<Option<Prepared>>>;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let arg = |n: usize| std::env::args().nth(n);
    let port = arg(1).unwrap_or_else(|| String::from("COM5"));
    let index: u32 = arg(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let secs: u64 = arg(3).and_then(|s| s.parse().ok()).unwrap_or(30);
    let mode = match arg(4).as_deref() {
        Some("lz4") => Mode::Lz4,
        Some("jpeg") => Mode::Jpeg(arg(5).and_then(|s| s.parse().ok()).unwrap_or(85)),
        Some("jpeg2") => Mode::Jpeg2(arg(5).and_then(|s| s.parse().ok()).unwrap_or(85)),
        _ => Mode::Raw,
    };

    let mut board = Board::open(&port)?;
    let version = board.version()?;
    println!("protocol {} firmware {}", version.protocol, version.firmware);
    init_screen(&mut board)?;

    let latest: Latest = Arc::default();
    let producer = latest.clone();
    thread::spawn(move || {
        if let Err(err) = capture(index, &producer, mode) {
            eprintln!("摄像头出错：{err}");
            std::process::exit(1);
        }
    });

    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut window_start = Instant::now();
    let mut n = 0u32;
    let mut bytes = 0usize;
    // 取帧、解码、缩放和编码、发送（含设窗口）、固件第一个核解码显示、第二个核解码显示
    let mut sums = [Duration::ZERO; 6];
    while Instant::now() < deadline {
        let Some(frame) = latest.lock().unwrap().take() else {
            thread::sleep(Duration::from_millis(1));
            continue;
        };
        let started = Instant::now();
        let (mut core0, mut core1) = (Duration::ZERO, Duration::ZERO);
        match mode {
            Mode::Raw => {
                set_window(&mut board)?;
                board.spi_write(BUS, &frame.parts[0])?;
            }
            Mode::Lz4 => {
                set_window(&mut board)?;
                board.spi_write_lz4(BUS, &frame.parts[0])?;
            }
            Mode::Jpeg(_) => core0 = board.jpeg_show(BUS, DC, 0, 0, &frame.parts[0], false)?,
            Mode::Jpeg2(_) => {
                core1 = board.jpeg_show(BUS, DC, 0, 0, &frame.parts[0], true)?;
                core0 = board.jpeg_show(BUS, DC, 0, SPLIT as u16, &frame.parts[1], false)?;
            }
        }
        let sent = started.elapsed();
        for (sum, add) in sums
            .iter_mut()
            .zip([frame.grab, frame.decode, frame.convert, sent, core0, core1])
        {
            *sum += add;
        }
        bytes += frame.parts.iter().map(Vec::len).sum::<usize>();
        n += 1;
        if n == STATS {
            let [grab, decode, convert, sent, core0, core1] = sums.map(|s| (s / n).as_secs_f64() * 1000.0);
            let mut line = format!(
                "显示 {:.1} fps，每帧 {} 字节 | 后台：取帧 {grab:.1} ms，解码 {decode:.1} ms，缩放编码 {convert:.1} ms | 发送 {sent:.1} ms",
                f64::from(n) / window_start.elapsed().as_secs_f64(),
                bytes / n as usize,
            );
            match mode {
                Mode::Jpeg(_) => line.push_str(&format!("（其中固件解码显示 {core0:.1} ms）")),
                Mode::Jpeg2(_) => {
                    line.push_str(&format!("（固件解码显示：上半 {core1:.1} ms，下半 {core0:.1} ms）"))
                }
                _ => {}
            }
            println!("{line}");
            window_start = Instant::now();
            n = 0;
            bytes = 0;
            sums = [Duration::ZERO; 6];
        }
    }
    Ok(())
}

/// 后台线程：取帧、解码、缩到 240×240，再转成 RGB565 或编码成 JPEG，只留最新的一帧。
fn capture(index: u32, latest: &Latest, mode: Mode) -> Result<(), String> {
    let wanted = RequestedFormat::new::<RgbFormat>(RequestedFormatType::Closest(CameraFormat::new(
        Resolution::new(640, 480),
        FrameFormat::MJPEG,
        30,
    )));
    // Windows 上按指定格式打开有时报 Failed to fulfill requested format，退回让驱动挑帧率最高的格式
    let mut camera = Camera::new(CameraIndex::Index(index), wanted)
        .or_else(|_| {
            let fastest = RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
            Camera::new(CameraIndex::Index(index), fastest)
        })
        .map_err(|e| e.to_string())?;
    camera.open_stream().map_err(|e| e.to_string())?;
    println!("摄像头 {}，格式 {}", camera.info().human_name(), camera.camera_format());
    loop {
        let started = Instant::now();
        let frame = camera.frame().map_err(|e| e.to_string())?;
        let grab = started.elapsed();
        let image = decode(&frame)?;
        let decode = started.elapsed() - grab;
        let small = fit(&image);
        let parts = match mode {
            Mode::Raw | Mode::Lz4 => vec![rgb565(&small)],
            Mode::Jpeg(quality) => vec![jpeg(&small, quality)?],
            Mode::Jpeg2(quality) => {
                let top = imageops::crop_imm(&small, 0, 0, SIDE, SPLIT).to_image();
                let bottom = imageops::crop_imm(&small, 0, SPLIT, SIDE, SIDE - SPLIT).to_image();
                vec![jpeg(&top, quality)?, jpeg(&bottom, quality)?]
            }
        };
        let convert = started.elapsed() - grab - decode;
        *latest.lock().unwrap() = Some(Prepared {
            parts,
            grab,
            decode,
            convert,
        });
    }
}

fn jpeg(image: &RgbImage, quality: u8) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    JpegEncoder::new_with_quality(&mut out, quality)
        .encode_image(image)
        .map_err(|e| e.to_string())?;
    Ok(out)
}

fn decode(frame: &Buffer) -> Result<RgbImage, String> {
    if frame.source_frame_format() == FrameFormat::MJPEG {
        return image::load_from_memory_with_format(frame.buffer(), ImageFormat::Jpeg)
            .map(DynamicImage::into_rgb8)
            .map_err(|e| e.to_string());
    }
    let decoded = frame.decode_image::<RgbFormat>().map_err(|e| e.to_string())?;
    let (width, height) = (decoded.width(), decoded.height());
    RgbImage::from_raw(width, height, decoded.into_raw()).ok_or_else(|| String::from("解码失败"))
}

/// 居中裁成正方形再缩到屏幕大小（和 ai-box 的预览一样）。
fn fit(image: &RgbImage) -> RgbImage {
    let side = image.width().min(image.height());
    let x = (image.width() - side) / 2;
    let y = (image.height() - side) / 2;
    let square = imageops::crop_imm(image, x, y, side, side).to_image();
    imageops::resize(&square, SIDE, SIDE, FilterType::Triangle)
}

fn rgb565(image: &RgbImage) -> Vec<u8> {
    let mut out = Vec::with_capacity((SIDE * SIDE * 2) as usize);
    for p in image.pixels() {
        let [r, g, b] = p.0;
        let c = (u16::from(r >> 3) << 11) | (u16::from(g >> 2) << 5) | u16::from(b >> 3);
        out.extend_from_slice(&c.to_be_bytes());
    }
    out
}

/// DC 拉低发命令字节，拉高发参数；结束时 DC 保持高，后面直接发像素。
fn cmd(board: &mut Board, c: u8, params: &[u8]) -> Result<(), Error> {
    board.gpio_write(DC, false)?;
    board.spi_write(BUS, &[c])?;
    board.gpio_write(DC, true)?;
    if !params.is_empty() {
        board.spi_write(BUS, params)?;
    }
    Ok(())
}

fn set_window(board: &mut Board) -> Result<(), Error> {
    let end = (SIDE - 1) as u16;
    let range = [0, 0, (end >> 8) as u8, end as u8];
    cmd(board, 0x2A, &range)?;
    cmd(board, 0x2B, &range)?;
    cmd(board, 0x2C, &[])
}

fn init_screen(board: &mut Board) -> Result<(), Error> {
    board.spi_config(SCK, MOSI, None, SPI_HZ, SPI_MODE)?;
    board.pin_output(DC, true)?;
    board.pin_output(RST, true)?;
    thread::sleep(Duration::from_millis(10));
    board.pin_output(RST, false)?;
    thread::sleep(Duration::from_millis(20));
    board.pin_output(RST, true)?;
    thread::sleep(Duration::from_millis(120));
    cmd(board, 0x01, &[])?; // SWRESET
    thread::sleep(Duration::from_millis(150));
    cmd(board, 0x11, &[])?; // SLPOUT
    thread::sleep(Duration::from_millis(120));
    cmd(board, 0x21, &[])?; // INVON：这块屏要开颜色反转，不开是底片色
    cmd(board, 0x36, &[0x00])?; // MADCTL 旋转 0
    cmd(board, 0x3A, &[0x55])?; // COLMOD 16 位色 RGB565
    cmd(board, 0x13, &[])?; // NORON
    thread::sleep(Duration::from_millis(10));
    cmd(board, 0x29, &[])?; // DISPON
    thread::sleep(Duration::from_millis(10));
    Ok(())
}
