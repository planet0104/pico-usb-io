use std::thread;
use std::time::Duration;

use host::{BitTiming, Board};

fn main() {
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  板载 WS2812 接 GP16，第二个参数可以改数据脚");
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
        .unwrap_or(16u8);
    let mut board = Board::open(&port).expect("open serial");
    let version = board.version().expect("version");
    println!(
        "protocol {} firmware {} pins {}",
        version.protocol, version.firmware, version.pins
    );

    let timing = BitTiming::ws2812();
    let colors: &[(&str, [u8; 3])] = &[
        ("red", [0x20, 0x00, 0x00]),
        ("green", [0x00, 0x20, 0x00]),
        ("blue", [0x00, 0x00, 0x20]),
        ("off", [0x00, 0x00, 0x00]),
    ];
    for (name, rgb) in colors {
        board.bitstream(pin, timing, rgb).expect("ws2812");
        println!("GP{pin} {name}");
        thread::sleep(Duration::from_millis(400));
    }
}
