use std::thread;
use std::time::Duration;

use host::Board;

fn main() {
    println!("连接:");
    println!("  USB 接到电脑。Windows 默认串口 COM5，也可以把端口当第一个参数");
    println!("  GP25 是 Pico 板载 LED，不用外接");
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

    board.pin_output(25, false).expect("led output");
    for _ in 0..3 {
        board.gpio_write(25, true).unwrap();
        thread::sleep(Duration::from_millis(150));
        board.gpio_write(25, false).unwrap();
        thread::sleep(Duration::from_millis(150));
    }
    println!("GP25 blinked");

    if std::env::var("DEMO").is_ok() {
        extra(&mut board);
    }
}

fn extra(board: &mut Board) {
    board.pwm_set(15, 1_000, 250).unwrap();
    println!("adc26 {}", board.adc_read(26).unwrap());
    board.i2c_config(4, 5, 100_000).unwrap();
    let _ = board.i2c_transfer(0, 0x3C, &[0x00], 0);
    board.spi_config(18, 19, Some(16), 1_000_000, 0).unwrap();
    board.gpio_write(17, false).unwrap();
    let _ = board.spi_transfer(0, &[0x9F]);
    board.uart_config(8, 9, 115_200).unwrap();
    board.uart_write(1, b"hi").unwrap();
    let _ = board.gpio_read(7);
}
