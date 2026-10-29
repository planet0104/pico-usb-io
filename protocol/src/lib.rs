//! 上位机和固件共用的串口帧协议。
//!
//! 一帧的格式（多字节字段都是小端）：
//!
//! ```text
//! A5 5A | kind(1) seq(2) op(2) status(1) len(2) | body(len) | crc16(2)
//! ```
//!
//! `kind` 区分请求 / 应答 / 事件；`seq` 用来把应答和请求对上；`op` 是操作码；
//! `status` 只在应答里有意义；CRC16 覆盖从 kind 到 body 结束的所有字节。
//! 这个 crate 不依赖标准库，固件（no_std）和上位机都直接引用它，保证两边常量一致。

#![cfg_attr(not(test), no_std)]

/// 帧起始标志的两个字节。
pub const SOF0: u8 = 0xA5;
pub const SOF1: u8 = 0x5A;
/// 协议版本。删除旧的 I2S / pulse / bitstream 专用命令后升为 2；
/// 加 OP_SPI_WRITE、body 上限从 1 KB 提到 4 KB 后升为 3；加 OP_SPI_WRITE_LZ4、OP_JPEG 后升为 4；
/// 加 JPEG_CORE1 后升为 5。
pub const PROTO_VERSION: u16 = 5;
/// RP2040 可用的 GPIO 数量（GP0..GP29）。
pub const PIN_COUNT: u8 = 30;
/// 单帧 body 的最大字节数。刷屏这种大块数据每帧要等一次应答，帧越大往返越少。
pub const MAX_BODY: usize = 4096;
/// kind + seq + op + status + len 共 8 字节。
pub const HEADER_LEN: usize = 8;
/// 一帧的最大总长度：SOF(2) + 头(8) + body + CRC(2)。
pub const MAX_FRAME: usize = 2 + HEADER_LEN + MAX_BODY + 2;

/// 帧类型：上位机发出的请求。
pub const KIND_REQ: u8 = 1;
/// 帧类型：固件对请求的应答，seq 与请求相同。
pub const KIND_RESP: u8 = 2;
/// 帧类型：固件主动上报的事件（例如 PIO 流数据），seq 固定为 0。
pub const KIND_EVENT: u8 = 3;

// 基础通讯操作码。
pub const OP_VERSION: u16 = 1;
pub const OP_PIN_CONFIG: u16 = 2;
pub const OP_GPIO_WRITE: u16 = 3;
pub const OP_GPIO_READ: u16 = 4;
pub const OP_PWM_SET: u16 = 5;
pub const OP_ADC_READ: u16 = 6;
pub const OP_I2C_CONFIG: u16 = 7;
pub const OP_I2C_TRANSFER: u16 = 8;
pub const OP_SPI_CONFIG: u16 = 9;
pub const OP_SPI_TRANSFER: u16 = 10;
pub const OP_UART_CONFIG: u16 = 11;
pub const OP_UART_WRITE: u16 = 12;
pub const OP_UART_READ: u16 = 13;
/// 只发不收的 SPI 写：body = [bus, 数据...]，应答没有 body。
/// 刷屏用它代替 OP_SPI_TRANSFER，USB 上不用再把读回的同样多字节传回来。
pub const OP_SPI_WRITE: u16 = 14;
/// 同 OP_SPI_WRITE，但数据是一个 LZ4 块（lz4_flex 的 block 格式，不带长度前缀）：
/// body = [bus, 压缩数据...]，固件解压后再发，解压后最多 SPI_LZ4_MAX 字节。
pub const OP_SPI_WRITE_LZ4: u16 = 15;
/// 一个 LZ4 块解压后的最大字节数。
pub const SPI_LZ4_MAX: usize = 16 * 1024;
/// 把一张基线 JPEG 解码后画到 ST7789 这类"设窗口 + 写显存"的屏上。一张图分几帧发：
/// body = [flags, 数据...]；第一帧（JPEG_FIRST）的数据前面多 6 字节 [bus, DC 脚, x u16, y u16]，
/// 是总线、屏的数据/命令选择脚和图片左上角在屏上的位置；最后一帧（JPEG_LAST）收齐后解码显示，
/// 应答 [解码显示用时 u32 微秒]。DC 脚要先用 OP_GPIO_WRITE 设成输出。
/// 第一帧带 JPEG_CORE1 时交给第二个核解码：收齐就应答，不等画完，应答里是第二个核上一张的用时。
/// 这样把一屏切成两张图、一张带 JPEG_CORE1 一张不带，两个核同时解码。
pub const OP_JPEG: u16 = 16;
pub const JPEG_FIRST: u8 = 1 << 0;
pub const JPEG_LAST: u8 = 1 << 1;
pub const JPEG_CORE1: u8 = 1 << 2;
/// 一张 JPEG 最大字节数（240×240 质量 95 约 21 KB）。两个核各要一块接收缓冲区，内存不够再大了。
pub const JPEG_MAX: usize = 24 * 1024;

// 通用 PIO 原语操作码，body 布局见 pio-refactor.md 第 4 节。
/// 把 PIO 指令字装进指令内存。
pub const OP_PIO_LOAD: u16 = 20;
/// 释放一段指令内存。
pub const OP_PIO_UNLOAD: u16 = 21;
/// 配置一个状态机（程序、分频、引脚、移位、FIFO）。
pub const OP_PIO_SM_CONFIG: u16 = 22;
/// 使能 / 停用状态机。
pub const OP_PIO_SM_ENABLE: u16 = 23;
/// 复位状态机内部状态、分频相位，可选清 FIFO。
pub const OP_PIO_SM_RESTART: u16 = 24;
/// 往状态机的 TX FIFO 写数据。
pub const OP_PIO_TX_PUSH: u16 = 25;
/// 从状态机的 RX FIFO 读数据。
pub const OP_PIO_RX_POP: u16 = 26;
/// 启动流：固件持续把 RX FIFO 的数据作为事件帧发给上位机。
pub const OP_PIO_STREAM_START: u16 = 27;
/// 停止流。
pub const OP_PIO_STREAM_STOP: u16 = 28;
/// 查询流状态和丢弃计数。
pub const OP_PIO_STREAM_STATUS: u16 = 29;
/// 只用作事件帧的 op，body 是流数据（u32 小端序列）。
pub const OP_PIO_STREAM_EVENT: u16 = 30;

/// OP_PIO_LOAD 的 flags：由固件自动选择装载地址。
pub const PIO_LOAD_AUTO: u8 = 1 << 0;

/// OP_PIO_SM_CONFIG 的 body 固定长度。
pub const PIO_SM_CONFIG_LEN: usize = 24;
// OP_PIO_SM_CONFIG 第 14 字节 flags 的各个位。
/// side-set 是可选的（指令里有一位决定本条是否 side-set）。
pub const PIO_SIDESET_OPT: u8 = 1 << 0;
/// side-set 改的是引脚方向而不是电平。
pub const PIO_SIDESET_PINDIRS: u8 = 1 << 1;
/// OUT 移位方向：1 = 右移（先移出低位）。
pub const PIO_OUT_SHIFT_RIGHT: u8 = 1 << 2;
/// IN 移位方向：1 = 右移。
pub const PIO_IN_SHIFT_RIGHT: u8 = 1 << 3;
/// OSR 空到阈值时自动从 TX FIFO 取数。
pub const PIO_AUTOPULL: u8 = 1 << 4;
/// ISR 满到阈值时自动推入 RX FIFO。
pub const PIO_AUTOPUSH: u8 = 1 << 5;
// FIFO 合并方式：把 TX 和 RX 的 4 级 FIFO 合成单向 8 级。
pub const PIO_JOIN_NONE: u8 = 0;
pub const PIO_JOIN_TX: u8 = 1;
pub const PIO_JOIN_RX: u8 = 2;
/// 表示"不使用这个引脚"，用于 jmp_pin。
pub const PIO_NO_PIN: u8 = 0xFF;

// OP_PIO_SM_RESTART 的 flags。
/// 让分频器从相位 0 重新开始。
pub const PIO_RESTART_CLKDIV: u8 = 1 << 0;
/// 清空 TX / RX FIFO。
pub const PIO_RESTART_FIFOS: u8 = 1 << 1;
/// 复位状态机内部状态（移位计数、延时、阻塞中的指令），不改 PC。
pub const PIO_RESTART_SM: u8 = 1 << 2;

/// OP_PIO_TX_PUSH 的 flags：FIFO 满时等待（有上限），否则立即返回。
pub const PIO_TX_WAIT: u8 = 1 << 0;

/// OP_PIO_STREAM_START 的 flags：稀疏模式，不用 DMA，有数据才发。
pub const PIO_STREAM_SPARSE: u8 = 1 << 0;

// 应答里的 status。
pub const ST_OK: u8 = 0;
pub const ST_BAD_FRAME: u8 = 1;
pub const ST_BAD_PIN: u8 = 2;
pub const ST_BAD_ARG: u8 = 3;
pub const ST_BUSY: u8 = 4;
pub const ST_IO: u8 = 5;
pub const ST_NO_SPACE: u8 = 6;

// OP_PIN_CONFIG 的引脚模式。
pub const PIN_HIZ: u8 = 0;
pub const PIN_INPUT: u8 = 1;
pub const PIN_OUTPUT: u8 = 2;

// OP_PIN_CONFIG 的上下拉。
pub const PULL_NONE: u8 = 0;
pub const PULL_UP: u8 = 1;
pub const PULL_DOWN: u8 = 2;

/// OP_SPI_CONFIG 里表示"不接 MISO"（只发不收）。
pub const SPI_MISO_NONE: u8 = 0xFF;

/// 解析后的一帧，body 借用自解析缓冲区。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    pub kind: u8,
    pub seq: u16,
    pub op: u16,
    pub status: u8,
    pub body: &'a [u8],
}

/// 把一帧编码进 `out`，返回总字节数；body 过长或 `out` 不够大时返回 None。
pub fn encode(frame: Frame<'_>, out: &mut [u8]) -> Option<usize> {
    let n = 2 + HEADER_LEN + frame.body.len() + 2;
    if frame.body.len() > MAX_BODY || out.len() < n {
        return None;
    }
    out[0] = SOF0;
    out[1] = SOF1;
    out[2] = frame.kind;
    out[3..5].copy_from_slice(&frame.seq.to_le_bytes());
    out[5..7].copy_from_slice(&frame.op.to_le_bytes());
    out[7] = frame.status;
    out[8..10].copy_from_slice(&(frame.body.len() as u16).to_le_bytes());
    out[10..10 + frame.body.len()].copy_from_slice(frame.body);
    let crc = crc16(&out[2..10 + frame.body.len()]);
    let at = 10 + frame.body.len();
    out[at..at + 2].copy_from_slice(&crc.to_le_bytes());
    Some(n)
}

/// 流式解析器：串口数据可以任意切块喂进来，凑齐一整帧且 CRC 正确就回调一次。
/// 遇到垃圾数据或 CRC 错误时丢掉一个字节，重新寻找下一个 A5 5A，所以能自动重新同步。
pub struct Parser {
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_FRAME],
            len: 0,
        }
    }

    /// 缓冲区里没有收了一半的帧。
    pub fn is_idle(&self) -> bool {
        self.len == 0
    }

    /// 丢掉收了一半的帧。
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// 喂入一段收到的字节，每解出一帧调用一次 `on_frame`。整段一次拷进缓冲区再找帧，
    /// 固件收 USB 时不用每个字节都检查一遍。
    pub fn push(&mut self, mut data: &[u8], mut on_frame: impl for<'a> FnMut(Frame<'a>)) {
        while !data.is_empty() {
            if self.len >= MAX_FRAME {
                self.shift_sof();
            }
            let n = data.len().min(MAX_FRAME - self.len);
            self.buf[self.len..self.len + n].copy_from_slice(&data[..n]);
            self.len += n;
            data = &data[n..];
            self.align_sof();
            while self.try_one(&mut on_frame) {}
        }
    }

    /// 让缓冲区从 A5 5A 开头；找不到时只保留可能是半个 SOF 的最后一个 A5。
    fn align_sof(&mut self) {
        if self.len >= 2 && self.buf[0] == SOF0 && self.buf[1] == SOF1 {
            return;
        }
        if let Some(i) = self.buf[..self.len].windows(2).position(|w| w == [SOF0, SOF1]) {
            self.buf.copy_within(i..self.len, 0);
            self.len -= i;
        } else if self.len > 0 && self.buf[self.len - 1] == SOF0 {
            self.buf[0] = SOF0;
            self.len = 1;
        } else {
            self.len = 0;
        }
    }

    /// 丢掉开头一个字节，再重新对齐到下一个 SOF。
    fn shift_sof(&mut self) {
        if self.len > 1 {
            self.buf.copy_within(1..self.len, 0);
            self.len -= 1;
        } else {
            self.len = 0;
        }
        self.align_sof();
    }

    /// 尝试从缓冲区开头解出一帧；返回 true 表示缓冲区有变化，应再试一次。
    fn try_one(&mut self, on_frame: &mut impl for<'a> FnMut(Frame<'a>)) -> bool {
        if self.len < 2 + HEADER_LEN + 2 {
            return false;
        }
        if self.buf[0] != SOF0 || self.buf[1] != SOF1 {
            self.shift_sof();
            return self.len > 0;
        }
        let body_len = u16::from_le_bytes([self.buf[8], self.buf[9]]) as usize;
        if body_len > MAX_BODY {
            self.shift_sof();
            return self.len > 0;
        }
        let total = 2 + HEADER_LEN + body_len + 2;
        if self.len < total {
            return false;
        }
        let expect = u16::from_le_bytes([self.buf[total - 2], self.buf[total - 1]]);
        let actual = crc16(&self.buf[2..total - 2]);
        if expect == actual {
            // 先回调（body 直接借用缓冲区），再把剩余数据前移。
            on_frame(Frame {
                kind: self.buf[2],
                seq: u16::from_le_bytes([self.buf[3], self.buf[4]]),
                op: u16::from_le_bytes([self.buf[5], self.buf[6]]),
                status: self.buf[7],
                body: &self.buf[10..10 + body_len],
            });
            self.buf.copy_within(total..self.len, 0);
            self.len -= total;
            true
        } else {
            self.shift_sof();
            self.len > 0
        }
    }
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC-16/MODBUS（多项式 0xA001 反射，初值 0xFFFF）。按字节查表：固件的 Cortex-M0+ 上
/// 逐位算一个 4 KB 的帧要 1 ms 以上，刷屏时每帧都要算。
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc = (crc >> 8) ^ CRC_TABLE[((crc ^ b as u16) & 0xFF) as usize];
    }
    crc
}

const CRC_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u16;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

/// 从 body 的 `at` 位置读小端 u16，越界返回 None。
pub fn read_u16(body: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*body.get(at)?, *body.get(at + 1)?]))
}

/// 从 body 的 `at` 位置读小端 u32，越界返回 None。
pub fn read_u32(body: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes([
        *body.get(at)?,
        *body.get(at + 1)?,
        *body.get(at + 2)?,
        *body.get(at + 3)?,
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 一帧前面混入垃圾字节和半个 SOF，解析器仍能正确解出这一帧。
    #[test]
    fn roundtrip_and_resync() {
        let body = [1u8, 2, 3, 4, 5];
        let mut raw = [0u8; 64];
        let n = encode(
            Frame {
                kind: KIND_REQ,
                seq: 0x1234,
                op: OP_GPIO_WRITE,
                status: 0,
                body: &body,
            },
            &mut raw,
        )
        .unwrap();
        let stream = [0x00, 0xA5, 0x11];
        let mut all = [0u8; 80];
        all[..3].copy_from_slice(&stream);
        all[3..3 + n].copy_from_slice(&raw[..n]);
        let mut p = Parser::new();
        let mut got = false;
        p.push(&all[..3 + n], |f| {
            assert_eq!(f.seq, 0x1234);
            assert_eq!(f.op, OP_GPIO_WRITE);
            assert_eq!(f.body, &body);
            got = true;
        });
        assert!(got);
    }

    #[test]
    fn crc16_modbus_check_value() {
        assert_eq!(crc16(b"123456789"), 0x4B37);
    }

    /// 两帧挤在同一块数据里、又被切成大小不一的块喂进来，都能解出来。
    #[test]
    fn frames_split_across_chunks() {
        let mut raw = [0u8; 2 * MAX_FRAME];
        let big = [7u8; MAX_BODY];
        let mut n = 0;
        for (seq, body) in [(1u16, &big[..]), (2, &[9u8, 8][..])] {
            let frame = Frame { kind: KIND_REQ, seq, op: OP_SPI_WRITE, status: 0, body };
            n += encode(frame, &mut raw[n..]).unwrap();
        }
        let mut p = Parser::new();
        let mut seqs = [0u16; 2];
        let mut count = 0;
        for chunk in raw[..n].chunks(37) {
            p.push(chunk, |f| {
                seqs[count] = f.seq;
                assert_eq!(f.body.len(), if f.seq == 1 { MAX_BODY } else { 2 });
                count += 1;
            });
        }
        assert_eq!((count, seqs), (2, [1, 2]));
    }
}
