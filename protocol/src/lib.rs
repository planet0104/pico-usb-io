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
/// 协议版本。删除旧的 I2S / pulse / bitstream 专用命令后升为 2。
pub const PROTO_VERSION: u16 = 2;
/// RP2040 可用的 GPIO 数量（GP0..GP29）。
pub const PIN_COUNT: u8 = 30;
/// 单帧 body 的最大字节数。
pub const MAX_BODY: usize = 1024;
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

    /// 喂入一段收到的字节，每解出一帧调用一次 `on_frame`。
    pub fn push(&mut self, data: &[u8], mut on_frame: impl for<'a> FnMut(Frame<'a>)) {
        for &b in data {
            if self.len >= MAX_FRAME {
                self.shift_sof();
            }
            self.buf[self.len] = b;
            self.len += 1;
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
            let kind = self.buf[2];
            let seq = u16::from_le_bytes([self.buf[3], self.buf[4]]);
            let op = u16::from_le_bytes([self.buf[5], self.buf[6]]);
            let status = self.buf[7];
            // 先把 body 复制出来，再把剩余数据前移，回调里拿到的 body 才不会被覆盖。
            let mut body = [0u8; MAX_BODY];
            body[..body_len].copy_from_slice(&self.buf[10..10 + body_len]);
            self.buf.copy_within(total..self.len, 0);
            self.len -= total;
            on_frame(Frame {
                kind,
                seq,
                op,
                status,
                body: &body[..body_len],
            });
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

/// CRC-16/MODBUS（多项式 0xA001 反射，初值 0xFFFF）。
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

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
}
