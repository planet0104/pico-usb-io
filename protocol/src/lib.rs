#![cfg_attr(not(test), no_std)]

pub const SOF0: u8 = 0xA5;
pub const SOF1: u8 = 0x5A;
pub const PROTO_VERSION: u16 = 2;
pub const PIN_COUNT: u8 = 30;
pub const MAX_BODY: usize = 1024;
pub const HEADER_LEN: usize = 8;
pub const MAX_FRAME: usize = 2 + HEADER_LEN + MAX_BODY + 2;

pub const KIND_REQ: u8 = 1;
pub const KIND_RESP: u8 = 2;
pub const KIND_EVENT: u8 = 3;

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
pub const OP_PIO_LOAD: u16 = 20;
pub const OP_PIO_UNLOAD: u16 = 21;
pub const OP_PIO_SM_CONFIG: u16 = 22;
pub const OP_PIO_SM_ENABLE: u16 = 23;
pub const OP_PIO_SM_RESTART: u16 = 24;
pub const OP_PIO_TX_PUSH: u16 = 25;
pub const OP_PIO_RX_POP: u16 = 26;
pub const OP_PIO_STREAM_START: u16 = 27;
pub const OP_PIO_STREAM_STOP: u16 = 28;
pub const OP_PIO_STREAM_STATUS: u16 = 29;
pub const OP_PIO_STREAM_EVENT: u16 = 30;

pub const PIO_LOAD_AUTO: u8 = 1 << 0;

pub const PIO_SM_CONFIG_LEN: usize = 24;
pub const PIO_SIDESET_OPT: u8 = 1 << 0;
pub const PIO_SIDESET_PINDIRS: u8 = 1 << 1;
pub const PIO_OUT_SHIFT_RIGHT: u8 = 1 << 2;
pub const PIO_IN_SHIFT_RIGHT: u8 = 1 << 3;
pub const PIO_AUTOPULL: u8 = 1 << 4;
pub const PIO_AUTOPUSH: u8 = 1 << 5;
pub const PIO_JOIN_NONE: u8 = 0;
pub const PIO_JOIN_TX: u8 = 1;
pub const PIO_JOIN_RX: u8 = 2;
pub const PIO_NO_PIN: u8 = 0xFF;

pub const PIO_RESTART_CLKDIV: u8 = 1 << 0;
pub const PIO_RESTART_FIFOS: u8 = 1 << 1;
pub const PIO_RESTART_SM: u8 = 1 << 2;

pub const PIO_TX_WAIT: u8 = 1 << 0;

pub const PIO_STREAM_SPARSE: u8 = 1 << 0;

pub const ST_OK: u8 = 0;
pub const ST_BAD_FRAME: u8 = 1;
pub const ST_BAD_PIN: u8 = 2;
pub const ST_BAD_ARG: u8 = 3;
pub const ST_BUSY: u8 = 4;
pub const ST_IO: u8 = 5;
pub const ST_NO_SPACE: u8 = 6;

pub const PIN_HIZ: u8 = 0;
pub const PIN_INPUT: u8 = 1;
pub const PIN_OUTPUT: u8 = 2;

pub const PULL_NONE: u8 = 0;
pub const PULL_UP: u8 = 1;
pub const PULL_DOWN: u8 = 2;

pub const SPI_MISO_NONE: u8 = 0xFF;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    pub kind: u8,
    pub seq: u16,
    pub op: u16,
    pub status: u8,
    pub body: &'a [u8],
}

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

    fn shift_sof(&mut self) {
        if self.len > 1 {
            self.buf.copy_within(1..self.len, 0);
            self.len -= 1;
        } else {
            self.len = 0;
        }
        self.align_sof();
    }

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

pub fn read_u16(body: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*body.get(at)?, *body.get(at + 1)?]))
}

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
