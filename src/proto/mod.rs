//! The SD protocol as pure functions: command frames, CRC7, the four-line CRC16, responses and
//! card registers. No hardware, no time, no allocation.

mod command;
mod crc;
mod registers;
mod response;

pub use command::{Command, ResponseKind};
pub use crc::{crc7, crc16, crc16_4bit};
pub use registers::{Cid, Csd, CsdVersion, Ocr, RegisterError, Scr};
pub use response::{
    CardState, CardStatus, CrcStatus, IfCond, ResponseError, decode_r1, decode_r2, decode_r3,
    decode_r6, decode_r7,
};
