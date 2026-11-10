//! Test-local helpers for driving the simulated card through raw bus calls. Frame building and
//! CRCs are written here on purpose, independently of both the crate's protocol layer and the
//! simulator's own code.
#![allow(dead_code)]

use pio_sdmmc::sim::{SimCard, SimConfig};
use pio_sdmmc::{Bus, BusError, ResponseBuf};

pub const RCA: u16 = 0xAAAA;

/// CRC7 by a byte-wise table over a left-aligned register.
pub fn crc7(bytes: &[u8]) -> u8 {
    let mut table = [0u8; 256];
    for (i, slot) in table.iter_mut().enumerate() {
        let mut v = i as u8;
        for _ in 0..8 {
            v = if v & 0x80 != 0 {
                (v << 1) ^ (0x09 << 1)
            } else {
                v << 1
            };
        }
        *slot = v;
    }
    let mut crc = 0u8;
    for &b in bytes {
        crc = table[(crc ^ b) as usize];
    }
    crc >> 1
}

/// The 64-bit wire CRC of a block: each line's bit stream is packed into bytes and run through
/// a byte-wise CRC-16/XMODEM.
pub fn data_crc(block: &[u8]) -> u64 {
    let nibbles: Vec<u8> = block.iter().flat_map(|b| [b >> 4, b & 15]).collect();
    assert!(nibbles.len().is_multiple_of(8));
    let mut crcs = [0u16; 4];
    for (k, out) in crcs.iter_mut().enumerate() {
        let mut crc = 0u16;
        for chunk in nibbles.chunks(8) {
            let mut byte = 0u8;
            for (i, n) in chunk.iter().enumerate() {
                byte |= ((n >> k) & 1) << (7 - i);
            }
            crc ^= u16::from(byte) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 {
                    (crc << 1) ^ 0x1021
                } else {
                    crc << 1
                };
            }
        }
        *out = crc;
    }
    let mut out = 0u64;
    for i in 0..16 {
        for (k, crc) in crcs.iter().enumerate() {
            out |= u64::from((crc >> (15 - i)) & 1) << (60 - 4 * i + k);
        }
    }
    out
}

pub fn frame(index: u8, arg: u32) -> [u8; 6] {
    let a = arg.to_be_bytes();
    let mut f = [0x40 | index, a[0], a[1], a[2], a[3], 0];
    f[5] = (crc7(&f[..5]) << 1) | 1;
    f
}

pub async fn cmd(card: &mut SimCard, index: u8, arg: u32) -> Result<[u8; 6], BusError> {
    let mut r = [0u8; 6];
    card.command(&frame(index, arg), ResponseBuf::Short(&mut r))
        .await?;
    Ok(r)
}

pub async fn cmd_none(card: &mut SimCard, index: u8, arg: u32) -> Result<(), BusError> {
    card.command(&frame(index, arg), ResponseBuf::None).await
}

pub async fn cmd_long(card: &mut SimCard, index: u8, arg: u32) -> Result<[u8; 17], BusError> {
    let mut r = [0u8; 17];
    card.command(&frame(index, arg), ResponseBuf::Long(&mut r))
        .await?;
    Ok(r)
}

/// Sends CMD55 then the command.
pub async fn acmd(card: &mut SimCard, rca: u16, index: u8, arg: u32) -> Result<[u8; 6], BusError> {
    cmd(card, 55, u32::from(rca) << 16).await?;
    cmd(card, index, arg).await
}

/// Checks an R1-shaped frame and returns its 32 status bits.
pub fn status(resp: &[u8; 6], index: u8) -> u32 {
    assert_eq!(resp[0], index, "response index");
    assert_eq!(resp[5], (crc7(&resp[..5]) << 1) | 1, "response CRC");
    u32::from_be_bytes([resp[1], resp[2], resp[3], resp[4]])
}

pub fn state_field(status: u32) -> u32 {
    (status >> 9) & 0xF
}

pub const ILLEGAL: u32 = 1 << 22;
pub const COM_CRC: u32 = 1 << 23;
pub const OUT_OF_RANGE: u32 = 1 << 31;
pub const ADDRESS_ERROR: u32 = 1 << 30;
pub const ERROR_BIT: u32 = 1 << 19;
pub const APP_CMD: u32 = 1 << 5;
pub const READY_FOR_DATA: u32 = 1 << 8;

/// Power-up clocks, CMD0, CMD8, ACMD41 until ready, CMD2, CMD3. Returns the final OCR. Leaves the
/// card in stand-by at 400 kHz.
pub async fn identify(card: &mut SimCard, hcs: bool) -> u32 {
    assert_eq!(card.set_clock(400_000), 400_000);
    card.clock_idle(80).await;
    cmd_none(card, 0, 0).await.unwrap();
    let _ = cmd(card, 8, 0x1AA).await;
    let arg = 0x00FF_8000 | if hcs { 1 << 30 } else { 0 };
    let mut ocr = 0;
    for _ in 0..100 {
        cmd(card, 55, 0).await.unwrap();
        let r = cmd(card, 41, arg).await.unwrap();
        assert_eq!(r[0], 0x3F);
        assert_eq!(r[5], 0xFF);
        ocr = u32::from_be_bytes([r[1], r[2], r[3], r[4]]);
        if ocr & 0x8000_0000 != 0 {
            break;
        }
        card.delay_ms(10).await;
    }
    assert!(ocr & 0x8000_0000 != 0, "card never became ready");
    let cid = cmd_long(card, 2, 0).await.unwrap();
    assert_eq!(cid[0], 0x3F);
    let r6 = cmd(card, 3, 0).await.unwrap();
    assert_eq!(r6[0], 3);
    assert_eq!(r6[5], (crc7(&r6[..5]) << 1) | 1);
    assert_eq!(u16::from_be_bytes([r6[1], r6[2]]), RCA);
    ocr
}

/// Speeds up to 25 MHz, selects the card and switches to four-bit mode.
pub async fn select_and_widen(card: &mut SimCard) {
    assert_eq!(card.set_clock(25_000_000), 25_000_000);
    let r = cmd(card, 7, u32::from(RCA) << 16).await.unwrap();
    status(&r, 7);
    card.wait_ready(100).await.unwrap();
    let r = acmd(card, RCA, 6, 2).await.unwrap();
    status(&r, 6);
}

pub async fn bring_up(card: &mut SimCard) {
    identify(card, true).await;
    select_and_widen(card).await;
}

pub fn pattern(seed: u8) -> [u8; 512] {
    let mut b = [0u8; 512];
    for (i, v) in b.iter_mut().enumerate() {
        *v = (i as u8)
            .wrapping_mul(7)
            .wrapping_add(seed)
            .wrapping_add((i >> 8) as u8);
    }
    b
}

pub fn to_words(b: &[u8]) -> Vec<u32> {
    b.chunks(4)
        .map(|c| u32::from_ne_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

pub fn to_bytes(w: &[u32]) -> Vec<u8> {
    w.iter().flat_map(|x| x.to_ne_bytes()).collect()
}

/// Sends a block with its correct CRC and returns the token.
pub async fn send_block(card: &mut SimCard, data: &[u8; 512]) -> Result<u8, BusError> {
    card.write_block(&to_words(data), data_crc(data), 1000)
        .await
}

/// Reads one 512-byte block that a command has made pending; returns data and CRC.
pub async fn recv_block(card: &mut SimCard) -> Result<([u8; 512], u64), BusError> {
    let mut w = [0u32; 128];
    let crc = card.read_block(&mut w, 100).await?;
    let mut b = [0u8; 512];
    b.copy_from_slice(&to_bytes(&w));
    Ok((b, crc))
}

/// CMD24 and its block. `arg` is the address as the card takes it.
pub async fn write_single(card: &mut SimCard, arg: u32, data: &[u8; 512]) -> u8 {
    let r = cmd(card, 24, arg).await.unwrap();
    status(&r, 24);
    send_block(card, data).await.unwrap()
}

/// CMD17 and its block, checking the CRC.
pub async fn read_single(card: &mut SimCard, arg: u32) -> [u8; 512] {
    let r = cmd(card, 17, arg).await.unwrap();
    status(&r, 17);
    let (b, crc) = recv_block(card).await.unwrap();
    assert_eq!(crc, data_crc(&b));
    b
}

pub fn sim(cfg: SimConfig) -> SimCard {
    SimCard::new(cfg)
}
