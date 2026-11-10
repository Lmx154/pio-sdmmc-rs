//! The card's registers as they appear on the wire.

use super::config::{CardKind, Cid, SimConfig};
use super::crc::crc7;

fn put(reg: &mut u128, msb: u32, lsb: u32, value: u128) {
    let width = msb - lsb + 1;
    let mask = (1u128 << width) - 1;
    debug_assert!(value <= mask, "field value too wide");
    *reg = (*reg & !(mask << lsb)) | ((value & mask) << lsb);
}

/// Sets the register's CRC7 (bits 7..1) over bits 127..8, and its end bit, and returns the bytes.
fn seal(mut reg: u128) -> [u8; 16] {
    let bytes = reg.to_be_bytes();
    let crc = crc7(&bytes[..15]);
    put(&mut reg, 7, 0, (u128::from(crc) << 1) | 1);
    reg.to_be_bytes()
}

pub(super) fn cid(c: &Cid) -> [u8; 16] {
    let mut reg = 0u128;
    put(&mut reg, 127, 120, u128::from(c.manufacturer));
    put(&mut reg, 119, 104, u128::from(u16::from_be_bytes(c.oem)));
    let mut pnm = 0u128;
    for b in c.product {
        pnm = (pnm << 8) | u128::from(b);
    }
    put(&mut reg, 103, 64, pnm);
    put(&mut reg, 63, 56, u128::from(c.revision));
    put(&mut reg, 55, 24, u128::from(c.serial));
    put(&mut reg, 19, 8, u128::from(c.date & 0x0FFF));
    seal(reg)
}

pub(super) fn csd(cfg: &SimConfig) -> [u8; 16] {
    let mut reg = 0u128;
    let blocks = u128::from(cfg.capacity_blocks);
    if cfg.kind.is_high_capacity() {
        put(&mut reg, 127, 126, 1);
        put(&mut reg, 119, 112, 0x0E);
        put(&mut reg, 103, 96, 0x32);
        put(&mut reg, 95, 84, 0x5B5);
        put(&mut reg, 83, 80, 9);
        let c_size = (blocks / 1024).saturating_sub(1);
        put(&mut reg, 69, 48, c_size & 0x3F_FFFF);
        put(&mut reg, 46, 46, 1);
        put(&mut reg, 45, 39, 0x7F);
        put(&mut reg, 28, 26, 2);
        put(&mut reg, 25, 22, 9);
    } else {
        put(&mut reg, 127, 126, 0);
        put(&mut reg, 119, 112, 0x0E);
        put(&mut reg, 103, 96, 0x32);
        put(&mut reg, 95, 84, 0x5B5);
        put(&mut reg, 83, 80, 9);
        // capacity = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) blocks of 512 bytes.
        let mut mult = 0u128;
        while mult < 7 && (blocks >> (mult + 2)) > 4096 {
            mult += 1;
        }
        let c_size = (blocks >> (mult + 2)).clamp(1, 4096) - 1;
        put(&mut reg, 73, 62, c_size);
        put(&mut reg, 49, 47, mult);
        put(&mut reg, 46, 46, 1);
        put(&mut reg, 45, 39, 0x7F);
        put(&mut reg, 28, 26, 2);
        put(&mut reg, 25, 22, 9);
    }
    seal(reg)
}

pub(super) fn scr(cfg: &SimConfig) -> [u8; 8] {
    let (sd_spec, spec3, security) = match cfg.kind {
        CardKind::SdscV1 => (1u8, 0u8, 2u8),
        CardKind::SdscV2 => (2, 0, 2),
        CardKind::Sdhc => (2, 1, 3),
        CardKind::Sdxc => (2, 1, 4),
    };
    let mut b = [0u8; 8];
    b[0] = sd_spec;
    b[1] = (security << 4) | 0x05;
    b[2] = spec3 << 7;
    b[3] = if cfg.supports_cmd23 { 0x02 } else { 0x00 };
    b
}

pub(super) fn sd_status(four_bit: bool) -> [u8; 64] {
    let mut b = [0u8; 64];
    b[0] = if four_bit { 0b10 << 6 } else { 0 };
    b[8] = 4;
    b[10] = 0x90;
    b
}
