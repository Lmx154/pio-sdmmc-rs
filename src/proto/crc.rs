//! SD CRCs (Physical Layer Simplified Specification, section 4.5).

/// CRC7 of `bytes`, MSB first (x^7 + x^3 + 1, initial value 0), as used for command and
/// response frames (section 4.5). The result is in `0..=0x7F`.
pub const fn crc7(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    let mut i = 0;
    while i < bytes.len() {
        let byte = bytes[i];
        let mut bit = 0;
        while bit < 8 {
            let data = (byte >> (7 - bit)) & 1;
            let feedback = ((crc >> 6) & 1) ^ data;
            crc = (crc << 1) & 0x7F;
            if feedback != 0 {
                crc ^= 0x09;
            }
            bit += 1;
        }
        i += 1;
    }
    crc
}

/// CRC16-CCITT of `bytes` as one serial bit stream, MSB first (x^16 + x^12 + x^5 + 1, initial
/// value 0): the CRC of a single data line (section 4.5).
pub const fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0u16;
    let mut i = 0;
    while i < bytes.len() {
        crc ^= (bytes[i] as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            bit += 1;
        }
        i += 1;
    }
    crc
}

/// CRC16 of a four-bit-mode data block, in the wire form used by
/// [`Bus`](crate::bus::Bus) ("Data CRCs"): sixteen nibbles, the first sent in bits 63..60.
///
/// Each byte goes out high nibble first, nibble bit 3 on DAT3 down to bit 0 on DAT0, and bit `k`
/// of every CRC nibble is the CRC16 of line DATk (section 4.5, "CRC16 on each data line"). The
/// state holds the four line CRCs bit-interleaved, so one nibble is one step of all four.
///
/// The nearest feedback tap is four state bits (one CRC bit) from the top, so a step may take
/// 16 data bits at once: `t = (s >> 48) ^ d; s = (s << 16) ^ t ^ (t << 20) ^ (t << 48)`. Two
/// such steps fold into one 32-bit step on the two `u32` halves of the state (`hi`, `lo`):
/// with `x = hi ^ d`, `t1 = x >> 16` and `t2 = (x & 0xFFFF) ^ t1`, the new state is
/// `lo' = t2 ^ (t2 << 20) ^ (t1 << 16)` and `hi' = lo ^ (t1 << 4) ^ (t2 >> 12) ^ (t2 << 16)`.
/// That is about fourteen 32-bit operations per four bytes and no 64-bit shift.
pub fn crc16_4bit(block: &[u8]) -> u64 {
    let mut hi = 0u32;
    let mut lo = 0u32;
    let (words, tail) = block.as_chunks::<4>();
    for &w in words {
        let x = hi ^ u32::from_be_bytes(w);
        let t1 = x >> 16;
        let t2 = (x & 0xFFFF) ^ t1;
        hi = lo ^ (t1 << 4) ^ (t2 >> 12) ^ (t2 << 16);
        lo = t2 ^ (t2 << 20) ^ (t1 << 16);
    }
    let mut s = (u64::from(hi) << 32) | u64::from(lo);
    for &byte in tail {
        s = nibble_step(s, byte >> 4);
        s = nibble_step(s, byte & 0xF);
    }
    s
}

/// One nibble of the interleaved four-line CRC16.
const fn nibble_step(s: u64, d: u8) -> u64 {
    let t = (s >> 60) ^ d as u64;
    (s << 4) ^ t ^ (t << 20) ^ (t << 48)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Xorshift(u64);

    impl Xorshift {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn fill(&mut self, buf: &mut [u8]) {
            for b in buf {
                *b = (self.next() >> 24) as u8;
            }
        }
    }

    /// The nibble-at-a-time form, kept as a reference.
    fn crc16_4bit_nibbles(block: &[u8]) -> u64 {
        let mut s = 0u64;
        for &byte in block {
            s = nibble_step(s, byte >> 4);
            s = nibble_step(s, byte & 0xF);
        }
        s
    }

    /// Plain bit-serial CRC16 over a stream of single bits.
    fn crc16_bits(bits: &[u8]) -> u16 {
        let mut crc = 0u16;
        for &b in bits {
            let feedback = ((crc >> 15) & 1) ^ u16::from(b);
            crc <<= 1;
            if feedback != 0 {
                crc ^= 0x1021;
            }
        }
        crc
    }

    /// Independent reference: split into four per-line bit streams, CRC each, interleave.
    fn crc16_4bit_reference(block: &[u8]) -> u64 {
        let mut lines: [Vec<u8>; 4] = Default::default();
        for &byte in block {
            for nibble in [byte >> 4, byte & 0xF] {
                for (k, line) in lines.iter_mut().enumerate() {
                    line.push((nibble >> k) & 1);
                }
            }
        }
        let crcs: [u16; 4] = core::array::from_fn(|k| crc16_bits(&lines[k]));
        let mut out = 0u64;
        for i in 0..16 {
            let mut nibble = 0u64;
            for (k, crc) in crcs.iter().enumerate() {
                nibble |= u64::from((crc >> (15 - i)) & 1) << k;
            }
            out |= nibble << (60 - 4 * i);
        }
        out
    }

    #[test]
    fn crc7_known_frames() {
        assert_eq!(crc7(&[0x40, 0, 0, 0, 0]), 0x4A);
        assert_eq!(crc7(&[0x48, 0, 0, 0x01, 0xAA]), 0x43);
        assert_eq!(crc7(&[0x77, 0, 0, 0, 0]), 0x32);
        assert_eq!(crc7(&[0x51, 0, 0, 0, 0]), 0x2A);
        assert_eq!(crc7(&[]), 0);
    }

    #[test]
    fn crc7_single_bit_flips_change_crc() {
        let frame = [0x51u8, 0x12, 0x34, 0x56, 0x78];
        let good = crc7(&frame);
        for bit in 0..40 {
            let mut f = frame;
            f[bit / 8] ^= 0x80 >> (bit % 8);
            assert_ne!(crc7(&f), good, "bit {bit}");
        }
    }

    #[test]
    fn crc16_known_vector() {
        assert_eq!(crc16(&[0xFF; 512]), 0x7FA1);
        assert_eq!(crc16(&[]), 0);
    }

    #[test]
    fn crc16_single_bit_flips_change_crc() {
        let mut rng = Xorshift(0x1234_5678_9ABC_DEF1);
        let mut block = [0u8; 64];
        rng.fill(&mut block);
        let good = crc16(&block);
        for bit in 0..block.len() * 8 {
            let mut b = block;
            b[bit / 8] ^= 0x80 >> (bit % 8);
            assert_ne!(crc16(&b), good, "bit {bit}");
        }
    }

    #[test]
    fn crc16_4bit_matches_references() {
        let mut rng = Xorshift(0x9E37_79B9_7F4A_7C15);
        for len in [0usize, 1, 2, 3, 4, 8, 63, 64, 512, 513] {
            for _ in 0..300 {
                let mut block = vec![0u8; len];
                rng.fill(&mut block);
                let fast = crc16_4bit(&block);
                assert_eq!(fast, crc16_4bit_nibbles(&block), "nibbles, len {len}");
                assert_eq!(fast, crc16_4bit_reference(&block), "per-line, len {len}");
            }
        }
        for len in [4usize, 512] {
            assert_eq!(
                crc16_4bit(&vec![0xFF; len]),
                crc16_4bit_reference(&vec![0xFF; len])
            );
            assert_eq!(crc16_4bit(&vec![0x00; len]), 0);
        }
    }

    #[test]
    #[ignore = "timing; run with --release -- --ignored --nocapture"]
    fn crc16_4bit_timing() {
        use std::hint::black_box;
        use std::time::Instant;
        const BLOCKS: u32 = 20_000;
        let mut rng = Xorshift(0x0123_4567_89AB_CDEF);
        let mut block = [0u8; 512];
        rng.fill(&mut block);
        let mut acc = 0u64;
        let t = Instant::now();
        for i in 0..BLOCKS {
            block[0] = i as u8;
            acc ^= crc16_4bit_nibbles(black_box(&block));
        }
        let old = t.elapsed().as_nanos() / u128::from(BLOCKS);
        let t = Instant::now();
        for i in 0..BLOCKS {
            block[0] = i as u8;
            acc ^= crc16_4bit(black_box(&block));
        }
        let new = t.elapsed().as_nanos() / u128::from(BLOCKS);
        black_box(acc);
        println!("crc16_4bit, 512 bytes: nibble form {old} ns/block, 32-bit form {new} ns/block");
    }

    #[test]
    fn crc16_4bit_single_bit_flips_change_crc() {
        let mut rng = Xorshift(0xDEAD_BEEF_CAFE_F00D);
        let mut block = [0u8; 64];
        rng.fill(&mut block);
        let good = crc16_4bit(&block);
        for bit in 0..block.len() * 8 {
            let mut b = block;
            b[bit / 8] ^= 0x80 >> (bit % 8);
            assert_ne!(crc16_4bit(&b), good, "bit {bit}");
        }
    }
}
