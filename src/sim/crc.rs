//! Bit-serial CRCs, written out longhand so the simulator shares no code with the protocol layer.

/// CRC7 (polynomial x^7 + x^3 + 1, initial value 0) over `bytes`, most significant bit first.
pub(super) fn crc7(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let feedback = ((crc >> 6) & 1) ^ bit;
            crc = (crc << 1) & 0x7F;
            if feedback != 0 {
                crc ^= 0x09;
            }
        }
    }
    crc
}

/// The four per-line CRC16s (polynomial x^16 + x^12 + x^5 + 1, initial value 0) of a block sent
/// on four lines, packed into the 64-bit wire form of the bus contract: sixteen nibbles, the
/// first sent in bits 63..60, bit `k` of nibble `i` being bit `15 - i` of the CRC of line `k`.
pub(super) fn data_crc(bytes: &[u8]) -> u64 {
    let mut crc = [0u16; 4];
    for &byte in bytes {
        for nibble in [byte >> 4, byte & 0x0F] {
            for (line, reg) in crc.iter_mut().enumerate() {
                let bit = u16::from((nibble >> line) & 1);
                let feedback = ((*reg >> 15) & 1) ^ bit;
                *reg <<= 1;
                if feedback != 0 {
                    *reg ^= 0x1021;
                }
            }
        }
    }
    let mut out = 0u64;
    for i in 0..16 {
        let mut nibble = 0u64;
        for (line, reg) in crc.iter().enumerate() {
            nibble |= u64::from((*reg >> (15 - i)) & 1) << line;
        }
        out |= nibble << (60 - 4 * i);
    }
    out
}
