//! Responses and the card status (Physical Layer Simplified Specification, sections 4.9 and
//! 4.10). Decoders take the raw frame as [`Bus::command`](crate::bus::Bus::command) delivers it:
//! the first byte holds the start bit, the transmission bit and the six index (or reserved) bits.

use super::crc::crc7;
use super::registers::Ocr;

/// Why a response frame was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResponseError {
    /// The start bit was not 0.
    StartBit,
    /// The transmission bit was not 0 (the frame came from the host, not a card).
    TransmissionBit,
    /// The command index field was not the one expected.
    WrongIndex {
        /// The index the decoder required.
        expected: u8,
        /// The index in the frame.
        got: u8,
    },
    /// The CRC7 in the frame does not match the one computed over its contents.
    Crc {
        /// CRC7 computed over the frame.
        expected: u8,
        /// CRC7 in the frame.
        got: u8,
    },
    /// The end bit was not 1.
    EndBit,
    /// Bits that the format fixes to 1 were not: the index field of R2 and R3, or the CRC field
    /// of R3.
    ReservedBits,
}

fn check_header(first: u8) -> Result<(), ResponseError> {
    if first & 0x80 != 0 {
        Err(ResponseError::StartBit)
    } else if first & 0x40 != 0 {
        Err(ResponseError::TransmissionBit)
    } else {
        Ok(())
    }
}

/// Checks a 48-bit response that carries a command index and CRC7 (R1, R1b, R6, R7) and returns
/// its 32-bit payload.
fn decode_short(index: u8, frame: &[u8; 6]) -> Result<u32, ResponseError> {
    check_header(frame[0])?;
    if frame[5] & 1 == 0 {
        return Err(ResponseError::EndBit);
    }
    let expected = crc7(&frame[..5]);
    let got = frame[5] >> 1;
    if expected != got {
        return Err(ResponseError::Crc { expected, got });
    }
    let got = frame[0] & 0x3F;
    if got != index {
        return Err(ResponseError::WrongIndex {
            expected: index,
            got,
        });
    }
    Ok(u32::from_be_bytes([frame[1], frame[2], frame[3], frame[4]]))
}

/// Decodes an R1 or R1b response to command `index` into its card status (section 4.9.1).
pub fn decode_r1(index: u8, frame: &[u8; 6]) -> Result<CardStatus, ResponseError> {
    decode_short(index, frame).map(CardStatus)
}

/// Decodes an R2 response (CID or CSD, section 4.9.2) and returns the register bytes
/// `[127:0]`, most significant byte first. Checks the CRC7 the register carries in bits 7:1
/// over bits 127:8.
pub fn decode_r2(frame: &[u8; 17]) -> Result<[u8; 16], ResponseError> {
    check_header(frame[0])?;
    if frame[16] & 1 == 0 {
        return Err(ResponseError::EndBit);
    }
    if frame[0] & 0x3F != 0x3F {
        return Err(ResponseError::ReservedBits);
    }
    let expected = crc7(&frame[1..16]);
    let got = frame[16] >> 1;
    if expected != got {
        return Err(ResponseError::Crc { expected, got });
    }
    let mut reg = [0u8; 16];
    reg.copy_from_slice(&frame[1..]);
    Ok(reg)
}

/// Decodes an R3 response (the OCR, section 4.9.4). It has no CRC; its index and CRC fields
/// are all 1.
pub fn decode_r3(frame: &[u8; 6]) -> Result<Ocr, ResponseError> {
    check_header(frame[0])?;
    if frame[5] & 1 == 0 {
        return Err(ResponseError::EndBit);
    }
    if frame[0] & 0x3F != 0x3F || frame[5] != 0xFF {
        return Err(ResponseError::ReservedBits);
    }
    Ok(Ocr(u32::from_be_bytes([
        frame[1], frame[2], frame[3], frame[4],
    ])))
}

/// Decodes an R6 response to CMD3 (section 4.9.5): the published RCA and the 16 status bits
/// expanded to their card-status positions (bits 15, 14, 13 become 23, 22, 19; bits 12:0 stay).
pub fn decode_r6(frame: &[u8; 6]) -> Result<(u16, CardStatus), ResponseError> {
    let payload = decode_short(3, frame)?;
    let rca = (payload >> 16) as u16;
    let s = payload & 0xFFFF;
    let status =
        (s & 0x1FFF) | ((s >> 15) & 1) << 23 | ((s >> 14) & 1) << 22 | ((s >> 13) & 1) << 19;
    Ok((rca, CardStatus(status)))
}

/// The card interface condition returned in R7 (section 4.9.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IfCond {
    /// Voltage accepted (VCA, argument bits 11:8). `0b0001` is 2.7-3.6 V.
    pub voltage_accepted: u8,
    /// The check pattern the card echoed (bits 7:0).
    pub check_pattern: u8,
}

/// Decodes an R7 response to CMD8 (section 4.9.6).
pub fn decode_r7(frame: &[u8; 6]) -> Result<IfCond, ResponseError> {
    let payload = decode_short(8, frame)?;
    Ok(IfCond {
        voltage_accepted: ((payload >> 8) & 0xF) as u8,
        check_pattern: payload as u8,
    })
}

/// The 32-bit card status (R1 payload, section 4.10, table 4-42).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CardStatus(pub u32);

impl CardStatus {
    /// Bit 31: the argument was out of the card's range.
    pub const OUT_OF_RANGE: u32 = 1 << 31;
    /// Bit 30: a misaligned address did not match the block length.
    pub const ADDRESS_ERROR: u32 = 1 << 30;
    /// Bit 29: the block length is not allowed for this card, or the transferred length does
    /// not match.
    pub const BLOCK_LEN_ERROR: u32 = 1 << 29;
    /// Bit 28: an error in the sequence of erase commands.
    pub const ERASE_SEQ_ERROR: u32 = 1 << 28;
    /// Bit 27: an invalid selection of write blocks for erase.
    pub const ERASE_PARAM: u32 = 1 << 27;
    /// Bit 26: write to a protected block, or to a locked card.
    pub const WP_VIOLATION: u32 = 1 << 26;
    /// Bit 25: the card is locked by the host (a status, not an error).
    pub const CARD_IS_LOCKED: u32 = 1 << 25;
    /// Bit 24: a lock/unlock command sequence error or a wrong-password attempt.
    pub const LOCK_UNLOCK_FAILED: u32 = 1 << 24;
    /// Bit 23: the CRC of the previous command failed.
    pub const COM_CRC_ERROR: u32 = 1 << 23;
    /// Bit 22: the command is not legal in the card's current state.
    pub const ILLEGAL_COMMAND: u32 = 1 << 22;
    /// Bit 21: the card's internal ECC was applied but failed to correct the data.
    pub const CARD_ECC_FAILED: u32 = 1 << 21;
    /// Bit 20: an internal card controller error.
    pub const CC_ERROR: u32 = 1 << 20;
    /// Bit 19: a general or unknown error.
    pub const ERROR: u32 = 1 << 19;
    /// Bit 16: the CSD write was incompatible with the card's read-only fields, or a
    /// permanent write-protect was reversed.
    pub const CSD_OVERWRITE: u32 = 1 << 16;
    /// Bit 15: only part of an erase range was erased because of write protection.
    pub const WP_ERASE_SKIP: u32 = 1 << 15;
    /// Bit 14: the card's internal ECC was disabled by the host.
    pub const CARD_ECC_DISABLED: u32 = 1 << 14;
    /// Bit 13: an erase sequence was cleared before it was executed.
    pub const ERASE_RESET: u32 = 1 << 13;
    /// Bit 8: the card's buffer is empty and it can accept data.
    pub const READY_FOR_DATA: u32 = 1 << 8;
    /// Bit 7: the card has a function-extension event pending.
    pub const FX_EVENT: u32 = 1 << 7;
    /// Bit 5: the card expects an application command next.
    pub const APP_CMD: u32 = 1 << 5;
    /// Bit 3: an error in the sequence of the authentication process.
    pub const AKE_SEQ_ERROR: u32 = 1 << 3;
    /// Every error bit: `OUT_OF_RANGE`, `ADDRESS_ERROR`, `BLOCK_LEN_ERROR`, `ERASE_SEQ_ERROR`,
    /// `ERASE_PARAM`, `WP_VIOLATION`, `LOCK_UNLOCK_FAILED`, `COM_CRC_ERROR`, `ILLEGAL_COMMAND`,
    /// `CARD_ECC_FAILED`, `CC_ERROR`, `ERROR`, `CSD_OVERWRITE`, `WP_ERASE_SKIP` and
    /// `AKE_SEQ_ERROR`.
    pub const ERROR_MASK: u32 = Self::OUT_OF_RANGE
        | Self::ADDRESS_ERROR
        | Self::BLOCK_LEN_ERROR
        | Self::ERASE_SEQ_ERROR
        | Self::ERASE_PARAM
        | Self::WP_VIOLATION
        | Self::LOCK_UNLOCK_FAILED
        | Self::COM_CRC_ERROR
        | Self::ILLEGAL_COMMAND
        | Self::CARD_ECC_FAILED
        | Self::CC_ERROR
        | Self::ERROR
        | Self::CSD_OVERWRITE
        | Self::WP_ERASE_SKIP
        | Self::AKE_SEQ_ERROR;

    /// The error bits that are set (`self.0 & ERROR_MASK`); zero means no error.
    pub const fn errors(&self) -> u32 {
        self.0 & Self::ERROR_MASK
    }

    /// CURRENT_STATE, bits 12:9.
    pub const fn state(&self) -> CardState {
        match (self.0 >> 9) & 0xF {
            0 => CardState::Idle,
            1 => CardState::Ready,
            2 => CardState::Ident,
            3 => CardState::Stby,
            4 => CardState::Tran,
            5 => CardState::Data,
            6 => CardState::Rcv,
            7 => CardState::Prg,
            8 => CardState::Dis,
            n => CardState::Reserved(n as u8),
        }
    }

    /// READY_FOR_DATA, bit 8.
    pub const fn ready_for_data(&self) -> bool {
        self.0 & Self::READY_FOR_DATA != 0
    }

    /// APP_CMD, bit 5.
    pub const fn app_cmd(&self) -> bool {
        self.0 & Self::APP_CMD != 0
    }
}

/// The card state machine state, CURRENT_STATE of the card status (section 4.10.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CardState {
    /// 0: idle.
    Idle,
    /// 1: ready.
    Ready,
    /// 2: identification.
    Ident,
    /// 3: stand-by.
    Stby,
    /// 4: transfer.
    Tran,
    /// 5: sending data.
    Data,
    /// 6: receiving data.
    Rcv,
    /// 7: programming.
    Prg,
    /// 8: disconnect.
    Dis,
    /// 9..=15: reserved (15 is reserved for I/O mode).
    Reserved(u8),
}

/// The card's CRC status token after a write data block (section 4.3.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrcStatus {
    /// `010`: the block was accepted.
    Accepted,
    /// `101`: the block was rejected for a CRC error.
    CrcError,
    /// `110`: the block was rejected for a write error.
    WriteError,
    /// Any other value, as received.
    Invalid(u8),
}

impl CrcStatus {
    /// Interprets the 3-bit token returned by [`Bus::write_block`](crate::bus::Bus::write_block).
    pub const fn from_token(token: u8) -> Self {
        match token {
            0b010 => Self::Accepted,
            0b101 => Self::CrcError,
            0b110 => Self::WriteError,
            other => Self::Invalid(other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn short(index: u8, payload: [u8; 4]) -> [u8; 6] {
        let mut f = [
            index & 0x3F,
            payload[0],
            payload[1],
            payload[2],
            payload[3],
            0,
        ];
        f[5] = (crc7(&f[..5]) << 1) | 1;
        f
    }

    fn r3(ocr: u32) -> [u8; 6] {
        let p = ocr.to_be_bytes();
        [0x3F, p[0], p[1], p[2], p[3], 0xFF]
    }

    fn r2(reg: [u8; 16]) -> [u8; 17] {
        let mut f = [0u8; 17];
        f[0] = 0x3F;
        f[1..].copy_from_slice(&reg);
        f[16] = (crc7(&f[1..16]) << 1) | 1;
        f
    }

    #[test]
    fn r1_round_trip() {
        let f = short(17, 0x0000_0900u32.to_be_bytes());
        let s = decode_r1(17, &f).unwrap();
        assert_eq!(s, CardStatus(0x900));
        assert_eq!(s.state(), CardState::Tran);
        assert!(s.ready_for_data());
        assert!(!s.app_cmd());
        assert_eq!(s.errors(), 0);
    }

    #[test]
    fn r1_known_frame() {
        // The reply to CMD55 in idle state: APP_CMD and READY_FOR_DATA set.
        let f = short(55, 0x0000_0120u32.to_be_bytes());
        assert_eq!(f[0], 0x37);
        let s = decode_r1(55, &f).unwrap();
        assert!(s.app_cmd() && s.ready_for_data());
        assert_eq!(s.state(), CardState::Idle);
    }

    #[test]
    fn r3_round_trip() {
        let ocr = decode_r3(&r3(0xC0FF_8000)).unwrap();
        assert_eq!(ocr, Ocr(0xC0FF_8000));
    }

    #[test]
    fn r6_round_trip_and_status_expansion() {
        // RCA 0x5678; compressed status: COM_CRC_ERROR, ILLEGAL_COMMAND, ERROR, state 4, ready.
        let compressed: u16 = 1 << 15 | 1 << 14 | 1 << 13 | 4 << 9 | 1 << 8 | 1 << 5 | 1 << 3;
        let payload = (0x5678u32 << 16) | u32::from(compressed);
        let f = short(3, payload.to_be_bytes());
        let (rca, status) = decode_r6(&f).unwrap();
        assert_eq!(rca, 0x5678);
        assert_eq!(
            status.0,
            CardStatus::COM_CRC_ERROR
                | CardStatus::ILLEGAL_COMMAND
                | CardStatus::ERROR
                | 4 << 9
                | CardStatus::READY_FOR_DATA
                | CardStatus::APP_CMD
                | CardStatus::AKE_SEQ_ERROR
        );
        assert_eq!(status.state(), CardState::Tran);
        let (_, clean) = decode_r6(&short(3, [0x00, 0x01, 0x07, 0x00])).unwrap();
        assert_eq!(clean, CardStatus(0x0700));
    }

    #[test]
    fn r7_round_trip() {
        let f = short(8, [0, 0, 0x01, 0xAA]);
        assert_eq!(
            decode_r7(&f).unwrap(),
            IfCond {
                voltage_accepted: 1,
                check_pattern: 0xAA
            }
        );
        let f = short(8, [0xFF, 0xFF, 0xF2, 0x5A]);
        assert_eq!(
            decode_r7(&f).unwrap(),
            IfCond {
                voltage_accepted: 2,
                check_pattern: 0x5A
            }
        );
    }

    #[test]
    fn r2_round_trip() {
        let mut reg = [0u8; 16];
        for (i, b) in reg.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(37).wrapping_add(11);
        }
        let f = r2(reg);
        let mut want = reg;
        want[15] = f[16];
        assert_eq!(decode_r2(&f).unwrap(), want);
    }

    #[test]
    fn reject_start_bit() {
        let mut f = short(17, [0; 4]);
        f[0] |= 0x80;
        assert_eq!(decode_r1(17, &f), Err(ResponseError::StartBit));
        assert_eq!(
            decode_r3(&{
                let mut g = r3(0);
                g[0] |= 0x80;
                g
            }),
            Err(ResponseError::StartBit)
        );
        assert_eq!(
            decode_r2(&{
                let mut g = r2([0; 16]);
                g[0] |= 0x80;
                g
            }),
            Err(ResponseError::StartBit)
        );
    }

    #[test]
    fn reject_transmission_bit() {
        let mut f = short(17, [0; 4]);
        f[0] |= 0x40;
        assert_eq!(decode_r1(17, &f), Err(ResponseError::TransmissionBit));
        assert_eq!(decode_r7(&f), Err(ResponseError::TransmissionBit));
    }

    #[test]
    fn reject_wrong_index() {
        let f = short(16, [0; 4]);
        assert_eq!(
            decode_r1(17, &f),
            Err(ResponseError::WrongIndex {
                expected: 17,
                got: 16
            })
        );
        assert_eq!(
            decode_r6(&f),
            Err(ResponseError::WrongIndex {
                expected: 3,
                got: 16
            })
        );
        assert_eq!(
            decode_r7(&f),
            Err(ResponseError::WrongIndex {
                expected: 8,
                got: 16
            })
        );
    }

    #[test]
    fn reject_crc_mismatch() {
        let mut f = short(17, [0, 0, 9, 0]);
        f[3] ^= 0x01;
        let err = decode_r1(17, &f).unwrap_err();
        assert!(matches!(err, ResponseError::Crc { expected, got } if expected != got));
        let mut g = r2([0x55; 16]);
        g[7] ^= 0x10;
        assert!(matches!(decode_r2(&g), Err(ResponseError::Crc { .. })));
    }

    #[test]
    fn reject_end_bit() {
        let mut f = short(17, [0; 4]);
        f[5] &= !1;
        assert_eq!(decode_r1(17, &f), Err(ResponseError::EndBit));
        let mut g = r3(0);
        g[5] = 0xFE;
        assert_eq!(decode_r3(&g), Err(ResponseError::EndBit));
        let mut h = r2([0; 16]);
        h[16] &= !1;
        assert_eq!(decode_r2(&h), Err(ResponseError::EndBit));
    }

    #[test]
    fn reject_reserved_bits() {
        let mut g = r3(0);
        g[0] = 0x3E;
        assert_eq!(decode_r3(&g), Err(ResponseError::ReservedBits));
        let mut g = r3(0);
        g[5] = 0xFD;
        assert_eq!(decode_r3(&g), Err(ResponseError::ReservedBits));
        let mut h = r2([0; 16]);
        h[0] = 0x00;
        assert_eq!(decode_r2(&h), Err(ResponseError::ReservedBits));
        // An R1 frame is not an R3 frame.
        assert_eq!(
            decode_r3(&short(41, [0; 4])),
            Err(ResponseError::ReservedBits)
        );
    }

    #[test]
    fn decoders_never_panic() {
        let mut f = [0u8; 6];
        for a in 0..=255u8 {
            f[0] = a;
            for b in [0u8, 1, 0x80, 0xFF] {
                f[5] = b;
                let _ = decode_r1(0, &f);
                let _ = decode_r3(&f);
                let _ = decode_r6(&f);
                let _ = decode_r7(&f);
            }
        }
    }

    #[test]
    fn error_mask() {
        assert_eq!(CardStatus::ERROR_MASK, 0xFDF9_8008);
        assert_eq!(CardStatus(u32::MAX).errors(), 0xFDF9_8008);
        assert_eq!(
            CardStatus(CardStatus::CARD_IS_LOCKED | CardStatus::APP_CMD).errors(),
            0
        );
        assert_eq!(
            CardStatus(CardStatus::COM_CRC_ERROR).errors(),
            CardStatus::COM_CRC_ERROR
        );
    }

    #[test]
    fn states() {
        let want = [
            CardState::Idle,
            CardState::Ready,
            CardState::Ident,
            CardState::Stby,
            CardState::Tran,
            CardState::Data,
            CardState::Rcv,
            CardState::Prg,
            CardState::Dis,
            CardState::Reserved(9),
            CardState::Reserved(10),
            CardState::Reserved(11),
            CardState::Reserved(12),
            CardState::Reserved(13),
            CardState::Reserved(14),
            CardState::Reserved(15),
        ];
        for (n, state) in want.into_iter().enumerate() {
            assert_eq!(CardStatus((n as u32) << 9 | 0xFFFF_E1FF).state(), state);
        }
    }

    #[test]
    fn crc_status_tokens() {
        assert_eq!(CrcStatus::from_token(0b010), CrcStatus::Accepted);
        assert_eq!(CrcStatus::from_token(0b101), CrcStatus::CrcError);
        assert_eq!(CrcStatus::from_token(0b110), CrcStatus::WriteError);
        assert_eq!(CrcStatus::from_token(0b111), CrcStatus::Invalid(0b111));
        assert_eq!(CrcStatus::from_token(0), CrcStatus::Invalid(0));
    }
}
