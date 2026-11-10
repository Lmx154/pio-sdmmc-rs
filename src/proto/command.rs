//! Command frames (Physical Layer Simplified Specification, sections 4.7.1 and 4.7.4).

use super::crc::crc7;

/// The response format a command expects (section 4.9).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResponseKind {
    /// No response.
    None,
    /// R1: 48 bits, card status.
    R1,
    /// R1b: R1 followed by a busy signal on DAT0.
    R1b,
    /// R2: 136 bits, CID or CSD.
    R2,
    /// R3: 48 bits, OCR, no CRC.
    R3,
    /// R6: 48 bits, published RCA and compressed card status.
    R6,
    /// R7: 48 bits, card interface condition.
    R7,
}

/// One command: index, 32-bit argument and the response it expects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Command {
    /// Command index, 0..=63. For an application command this is the ACMD number, sent after
    /// [`Command::app_cmd`].
    pub index: u8,
    /// Command argument.
    pub arg: u32,
    /// The response format to expect.
    pub response: ResponseKind,
}

impl Command {
    const fn new(index: u8, arg: u32, response: ResponseKind) -> Self {
        Self {
            index,
            arg,
            response,
        }
    }

    /// The 48-bit frame in wire order (section 4.7.1): `[0x40 | index, arg[31:24] .. arg[7:0],
    /// (crc7 << 1) | 1]`.
    pub const fn frame(&self) -> [u8; 6] {
        let a = self.arg.to_be_bytes();
        let head = [0x40 | (self.index & 0x3F), a[0], a[1], a[2], a[3]];
        [
            head[0],
            head[1],
            head[2],
            head[3],
            head[4],
            (crc7(&head) << 1) | 1,
        ]
    }

    /// CMD0 GO_IDLE_STATE: resets all cards to idle. No response.
    pub const fn go_idle_state() -> Self {
        Self::new(0, 0, ResponseKind::None)
    }

    /// CMD2 ALL_SEND_CID: asks every card for its CID.
    pub const fn all_send_cid() -> Self {
        Self::new(2, 0, ResponseKind::R2)
    }

    /// CMD3 SEND_RELATIVE_ADDR: asks the card to publish a new RCA.
    pub const fn send_relative_addr() -> Self {
        Self::new(3, 0, ResponseKind::R6)
    }

    /// CMD7 SELECT/DESELECT_CARD: selects the card with address `rca`; any other address
    /// deselects it. The R1b response comes only from the card being selected.
    pub const fn select_card(rca: u16) -> Self {
        Self::new(7, (rca as u32) << 16, ResponseKind::R1b)
    }

    /// CMD8 SEND_IF_COND: announces the 2.7-3.6 V host window (VHS = 0001b, argument bits
    /// 11:8) and an echo `check_pattern` (bits 7:0).
    pub const fn send_if_cond(check_pattern: u8) -> Self {
        Self::new(8, 0x100 | check_pattern as u32, ResponseKind::R7)
    }

    /// CMD9 SEND_CSD: the addressed card sends its CSD.
    pub const fn send_csd(rca: u16) -> Self {
        Self::new(9, (rca as u32) << 16, ResponseKind::R2)
    }

    /// CMD10 SEND_CID: the addressed card sends its CID.
    pub const fn send_cid(rca: u16) -> Self {
        Self::new(10, (rca as u32) << 16, ResponseKind::R2)
    }

    /// CMD12 STOP_TRANSMISSION: ends a multiple-block transfer.
    pub const fn stop_transmission() -> Self {
        Self::new(12, 0, ResponseKind::R1b)
    }

    /// CMD13 SEND_STATUS: the addressed card sends its card status.
    pub const fn send_status(rca: u16) -> Self {
        Self::new(13, (rca as u32) << 16, ResponseKind::R1)
    }

    /// CMD16 SET_BLOCKLEN: sets the block length in bytes for SDSC cards.
    pub const fn set_blocklen(bytes: u32) -> Self {
        Self::new(16, bytes, ResponseKind::R1)
    }

    /// CMD17 READ_SINGLE_BLOCK at `addr` (block number on SDHC/SDXC, byte address on SDSC).
    pub const fn read_single_block(addr: u32) -> Self {
        Self::new(17, addr, ResponseKind::R1)
    }

    /// CMD18 READ_MULTIPLE_BLOCK from `addr`, until CMD12 or the block count set by CMD23.
    pub const fn read_multiple_block(addr: u32) -> Self {
        Self::new(18, addr, ResponseKind::R1)
    }

    /// CMD23 SET_BLOCK_COUNT: block count (argument bits 15:0) for the next CMD18 or CMD25.
    pub const fn set_block_count(count: u16) -> Self {
        Self::new(23, count as u32, ResponseKind::R1)
    }

    /// CMD24 WRITE_BLOCK at `addr`.
    pub const fn write_block(addr: u32) -> Self {
        Self::new(24, addr, ResponseKind::R1)
    }

    /// CMD25 WRITE_MULTIPLE_BLOCK from `addr`.
    pub const fn write_multiple_block(addr: u32) -> Self {
        Self::new(25, addr, ResponseKind::R1)
    }

    /// CMD32 ERASE_WR_BLK_START: first block to erase.
    pub const fn erase_wr_blk_start(addr: u32) -> Self {
        Self::new(32, addr, ResponseKind::R1)
    }

    /// CMD33 ERASE_WR_BLK_END: last block to erase.
    pub const fn erase_wr_blk_end(addr: u32) -> Self {
        Self::new(33, addr, ResponseKind::R1)
    }

    /// CMD38 ERASE: erases the range set by CMD32 and CMD33 (argument 0, normal erase).
    pub const fn erase() -> Self {
        Self::new(38, 0, ResponseKind::R1b)
    }

    /// CMD55 APP_CMD: the next command is an application command (ACMD).
    pub const fn app_cmd(rca: u16) -> Self {
        Self::new(55, (rca as u32) << 16, ResponseKind::R1)
    }

    /// ACMD6 SET_BUS_WIDTH: four-bit mode if `four_bit`, else one-bit (argument bits 1:0 =
    /// 10b / 00b).
    pub const fn set_bus_width(four_bit: bool) -> Self {
        Self::new(6, if four_bit { 2 } else { 0 }, ResponseKind::R1)
    }

    /// ACMD13 SD_STATUS: the card sends its 512-bit SD Status on DAT.
    pub const fn sd_status() -> Self {
        Self::new(13, 0, ResponseKind::R1)
    }

    /// ACMD22 SEND_NUM_WR_BLOCKS: the card sends the count of well-written blocks on DAT.
    pub const fn send_num_wr_blocks() -> Self {
        Self::new(22, 0, ResponseKind::R1)
    }

    /// ACMD23 SET_WR_BLK_ERASE_COUNT: blocks to pre-erase before the next write (argument bits
    /// 22:0).
    pub const fn set_wr_blk_erase_count(count: u32) -> Self {
        Self::new(23, count & 0x007F_FFFF, ResponseKind::R1)
    }

    /// ACMD41 SD_SEND_OP_COND: HCS in bit 30, the host's voltage window `ocr_window` in bits
    /// 23:0 (use the OCR window bits, for example `0x00FF_8000` for 2.7-3.6 V). Other bits of
    /// `ocr_window` are ignored. An argument of 0 inquires the card's OCR.
    pub const fn sd_send_op_cond(hcs: bool, ocr_window: u32) -> Self {
        let hcs = if hcs { 1 << 30 } else { 0 };
        Self::new(41, hcs | (ocr_window & 0x00FF_FFFF), ResponseKind::R3)
    }

    /// ACMD42 SET_CLR_CARD_DETECT: connects (`true`) or disconnects the 50 kOhm pull-up on
    /// DAT3 (argument bit 0).
    pub const fn set_clr_card_detect(connect: bool) -> Self {
        Self::new(42, connect as u32, ResponseKind::R1)
    }

    /// ACMD51 SEND_SCR: the card sends its SCR on DAT.
    pub const fn send_scr() -> Self {
        Self::new(51, 0, ResponseKind::R1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_frames() {
        assert_eq!(Command::go_idle_state().frame(), [0x40, 0, 0, 0, 0, 0x95]);
        assert_eq!(
            Command::send_if_cond(0xAA).frame(),
            [0x48, 0, 0, 0x01, 0xAA, 0x87]
        );
        assert_eq!(Command::app_cmd(0).frame(), [0x77, 0, 0, 0, 0, 0x65]);
        assert_eq!(
            Command::read_single_block(0).frame(),
            [0x51, 0, 0, 0, 0, 0x55]
        );
    }

    #[test]
    fn other_published_frames() {
        // ACMD41 with HCS only (the widely published `69 40 00 00 00 77`), and with the window.
        assert_eq!(
            Command::sd_send_op_cond(true, 0).frame(),
            [0x69, 0x40, 0, 0, 0, 0x77]
        );
        assert_eq!(
            Command::sd_send_op_cond(true, 0x00FF_8000).frame(),
            [0x69, 0x40, 0xFF, 0x80, 0x00, 0x17]
        );
        // CMD12 with argument 0.
        assert_eq!(
            Command::stop_transmission().frame(),
            [0x4C, 0, 0, 0, 0, 0x61]
        );
    }

    #[test]
    fn frame_layout_and_crc() {
        let c = Command::read_multiple_block(0x1234_5678);
        let f = c.frame();
        assert_eq!(f[0], 0x40 | 18);
        assert_eq!(f[1..5], [0x12, 0x34, 0x56, 0x78]);
        assert_eq!(f[5] & 1, 1);
        assert_eq!(f[5] >> 1, crc7(&f[..5]));
    }

    #[test]
    fn rca_goes_in_upper_half() {
        assert_eq!(Command::select_card(0xABCD).arg, 0xABCD_0000);
        assert_eq!(Command::send_csd(0xABCD).arg, 0xABCD_0000);
        assert_eq!(Command::send_cid(0xABCD).arg, 0xABCD_0000);
        assert_eq!(Command::send_status(0xABCD).arg, 0xABCD_0000);
        assert_eq!(Command::app_cmd(0xABCD).arg, 0xABCD_0000);
    }

    #[test]
    fn indices_and_responses() {
        use ResponseKind::*;
        let table = [
            (Command::go_idle_state(), 0, None),
            (Command::all_send_cid(), 2, R2),
            (Command::send_relative_addr(), 3, R6),
            (Command::select_card(1), 7, R1b),
            (Command::send_if_cond(0xAA), 8, R7),
            (Command::send_csd(1), 9, R2),
            (Command::send_cid(1), 10, R2),
            (Command::stop_transmission(), 12, R1b),
            (Command::send_status(1), 13, R1),
            (Command::set_blocklen(512), 16, R1),
            (Command::read_single_block(0), 17, R1),
            (Command::read_multiple_block(0), 18, R1),
            (Command::set_block_count(8), 23, R1),
            (Command::write_block(0), 24, R1),
            (Command::write_multiple_block(0), 25, R1),
            (Command::erase_wr_blk_start(0), 32, R1),
            (Command::erase_wr_blk_end(0), 33, R1),
            (Command::erase(), 38, R1b),
            (Command::app_cmd(0), 55, R1),
            (Command::set_bus_width(true), 6, R1),
            (Command::sd_status(), 13, R1),
            (Command::send_num_wr_blocks(), 22, R1),
            (Command::set_wr_blk_erase_count(1), 23, R1),
            (Command::sd_send_op_cond(true, 0), 41, R3),
            (Command::set_clr_card_detect(false), 42, R1),
            (Command::send_scr(), 51, R1),
        ];
        for (cmd, index, response) in table {
            assert_eq!(cmd.index, index);
            assert_eq!(cmd.response, response, "CMD{index}");
        }
    }

    #[test]
    fn arguments() {
        assert_eq!(Command::send_if_cond(0x5A).arg, 0x15A);
        assert_eq!(Command::set_bus_width(true).arg, 2);
        assert_eq!(Command::set_bus_width(false).arg, 0);
        assert_eq!(Command::set_block_count(0xFFFF).arg, 0xFFFF);
        assert_eq!(Command::sd_send_op_cond(true, 0x00FF_8000).arg, 0x40FF_8000);
        assert_eq!(
            Command::sd_send_op_cond(false, 0xFFFF_FFFF).arg,
            0x00FF_FFFF
        );
        assert_eq!(
            Command::set_wr_blk_erase_count(0xFFFF_FFFF).arg,
            0x007F_FFFF
        );
        assert_eq!(Command::set_clr_card_detect(true).arg, 1);
    }
}
