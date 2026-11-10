//! Card registers (Physical Layer Simplified Specification, section 5). Register bytes are in
//! wire order: byte 0 of a 128-bit register holds bits `[127:120]`, byte 0 of the 64-bit SCR
//! holds `[63:56]`.

/// The operating conditions register (section 5.1), as returned by R3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ocr(pub u32);

impl Ocr {
    /// Bit 31 (card power up status): set when the card has finished power-up.
    pub const POWER_UP_DONE: u32 = 1 << 31;
    /// Bit 30 (CCS): set for a high or extended capacity card (block addressing). Valid only
    /// when `POWER_UP_DONE` is set.
    pub const CCS: u32 = 1 << 30;
    /// Bit 24 (S18A): the card switched to 1.8 V signalling.
    pub const S18A: u32 = 1 << 24;
    /// Bit 20: 3.2-3.3 V supported.
    pub const V3_2_3_3: u32 = 1 << 20;
    /// Bit 21: 3.3-3.4 V supported.
    pub const V3_3_3_4: u32 = 1 << 21;
    /// Bits 23:15: the whole 2.7-3.6 V window.
    pub const WINDOW_2_7_3_6: u32 = 0x00FF_8000;

    /// Power-up routine finished (bit 31).
    pub const fn power_up_done(&self) -> bool {
        self.0 & Self::POWER_UP_DONE != 0
    }

    /// Card capacity status (bit 30): block addressing. Meaningful only once
    /// [`power_up_done`](Self::power_up_done).
    pub const fn ccs(&self) -> bool {
        self.0 & Self::CCS != 0
    }

    /// Switch to 1.8 V accepted (bit 24).
    pub const fn s18a(&self) -> bool {
        self.0 & Self::S18A != 0
    }

    /// The card supports 3.2-3.3 V (bit 20).
    pub const fn v3_2_3_3(&self) -> bool {
        self.0 & Self::V3_2_3_3 != 0
    }

    /// The card supports 3.3-3.4 V (bit 21).
    pub const fn v3_3_3_4(&self) -> bool {
        self.0 & Self::V3_3_3_4 != 0
    }
}

/// A register could not be interpreted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegisterError {
    /// CSD_STRUCTURE (bits 127:126) holds the reserved value 3.
    UnknownCsdStructure(u8),
}

/// The card identification register (section 5.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cid {
    /// MID, bits 127:120: manufacturer ID.
    pub manufacturer_id: u8,
    /// OID, bits 119:104: OEM/application ID, two ASCII characters.
    pub oem_id: [u8; 2],
    /// PNM, bits 103:64: product name, five ASCII characters.
    pub product_name: [u8; 5],
    /// PRV, bits 63:56: product revision, major in the high BCD digit, minor in the low.
    pub revision: u8,
    /// PSN, bits 55:24: product serial number.
    pub serial: u32,
    /// MDT year, bits 19:12: years since 2000.
    pub year_offset: u8,
    /// MDT month, bits 11:8: 1..=12.
    pub month: u8,
}

impl Cid {
    /// Parses the register from its 16 bytes.
    pub const fn parse(raw: &[u8; 16]) -> Self {
        Self {
            manufacturer_id: raw[0],
            oem_id: [raw[1], raw[2]],
            product_name: [raw[3], raw[4], raw[5], raw[6], raw[7]],
            revision: raw[8],
            serial: u32::from_be_bytes([raw[9], raw[10], raw[11], raw[12]]),
            year_offset: ((raw[13] & 0x0F) << 4) | (raw[14] >> 4),
            month: raw[14] & 0x0F,
        }
    }

    /// The manufacturing year (2000 + MDT year field).
    pub const fn year(&self) -> u16 {
        2000 + self.year_offset as u16
    }

    /// The revision as `(major, minor)` BCD digits.
    pub const fn revision_parts(&self) -> (u8, u8) {
        (self.revision >> 4, self.revision & 0xF)
    }
}

/// The CSD structure version (CSD_STRUCTURE, bits 127:126).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CsdVersion {
    /// 0: version 1.0, standard capacity (SDSC).
    V1,
    /// 1: version 2.0, high and extended capacity (SDHC, SDXC).
    V2,
    /// 2: version 3.0, ultra capacity (SDUC).
    V3,
}

/// The card-specific data register (section 5.3), reduced to the fields the driver uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Csd {
    /// CSD_STRUCTURE.
    pub version: CsdVersion,
    /// Capacity in 512-byte blocks, from C_SIZE (and for v1 C_SIZE_MULT and READ_BL_LEN).
    pub capacity_blocks: u64,
    /// CCC, bits 95:84: supported command classes, bit `n` for class `n`.
    pub ccc: u16,
    /// TRAN_SPEED, bits 103:96: maximum data transfer rate, raw.
    pub tran_speed: u8,
}

impl Csd {
    /// Parses the register from its 16 bytes.
    pub const fn parse(raw: &[u8; 16]) -> Result<Self, RegisterError> {
        let r = u128::from_be_bytes(*raw);
        let ccc = ((r >> 84) & 0xFFF) as u16;
        let tran_speed = ((r >> 96) & 0xFF) as u8;
        let (version, capacity_blocks) = match (r >> 126) as u8 {
            0 => {
                let read_bl_len = ((r >> 80) & 0xF) as u32;
                let c_size = ((r >> 62) & 0xFFF) as u64;
                let mult = ((r >> 47) & 0x7) as u32;
                (
                    CsdVersion::V1,
                    ((c_size + 1) << (mult + 2 + read_bl_len)) >> 9,
                )
            }
            1 => (CsdVersion::V2, (((r >> 48) & 0x3F_FFFF) as u64 + 1) * 1024),
            2 => (
                CsdVersion::V3,
                (((r >> 48) & 0x0FFF_FFFF) as u64 + 1) * 1024,
            ),
            n => return Err(RegisterError::UnknownCsdStructure(n)),
        };
        Ok(Self {
            version,
            capacity_blocks,
            ccc,
            tran_speed,
        })
    }
}

/// The SD configuration register (section 5.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Scr {
    /// SCR_STRUCTURE, bits 63:60.
    pub structure: u8,
    /// SD_SPEC, bits 59:56.
    pub sd_spec: u8,
    /// SD_SPEC3, bit 47.
    pub sd_spec3: bool,
    /// SD_SPEC4, bit 42.
    pub sd_spec4: bool,
    /// SD_SPECX, bits 41:38.
    pub sd_specx: u8,
    /// SD_BUS_WIDTHS, bits 51:48: bit 0 one-bit (DAT0), bit 2 four-bit (DAT0-3).
    pub sd_bus_widths: u8,
    /// CMD_SUPPORT, bits 35:32: bit 0 CMD20, bit 1 CMD23.
    pub cmd_support: u8,
}

impl Scr {
    /// Parses the register from its 8 bytes.
    pub const fn parse(raw: &[u8; 8]) -> Self {
        let r = u64::from_be_bytes(*raw);
        Self {
            structure: ((r >> 60) & 0xF) as u8,
            sd_spec: ((r >> 56) & 0xF) as u8,
            sd_spec3: (r >> 47) & 1 != 0,
            sd_spec4: (r >> 42) & 1 != 0,
            sd_specx: ((r >> 38) & 0xF) as u8,
            sd_bus_widths: ((r >> 48) & 0xF) as u8,
            cmd_support: ((r >> 32) & 0xF) as u8,
        }
    }

    /// The card supports one-bit bus width.
    pub const fn supports_1bit(&self) -> bool {
        self.sd_bus_widths & 0b0001 != 0
    }

    /// The card supports four-bit bus width.
    pub const fn supports_4bit(&self) -> bool {
        self.sd_bus_widths & 0b0100 != 0
    }

    /// The card supports CMD20 (speed class control).
    pub const fn supports_cmd20(&self) -> bool {
        self.cmd_support & 0b0001 != 0
    }

    /// The card supports CMD23 (set block count).
    pub const fn supports_cmd23(&self) -> bool {
        self.cmd_support & 0b0010 != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Writes `value` into bits `[hi:lo]` of a 128-bit register image.
    fn put(reg: &mut u128, hi: u32, lo: u32, value: u128) {
        let width = hi - lo + 1;
        let mask = ((1u128 << width) - 1) << lo;
        assert!(value < (1u128 << width));
        *reg = (*reg & !mask) | (value << lo);
    }

    #[test]
    fn ocr_bits() {
        let ocr = Ocr(0xC0FF_8000);
        assert!(ocr.power_up_done());
        assert!(ocr.ccs());
        assert!(!ocr.s18a());
        assert!(ocr.v3_2_3_3());
        assert!(ocr.v3_3_3_4());
        let ocr = Ocr(0x80FF_8000);
        assert!(ocr.power_up_done() && !ocr.ccs());
        assert!(Ocr(1 << 24).s18a());
        assert!(!Ocr(0).power_up_done());
        assert!(!Ocr(!(1 << 20)).v3_2_3_3());
        assert!(!Ocr(!(1 << 21)).v3_3_3_4());
        assert_eq!(Ocr::WINDOW_2_7_3_6, 0x00FF_8000);
    }

    #[test]
    fn cid_fields() {
        let mut raw = [0u8; 16];
        raw[0] = 0x03;
        raw[1..3].copy_from_slice(b"SD");
        raw[3..8].copy_from_slice(b"SR64G");
        raw[8] = 0x85;
        raw[9..13].copy_from_slice(&0xDEAD_BEEFu32.to_be_bytes());
        // Reserved nibble 0xA, then year 0x15 (2021), month 7.
        raw[13] = 0xA1;
        raw[14] = 0x57;
        raw[15] = 0xFF;
        let cid = Cid::parse(&raw);
        assert_eq!(cid.manufacturer_id, 0x03);
        assert_eq!(&cid.oem_id, b"SD");
        assert_eq!(&cid.product_name, b"SR64G");
        assert_eq!(cid.revision, 0x85);
        assert_eq!(cid.revision_parts(), (8, 5));
        assert_eq!(cid.serial, 0xDEAD_BEEF);
        assert_eq!(cid.year_offset, 0x15);
        assert_eq!(cid.year(), 2021);
        assert_eq!(cid.month, 7);
    }

    #[test]
    fn csd_v2_bench_card() {
        let mut r = 0u128;
        put(&mut r, 127, 126, 1);
        put(&mut r, 103, 96, 0x32);
        put(&mut r, 95, 84, 0x5B5);
        put(&mut r, 83, 80, 9);
        put(&mut r, 69, 48, 121_811);
        put(&mut r, 0, 0, 1);
        let csd = Csd::parse(&r.to_be_bytes()).unwrap();
        assert_eq!(csd.version, CsdVersion::V2);
        assert_eq!(csd.capacity_blocks, 124_735_488);
        assert_eq!(csd.ccc, 0x5B5);
        assert_eq!(csd.tran_speed, 0x32);
    }

    #[test]
    fn csd_v2_c_size_is_22_bits() {
        let mut r = 0u128;
        put(&mut r, 127, 126, 1);
        put(&mut r, 69, 48, 0x3F_FFFF);
        put(&mut r, 75, 70, 0x3F); // bits above C_SIZE are ignored in v2
        assert_eq!(
            Csd::parse(&r.to_be_bytes()).unwrap().capacity_blocks,
            0x40_0000 * 1024
        );
    }

    #[test]
    fn csd_v1_example() {
        // 1 GB-class SDSC: READ_BL_LEN 10 (1024 B), C_SIZE 4095, C_SIZE_MULT 7:
        // 4096 * 2^9 * 1024 B = 2 GiB = 4_194_304 blocks.
        let mut r = 0u128;
        put(&mut r, 127, 126, 0);
        put(&mut r, 103, 96, 0x32);
        put(&mut r, 95, 84, 0x5B5);
        put(&mut r, 83, 80, 10);
        put(&mut r, 73, 62, 4095);
        put(&mut r, 49, 47, 7);
        let csd = Csd::parse(&r.to_be_bytes()).unwrap();
        assert_eq!(csd.version, CsdVersion::V1);
        assert_eq!(csd.capacity_blocks, 4_194_304);
        assert_eq!(csd.ccc, 0x5B5);

        // 512 B blocks, C_SIZE 1000, C_SIZE_MULT 3: 1001 * 32 * 512 / 512 = 32032 blocks.
        let mut r = 0u128;
        put(&mut r, 83, 80, 9);
        put(&mut r, 73, 62, 1000);
        put(&mut r, 49, 47, 3);
        assert_eq!(
            Csd::parse(&r.to_be_bytes()).unwrap().capacity_blocks,
            32_032
        );
    }

    #[test]
    fn csd_v3_c_size_is_28_bits() {
        let mut r = 0u128;
        put(&mut r, 127, 126, 2);
        put(&mut r, 75, 48, 0x0FFF_FFFF);
        let csd = Csd::parse(&r.to_be_bytes()).unwrap();
        assert_eq!(csd.version, CsdVersion::V3);
        assert_eq!(csd.capacity_blocks, 0x1000_0000u64 * 1024);
    }

    #[test]
    fn csd_reserved_structure() {
        let mut r = 0u128;
        put(&mut r, 127, 126, 3);
        assert_eq!(
            Csd::parse(&r.to_be_bytes()),
            Err(RegisterError::UnknownCsdStructure(3))
        );
    }

    #[test]
    fn csd_parse_never_panics_on_extremes() {
        for byte in [0x00u8, 0x55, 0xAA, 0xFF] {
            let _ = Csd::parse(&[byte; 16]);
        }
    }

    #[test]
    fn scr_fields() {
        let mut r = 0u64;
        r |= 0 << 60; // SCR_STRUCTURE 0
        r |= 2 << 56; // SD_SPEC 2
        r |= 1 << 55; // DATA_STAT_AFTER_ERASE
        r |= 0b101 << 52; // SD_SECURITY
        r |= 0b0101 << 48; // 1-bit and 4-bit
        r |= 1 << 47; // SD_SPEC3
        r |= 1 << 42; // SD_SPEC4
        r |= 0b0101 << 38; // SD_SPECX
        r |= 0b0011 << 32; // CMD20, CMD23
        let scr = Scr::parse(&r.to_be_bytes());
        assert_eq!(scr.structure, 0);
        assert_eq!(scr.sd_spec, 2);
        assert!(scr.sd_spec3);
        assert!(scr.sd_spec4);
        assert_eq!(scr.sd_specx, 0b0101);
        assert_eq!(scr.sd_bus_widths, 0b0101);
        assert!(scr.supports_1bit() && scr.supports_4bit());
        assert!(scr.supports_cmd20() && scr.supports_cmd23());
    }

    #[test]
    fn scr_fields_are_independent() {
        let scr = Scr::parse(&[0x01, 0x25, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(scr.structure, 0);
        assert_eq!(scr.sd_spec, 1);
        assert_eq!(scr.sd_bus_widths, 0b0101);
        assert!(scr.sd_spec3);
        assert!(!scr.sd_spec4);
        assert_eq!(scr.sd_specx, 0);
        assert_eq!(scr.cmd_support, 0);
        assert!(!scr.supports_cmd23());

        let scr = Scr::parse(&[0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(scr.structure, 0);
        assert_eq!(scr.sd_spec, 2);
        let scr = Scr::parse(&[0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(scr.cmd_support, 2);
        assert!(scr.supports_cmd23() && !scr.supports_cmd20());
    }
}
