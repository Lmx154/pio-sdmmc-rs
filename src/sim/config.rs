//! Configuration of the simulated card.

/// Which kind of card is simulated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CardKind {
    /// Standard capacity, spec version 1: does not answer CMD8. Byte addressing, CSD version 1.
    SdscV1,
    /// Standard capacity, spec version 2 or later: answers CMD8. Byte addressing, CSD version 1.
    SdscV2,
    /// High capacity: block addressing, CSD version 2.
    Sdhc,
    /// Extended capacity: block addressing, CSD version 2.
    Sdxc,
}

impl CardKind {
    /// True for SDHC and SDXC, which are addressed in blocks.
    pub fn is_high_capacity(self) -> bool {
        matches!(self, CardKind::Sdhc | CardKind::Sdxc)
    }

    /// True if the card answers CMD8.
    pub fn answers_cmd8(self) -> bool {
        !matches!(self, CardKind::SdscV1)
    }
}

/// The fields of the CID register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cid {
    /// Manufacturer ID.
    pub manufacturer: u8,
    /// OEM/application ID, two ASCII characters.
    pub oem: [u8; 2],
    /// Product name, five ASCII characters.
    pub product: [u8; 5],
    /// Product revision, as the 8-bit BCD field.
    pub revision: u8,
    /// Product serial number.
    pub serial: u32,
    /// Manufacturing date: bits 11..4 year since 2000, bits 3..0 month.
    pub date: u16,
}

/// Everything that sets the simulated card's identity, speed and busy behaviour.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimConfig {
    /// Card kind.
    pub kind: CardKind,
    /// Capacity in 512-byte blocks.
    pub capacity_blocks: u32,
    /// CID fields.
    pub cid: Cid,
    /// The relative card address CMD3 publishes. Must not be 0.
    pub rca: u16,
    /// The OCR the card reports when ready. The busy bit is added and the CCS bit is decided by
    /// the card kind and the host's HCS bit, whatever this holds.
    pub ocr: u32,
    /// Number of ACMD41 polls that report busy before one may report ready.
    pub acmd41_polls: u32,
    /// Virtual milliseconds from the first ACMD41 that starts initialization until the card may
    /// report ready.
    pub acmd41_ms: u32,
    /// Whether the SCR advertises CMD23.
    pub supports_cmd23: bool,
    /// Busy time after each written block, in microseconds.
    pub write_busy_us: u64,
    /// Every this many blocks written (counted over the card's life), the busy time is
    /// `long_pause_us` instead. 0 disables it.
    pub long_pause_every_blocks: u32,
    /// The longer busy time, in microseconds.
    pub long_pause_us: u64,
    /// Busy time after CMD12, in microseconds.
    pub cmd12_busy_us: u64,
    /// Busy time after CMD7 select, in microseconds.
    pub cmd7_busy_us: u64,
    /// Busy time after CMD38, in microseconds.
    pub erase_busy_us: u64,
    /// The clock the bus divides down: the card clock is this divided by an integer.
    pub base_clock_hz: u32,
}

impl Default for SimConfig {
    /// The 64 GB SDXC card on the bench.
    fn default() -> Self {
        SimConfig {
            kind: CardKind::Sdxc,
            capacity_blocks: 124_735_488,
            cid: Cid {
                manufacturer: 0x03,
                oem: *b"SD",
                product: *b"SK64G",
                revision: 0x80,
                serial: 0xB183_C2B8,
                date: 0x183,
            },
            rca: 0xAAAA,
            ocr: 0xC0FF_8000,
            acmd41_polls: 2,
            acmd41_ms: 20,
            supports_cmd23: true,
            write_busy_us: 400,
            long_pause_every_blocks: 64,
            long_pause_us: 15_000,
            cmd12_busy_us: 250,
            cmd7_busy_us: 50,
            erase_busy_us: 1_000,
            base_clock_hz: 150_000_000,
        }
    }
}
