//! Scripted fault injection.

/// Card status bits, for [`Fault::StatusError`].
pub mod status {
    /// OUT_OF_RANGE.
    pub const OUT_OF_RANGE: u32 = 1 << 31;
    /// ADDRESS_ERROR.
    pub const ADDRESS_ERROR: u32 = 1 << 30;
    /// BLOCK_LEN_ERROR.
    pub const BLOCK_LEN_ERROR: u32 = 1 << 29;
    /// ERASE_SEQ_ERROR.
    pub const ERASE_SEQ_ERROR: u32 = 1 << 28;
    /// ERASE_PARAM.
    pub const ERASE_PARAM: u32 = 1 << 27;
    /// WP_VIOLATION.
    pub const WP_VIOLATION: u32 = 1 << 26;
    /// LOCK_UNLOCK_FAILED.
    pub const LOCK_UNLOCK_FAILED: u32 = 1 << 24;
    /// COM_CRC_ERROR.
    pub const COM_CRC_ERROR: u32 = 1 << 23;
    /// ILLEGAL_COMMAND.
    pub const ILLEGAL_COMMAND: u32 = 1 << 22;
    /// CARD_ECC_FAILED.
    pub const CARD_ECC_FAILED: u32 = 1 << 21;
    /// CC_ERROR.
    pub const CC_ERROR: u32 = 1 << 20;
    /// ERROR.
    pub const ERROR: u32 = 1 << 19;
    /// CSD_OVERWRITE.
    pub const CSD_OVERWRITE: u32 = 1 << 16;
    /// WP_ERASE_SKIP.
    pub const WP_ERASE_SKIP: u32 = 1 << 15;
    /// ERASE_RESET.
    pub const ERASE_RESET: u32 = 1 << 13;
    /// AKE_SEQ_ERROR.
    pub const AKE_SEQ_ERROR: u32 = 1 << 3;
    /// All error bits that clear when a response carries them.
    pub const ERROR_MASK: u32 = OUT_OF_RANGE
        | ADDRESS_ERROR
        | BLOCK_LEN_ERROR
        | ERASE_SEQ_ERROR
        | ERASE_PARAM
        | WP_VIOLATION
        | LOCK_UNLOCK_FAILED
        | COM_CRC_ERROR
        | ILLEGAL_COMMAND
        | CARD_ECC_FAILED
        | CC_ERROR
        | ERROR
        | CSD_OVERWRITE
        | WP_ERASE_SKIP
        | ERASE_RESET
        | AKE_SEQ_ERROR;
}

/// What goes wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// The command gets no response. Needs a command trigger.
    NoResponse,
    /// One bit of the response frame is flipped. `bit` counts in wire order, bit 0 being the
    /// start bit; it wraps at the frame length. Needs a command trigger.
    FlipResponseBit {
        /// Bit position in wire order.
        bit: u32,
    },
    /// One data bit of a block read is flipped, the CRC staying that of the true data. `bit`
    /// numbers the block's bits from the least significant bit of byte 0 up, and wraps at the
    /// block length. Needs a read trigger.
    FlipDataBit {
        /// Bit position.
        bit: u32,
    },
    /// One bit of the 64-bit CRC of a block read is flipped (`bit` 0 is the least significant,
    /// and wraps at 64). Needs a read trigger.
    FlipCrcBit {
        /// Bit position.
        bit: u32,
    },
    /// The block write is answered with the CRC-error token 0b101 and not stored, whatever CRC
    /// the host sent. Needs a write trigger.
    DataCrcErrorToken,
    /// The block write is answered with the write-error token 0b110 and not stored, and ERROR is
    /// raised. Needs a write trigger.
    WriteErrorToken,
    /// The card is busy for this long after the written block, instead of the configured time.
    /// Needs a write trigger.
    BusyAfterWrite {
        /// Busy time in microseconds.
        us: u64,
    },
    /// The card is busy for this long after the command's response, in addition to any busy the
    /// command has by itself. Intended for CMD12. Needs a command trigger.
    BusyAfterCommand {
        /// Busy time in microseconds.
        us: u64,
    },
    /// These card-status bits (see [`status`]) are raised by the operation. For a command they
    /// show in its own response; for a block transfer, in the next response.
    StatusError {
        /// Bits to set.
        mask: u32,
    },
    /// The card is removed when the operation arrives, which then times out like every later
    /// one, until [`SimCard::reinsert`](super::SimCard::reinsert).
    RemoveCard,
    /// ACMD41 reports busy. Needs a command trigger.
    Acmd41NeverReady,
}

/// What a trigger counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Match {
    /// Commands with this index on the wire (an application command counts under its own index).
    Command(u8),
    /// Every command.
    AnyCommand,
    /// Every block read that the card serves.
    BlockRead,
    /// Every block write that the card serves.
    BlockWrite,
    /// Block reads of this block address. SCR, SD status and write-count reads have none.
    ReadLba(u32),
    /// Block writes to this block address.
    WriteLba(u32),
    /// Block reads or writes at this block address.
    Lba(u32),
}

/// When a fault fires: on the `nth` matching operation counted from the moment of injection, and
/// on the `times - 1` matching operations after it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Trigger {
    /// What to count.
    pub on: Match,
    /// Which match fires first, counting from 1.
    pub nth: u32,
    /// How many consecutive matches fire.
    pub times: u32,
}

impl Trigger {
    /// Fires once, on the `nth` match of `on`.
    pub fn new(on: Match, nth: u32) -> Self {
        assert!(nth >= 1, "nth counts from 1");
        Trigger { on, nth, times: 1 }
    }

    /// The `nth` command with this index.
    pub fn command(index: u8, nth: u32) -> Self {
        Self::new(Match::Command(index), nth)
    }

    /// The `nth` block read.
    pub fn block_read(nth: u32) -> Self {
        Self::new(Match::BlockRead, nth)
    }

    /// The `nth` block write.
    pub fn block_write(nth: u32) -> Self {
        Self::new(Match::BlockWrite, nth)
    }

    /// The first read or write at this block address.
    pub fn lba(lba: u32) -> Self {
        Self::new(Match::Lba(lba), 1)
    }

    /// Fires on `times` consecutive matches instead of one.
    pub fn repeat(mut self, times: u32) -> Self {
        assert!(times >= 1, "times counts from 1");
        self.times = times;
        self
    }

    /// Fires on every match from the first one on.
    pub fn forever(self) -> Self {
        self.repeat(u32::MAX)
    }
}

/// A bus-level event that triggers are matched against.
#[derive(Debug, Clone, Copy)]
pub(super) enum Event {
    Command(u8),
    Read(Option<u32>),
    Write(u32),
}

impl Match {
    fn matches(self, ev: Event) -> bool {
        match (self, ev) {
            (Match::Command(i), Event::Command(j)) => i == j,
            (Match::AnyCommand, Event::Command(_)) => true,
            (Match::BlockRead, Event::Read(_)) => true,
            (Match::BlockWrite, Event::Write(_)) => true,
            (Match::ReadLba(l), Event::Read(Some(m))) => l == m,
            (Match::WriteLba(l), Event::Write(m)) => l == m,
            (Match::Lba(l), Event::Read(Some(m)) | Event::Write(m)) => l == m,
            _ => false,
        }
    }

    fn is_command(self) -> bool {
        matches!(self, Match::Command(_) | Match::AnyCommand)
    }

    fn can_read(self) -> bool {
        matches!(self, Match::BlockRead | Match::ReadLba(_) | Match::Lba(_))
    }

    fn can_write(self) -> bool {
        matches!(self, Match::BlockWrite | Match::WriteLba(_) | Match::Lba(_))
    }
}

/// An injected fault with its trigger and its count of matches so far.
#[derive(Debug, Clone)]
pub(super) struct Armed {
    pub(super) fault: Fault,
    pub(super) trigger: Trigger,
    pub(super) seen: u32,
}

impl Armed {
    pub(super) fn new(fault: Fault, trigger: Trigger) -> Self {
        let on = trigger.on;
        let ok = match fault {
            Fault::NoResponse
            | Fault::FlipResponseBit { .. }
            | Fault::BusyAfterCommand { .. }
            | Fault::Acmd41NeverReady => on.is_command(),
            Fault::FlipDataBit { .. } | Fault::FlipCrcBit { .. } => on.can_read(),
            Fault::DataCrcErrorToken | Fault::WriteErrorToken | Fault::BusyAfterWrite { .. } => {
                on.can_write()
            }
            Fault::StatusError { .. } | Fault::RemoveCard => true,
        };
        assert!(ok, "fault {fault:?} cannot be triggered by {on:?}");
        Armed {
            fault,
            trigger,
            seen: 0,
        }
    }

    /// Counts the event if it matches; true if the fault fires on it.
    pub(super) fn fires(&mut self, ev: Event) -> bool {
        if !self.trigger.on.matches(ev) {
            return false;
        }
        self.seen = self.seen.saturating_add(1);
        self.seen >= self.trigger.nth && self.seen - self.trigger.nth < self.trigger.times
    }
}
