//! What the simulator records: the bus-call trace and the rule violations.

use crate::bus::BusError;

/// The card's state in the SD state diagram, with the value it has in the CURRENT_STATE field of
/// the card status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CardState {
    /// Idle (0).
    Idle = 0,
    /// Ready (1).
    Ready = 1,
    /// Identification (2).
    Ident = 2,
    /// Stand-by (3).
    Stby = 3,
    /// Transfer (4).
    Tran = 4,
    /// Sending data (5).
    Data = 5,
    /// Receiving data (6).
    Rcv = 6,
    /// Programming (7).
    Prg = 7,
    /// Disconnect (8).
    Dis = 8,
}

/// How much response a host buffer takes or a command produces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResponseClass {
    /// No response.
    None,
    /// 48 bits.
    Short,
    /// 136 bits.
    Long,
}

/// A rule the host broke. The call that broke it is not failed; the entry is just recorded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Violation {
    /// The first command after power-up came before 74 idle clock cycles.
    CommandBeforePowerUpClocks {
        /// Idle cycles seen since power-up.
        idle_cycles: u64,
    },
    /// A command was sent above 400 kHz while the card was in idle, ready or identification.
    CommandClockAbove400k {
        /// Command index on the wire.
        index: u8,
        /// Card state when the command arrived.
        state: CardState,
        /// Clock rate in hertz.
        hz: u32,
    },
    /// An operation ran with the clock above 25 MHz.
    ClockAbove25M {
        /// Clock rate in hertz.
        hz: u32,
    },
    /// A command other than CMD0, CMD12, CMD13 or a CMD7 deselect arrived while the card was busy.
    CommandWhileBusy {
        /// Command index on the wire.
        index: u8,
        /// Busy time still to run.
        remaining_us: u64,
    },
    /// A block was moved with a word count that does not match the pending transfer.
    BlockLengthMismatch {
        /// True for a write.
        write: bool,
        /// Words the pending transfer has.
        expected_words: usize,
        /// Words the host used.
        got_words: usize,
    },
    /// Data moved before ACMD6 selected four-bit mode.
    DataBeforeFourBit {
        /// True for a write.
        write: bool,
    },
    /// A command other than CMD0, CMD12 or CMD13 arrived while a data transfer was pending.
    CommandWhileTransferPending {
        /// Command index on the wire.
        index: u8,
    },
    /// A frame whose start or transmission bit is not 0 then 1.
    MalformedFrame {
        /// The frame as sent.
        frame: [u8; 6],
    },
    /// The host's response buffer is not the kind the command produces.
    ResponseBufferMismatch {
        /// Command index on the wire.
        index: u8,
        /// What the command produces.
        expected: ResponseClass,
        /// What the host supplied.
        given: ResponseClass,
    },
    /// A command the card knows was sent in a state where it is not allowed.
    CommandInWrongState {
        /// Command index on the wire.
        index: u8,
        /// True if it was sent as an application command.
        app: bool,
        /// Card state when it arrived.
        state: CardState,
    },
    /// ACMD41 asked for high capacity before CMD8 had been accepted.
    HcsWithoutCmd8,
    /// A block was moved while the card was busy.
    DataWhileBusy {
        /// True for a write.
        write: bool,
    },
    /// `read_block` or `write_block` was called with no matching transfer pending.
    DataWithoutCommand {
        /// True for a write.
        write: bool,
    },
    /// A block was sent in a multi-block write after the card had answered an earlier block with
    /// the CRC-error or write-error token. The card ignores every further block until CMD12.
    WriteAfterErrorToken,
    /// CMD23 was sent to a card whose SCR does not advertise it.
    Cmd23NotSupported,
    /// CMD23 asked for a block count of zero.
    ZeroBlockCount,
}

/// One bus call, as the card saw it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceEvent {
    /// `set_clock`.
    SetClock {
        /// The cap asked for.
        requested_hz: u32,
        /// The rate granted.
        actual_hz: u32,
    },
    /// `clock_idle`.
    ClockIdle {
        /// Cycles run.
        cycles: u32,
    },
    /// `command`.
    Command {
        /// The frame as sent.
        frame: [u8; 6],
        /// Index decoded from the frame.
        index: u8,
        /// Argument decoded from the frame.
        arg: u32,
        /// The response frame that was put on the bus, after any injected corruption.
        response: Option<Vec<u8>>,
        /// What the call returned.
        result: Result<(), BusError>,
    },
    /// `read_block`.
    ReadBlock {
        /// Block address read, or `None` for SCR, SD status and the write-count block.
        lba: Option<u32>,
        /// Words in the host's buffer.
        words: usize,
        /// The timeout passed.
        timeout_ms: u32,
        /// The CRC returned, or the error.
        result: Result<u64, BusError>,
    },
    /// `write_block`.
    WriteBlock {
        /// Block address targeted, if a write was pending.
        lba: Option<u32>,
        /// Words in the host's buffer.
        words: usize,
        /// The CRC the host sent.
        crc: u64,
        /// The timeout passed.
        timeout_ms: u32,
        /// The status token returned, or the error.
        result: Result<u8, BusError>,
    },
    /// `wait_ready`.
    WaitReady {
        /// The timeout passed.
        timeout_ms: u32,
        /// What the call returned.
        result: Result<(), BusError>,
    },
    /// `delay_ms`.
    DelayMs {
        /// Milliseconds waited.
        ms: u32,
    },
}

/// A trace event with the virtual time at which the call began.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceEntry {
    /// Virtual microseconds at the start of the call.
    pub at_us: u64,
    /// The call.
    pub event: TraceEvent,
}
