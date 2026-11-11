use core::fmt;

use crate::bus::BusError;
use crate::proto::{CardState, CardStatus, CrcStatus, RegisterError, ResponseError};

/// Why identification refused a card.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnsupportedReason {
    /// CMD8 answered, but the card does not accept 2.7-3.6 V (`accepted` is the VCA field).
    Voltage {
        /// Voltage-accepted field of the R7 response.
        accepted: u8,
    },
    /// CMD8 answered with a check pattern other than the one sent.
    CheckPattern {
        /// The pattern sent.
        sent: u8,
        /// The pattern echoed.
        got: u8,
    },
    /// The CSD is version 3.0: an ultra-capacity (SDUC) card, which needs a 64-bit address.
    Sduc,
    /// The CSD could not be interpreted.
    Csd(RegisterError),
    /// The OCR's capacity bit and the CSD version disagree about the card's addressing.
    AddressingMismatch,
    /// The SCR does not list four-bit bus width.
    NoFourBit,
    /// The card published relative address 0 on every request.
    RcaZero,
}

/// Why a card operation failed. Each variant names the command index (`cmd`, the ACMD number
/// for an application command) and the block address (`lba`) involved, where there is one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The operation needs a successful [`SdCard::init`](super::SdCard::init) first.
    NotInitialised,
    /// Nothing answered CMD55 during identification.
    NoCard,
    /// The card is not one this driver handles.
    Unsupported {
        /// The command whose response showed it.
        cmd: u8,
        /// What is wrong with the card.
        reason: UnsupportedReason,
    },
    /// ACMD41 never reported the card ready.
    InitTimeout {
        /// Milliseconds of polling delay spent.
        waited_ms: u32,
    },
    /// A bus operation failed.
    Bus {
        /// The command (or, for a data stage, the command that started it).
        cmd: u8,
        /// The block involved, if any.
        lba: Option<u32>,
        /// What the bus reported.
        error: BusError,
    },
    /// A response frame was damaged or not the one expected.
    BadResponse {
        /// The command whose response it was.
        cmd: u8,
        /// The block involved, if any.
        lba: Option<u32>,
        /// What is wrong with the frame.
        error: ResponseError,
    },
    /// The card reported error bits in its status.
    CardStatus {
        /// The command whose response carried them.
        cmd: u8,
        /// The block involved, if any.
        lba: Option<u32>,
        /// The status as received.
        status: CardStatus,
    },
    /// The card answered in a state other than the one the sequence needs.
    WrongState {
        /// The command whose response showed the state.
        cmd: u8,
        /// The block involved, if any.
        lba: Option<u32>,
        /// The state reported.
        state: CardState,
    },
    /// A data block arrived with a CRC that does not match its contents.
    DataCrc {
        /// The command that started the transfer.
        cmd: u8,
        /// The block involved, if any.
        lba: Option<u32>,
        /// The CRC computed over the received data.
        expected: u64,
        /// The CRC the card sent.
        got: u64,
    },
    /// The card refused a written block.
    WriteRejected {
        /// The command that started the transfer.
        cmd: u8,
        /// The block refused.
        lba: u32,
        /// The CRC status token the card answered with.
        token: CrcStatus,
    },
    /// The request reaches past the end of the card. Nothing was sent.
    OutOfRange {
        /// First block of the request.
        lba: u32,
        /// Blocks requested.
        blocks: usize,
        /// Blocks the card has.
        capacity_blocks: u64,
    },
    /// Recovery could not bring the card back to the transfer state. Call
    /// [`SdCard::init`](super::SdCard::init).
    ///
    /// One case needs no `init`: when `cmd` is 55 or 23 the two commands that close an interrupted
    /// application command failed, and the next call simply tries them again.
    UnexpectedState {
        /// The last command tried.
        cmd: u8,
        /// The block involved, if any.
        lba: Option<u32>,
        /// The last state the card reported, or `None` if it never answered.
        state: Option<CardState>,
    },
}

impl fmt::Display for UnsupportedReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Voltage { accepted } => {
                write!(f, "card does not accept 2.7-3.6 V (VCA {accepted:#x})")
            }
            Self::CheckPattern { sent, got } => {
                write!(f, "check pattern {sent:#04x} came back as {got:#04x}")
            }
            Self::Sduc => f.write_str("SDUC card (needs 64-bit addressing)"),
            Self::Csd(e) => write!(f, "unreadable CSD ({e:?})"),
            Self::AddressingMismatch => f.write_str("OCR capacity bit and CSD version disagree"),
            Self::NoFourBit => f.write_str("SCR does not list four-bit bus width"),
            Self::RcaZero => f.write_str("card kept publishing RCA 0"),
        }
    }
}

fn at(f: &mut fmt::Formatter<'_>, lba: &Option<u32>) -> fmt::Result {
    match lba {
        Some(lba) => write!(f, " at block {lba}"),
        None => Ok(()),
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotInitialised => f.write_str("card not initialised"),
            Self::NoCard => f.write_str("no card answered during identification"),
            Self::Unsupported { cmd, reason } => write!(f, "unsupported card (CMD{cmd}): {reason}"),
            Self::InitTimeout { waited_ms } => {
                write!(f, "card not ready after ACMD41 polling for {waited_ms} ms")
            }
            Self::Bus { cmd, lba, error } => {
                write!(f, "bus error {error:?} during CMD{cmd}")?;
                at(f, lba)
            }
            Self::BadResponse { cmd, lba, error } => {
                write!(f, "bad response to CMD{cmd}: {error:?}")?;
                at(f, lba)
            }
            Self::CardStatus { cmd, lba, status } => {
                write!(
                    f,
                    "card status error {:#010x} in response to CMD{cmd}",
                    status.errors()
                )?;
                at(f, lba)
            }
            Self::WrongState { cmd, lba, state } => {
                write!(f, "card in state {state:?} after CMD{cmd}")?;
                at(f, lba)
            }
            Self::DataCrc {
                cmd,
                lba,
                expected,
                got,
            } => {
                write!(
                    f,
                    "data CRC mismatch on CMD{cmd} (computed {expected:#018x}, card sent {got:#018x})"
                )?;
                at(f, lba)
            }
            Self::WriteRejected { cmd, lba, token } => {
                write!(f, "block {lba} rejected during CMD{cmd}: {token:?}")
            }
            Self::OutOfRange {
                lba,
                blocks,
                capacity_blocks,
            } => {
                write!(
                    f,
                    "{blocks} block(s) from block {lba} exceed the card's {capacity_blocks} blocks"
                )
            }
            Self::UnexpectedState { cmd, lba, state } => {
                write!(
                    f,
                    "card not back in transfer state after recovery (last CMD{cmd}, state {state:?})"
                )?;
                at(f, lba)
            }
        }
    }
}

impl core::error::Error for Error {}
