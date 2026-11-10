//! The card driver: identification, block reads and writes, and error recovery, on top of any
//! [`Bus`].
//!
//! [`SdCard`] owns a bus. [`SdCard::init`] identifies the card and puts it in four-bit mode at
//! full speed; [`read_blocks`](SdCard::read_blocks) and [`write_blocks`](SdCard::write_blocks)
//! move 512-byte [`Block`]s. Every wait is bounded by a [`Config`] timeout, every response and
//! every data block is checked. A transfer that fails part-way is not retried: the driver makes
//! one attempt to bring the card back to the transfer state and returns the error.
//!
//! An operation marks itself in progress while it runs. If its future is dropped before it
//! finishes, the next call first makes the same single clean-up attempt.

mod block;
mod error;
mod init;
mod recovery;
mod transfer;

pub use block::Block;
pub use error::{Error, UnsupportedReason};

use crate::bus::{Bus, BusError, ResponseBuf};
use crate::proto::{CardStatus, Cid, Command, Csd, Ocr, ResponseError, Scr, decode_r1};

/// Highest clock the driver ever asks for, in hertz (default-speed SD).
const MAX_CLOCK_HZ: u32 = 25_000_000;
/// Highest clock used while the card is in identification mode, in hertz.
const MAX_INIT_CLOCK_HZ: u32 = 400_000;

/// Driver settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Clock cap during identification, in hertz. Values above 400 kHz are lowered to 400 kHz.
    pub init_clock_hz: u32,
    /// Clock cap for data transfer, in hertz. Values above 25 MHz are lowered to 25 MHz.
    pub max_clock_hz: u32,
    /// Total milliseconds of delay between ACMD41 polls before identification gives up.
    pub init_timeout_ms: u32,
    /// Milliseconds the card has to start each data block of a read.
    pub read_timeout_ms: u32,
    /// Milliseconds a write may keep the card busy, per wait: after each block, after CMD12 and
    /// while programming finishes.
    pub write_timeout_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            init_clock_hz: 400_000,
            max_clock_hz: 25_000_000,
            init_timeout_ms: 1000,
            read_timeout_ms: 100,
            write_timeout_ms: 500,
        }
    }
}

/// How the card is addressed and how large it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CapacityClass {
    /// SDSC: byte addressing, up to 2 GB.
    Standard,
    /// SDHC: block addressing, up to 32 GB.
    High,
    /// SDXC: block addressing, over 32 GB.
    Extended,
}

/// What identification learned about the card.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CardInfo {
    /// Standard, high or extended capacity.
    pub capacity_class: CapacityClass,
    /// Capacity in 512-byte blocks.
    pub capacity_blocks: u64,
    /// The relative card address.
    pub rca: u16,
    /// The OCR from the last ACMD41.
    pub ocr: Ocr,
    /// The card identification register.
    pub cid: Cid,
    /// The card-specific data register.
    pub csd: Csd,
    /// The SD configuration register.
    pub scr: Scr,
    /// The data-transfer clock in hertz, as the bus granted it.
    pub clock_hz: u32,
    /// True if multi-block transfers announce their length with CMD23.
    pub uses_cmd23: bool,
}

/// Counters since the driver was created. Each saturates at `u32::MAX`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Stats {
    /// Commands put on the bus (CMD55 and the command after it count separately).
    pub commands_sent: u32,
    /// Blocks read and kept.
    pub blocks_read: u32,
    /// Blocks written and kept.
    pub blocks_written: u32,
    /// Data blocks received with a CRC that did not match.
    pub read_crc_errors: u32,
    /// Written blocks the card answered with an error token.
    pub write_rejections: u32,
    /// Bus operations that ended in [`BusError::Timeout`].
    pub timeouts: u32,
    /// Responses rejected by the decoders.
    pub bad_responses: u32,
    /// Times a failed or abandoned operation was cleaned up (see [`SdCard::read_blocks`]).
    pub recoveries: u32,
}

fn add(counter: &mut u32, n: usize) {
    *counter = counter.saturating_add(u32::try_from(n).unwrap_or(u32::MAX));
}

/// An SD card on a [`Bus`].
#[derive(Debug)]
pub struct SdCard<B: Bus> {
    bus: B,
    config: Config,
    info: Option<CardInfo>,
    stats: Stats,
    in_progress: bool,
}

impl<B: Bus> SdCard<B> {
    /// A driver for the card on `bus`. Sends nothing; call [`init`](Self::init) next.
    pub fn new(bus: B, config: Config) -> Self {
        Self {
            bus,
            config,
            info: None,
            stats: Stats::default(),
            in_progress: false,
        }
    }

    /// What identification learned, or `None` before a successful [`init`](Self::init) and
    /// after a failed one.
    pub fn info(&self) -> Option<&CardInfo> {
        self.info.as_ref()
    }

    /// Sends CMD13 and returns the card status as received, error bits included.
    ///
    /// Fails with [`Error::NotInitialised`] before [`init`](Self::init).
    pub async fn status(&mut self) -> Result<CardStatus, Error> {
        let rca = self.info.ok_or(Error::NotInitialised)?.rca;
        self.resume().await?;
        self.r1_raw(Command::send_status(rca), None).await
    }

    /// The counters.
    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    /// The bus, for inspection.
    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Gives the bus back.
    pub fn release(self) -> B {
        self.bus
    }

    fn init_clock(&self) -> u32 {
        self.config.init_clock_hz.clamp(1, MAX_INIT_CLOCK_HZ)
    }

    fn data_clock(&self) -> u32 {
        self.config.max_clock_hz.clamp(1, MAX_CLOCK_HZ)
    }

    fn bus_error(&mut self, cmd: u8, lba: Option<u32>, error: BusError) -> Error {
        if error == BusError::Timeout {
            add(&mut self.stats.timeouts, 1);
        }
        Error::Bus { cmd, lba, error }
    }

    fn bad_response(&mut self, cmd: u8, lba: Option<u32>, error: ResponseError) -> Error {
        add(&mut self.stats.bad_responses, 1);
        Error::BadResponse { cmd, lba, error }
    }

    /// Sends a command that has a 48-bit response and returns the raw frame.
    async fn command_short(&mut self, cmd: Command, lba: Option<u32>) -> Result<[u8; 6], Error> {
        let mut frame = [0u8; 6];
        add(&mut self.stats.commands_sent, 1);
        match self
            .bus
            .command(&cmd.frame(), ResponseBuf::Short(&mut frame))
            .await
        {
            Ok(()) => Ok(frame),
            Err(e) => Err(self.bus_error(cmd.index, lba, e)),
        }
    }

    /// Sends a command that has a 136-bit response and returns the 16 register bytes.
    async fn command_long(&mut self, cmd: Command) -> Result<[u8; 16], Error> {
        let mut frame = [0u8; 17];
        add(&mut self.stats.commands_sent, 1);
        match self
            .bus
            .command(&cmd.frame(), ResponseBuf::Long(&mut frame))
            .await
        {
            Ok(()) => {
                crate::proto::decode_r2(&frame).map_err(|e| self.bad_response(cmd.index, None, e))
            }
            Err(e) => Err(self.bus_error(cmd.index, None, e)),
        }
    }

    /// Sends a command with an R1 or R1b response and returns the status, error bits unchecked.
    async fn r1_raw(&mut self, cmd: Command, lba: Option<u32>) -> Result<CardStatus, Error> {
        let frame = self.command_short(cmd, lba).await?;
        decode_r1(cmd.index, &frame).map_err(|e| self.bad_response(cmd.index, lba, e))
    }

    /// As [`r1_raw`](Self::r1_raw), failing if the status has any error bit.
    async fn r1(&mut self, cmd: Command, lba: Option<u32>) -> Result<CardStatus, Error> {
        let status = self.r1_raw(cmd, lba).await?;
        check_status(cmd.index, lba, status)
    }

    /// CMD55 then `cmd`, both checked.
    async fn acmd(
        &mut self,
        rca: u16,
        cmd: Command,
        lba: Option<u32>,
    ) -> Result<CardStatus, Error> {
        self.r1(Command::app_cmd(rca), lba).await?;
        self.r1(cmd, lba).await
    }

    /// Waits for the card to release DAT0, for at most `timeout_ms`.
    async fn wait_ready(
        &mut self,
        cmd: u8,
        lba: Option<u32>,
        timeout_ms: u32,
    ) -> Result<(), Error> {
        match self.bus.wait_ready(timeout_ms).await {
            Ok(()) => Ok(()),
            Err(e) => Err(self.bus_error(cmd, lba, e)),
        }
    }

    /// Receives one block into `words` and checks its CRC.
    async fn read_checked(
        &mut self,
        cmd: u8,
        lba: Option<u32>,
        words: &mut [u32],
    ) -> Result<(), Error> {
        let got = match self
            .bus
            .read_block(words, self.config.read_timeout_ms)
            .await
        {
            Ok(crc) => crc,
            Err(e) => return Err(self.bus_error(cmd, lba, e)),
        };
        let expected = crate::proto::crc16_4bit(crate::bus::as_bytes(words));
        self.check_data_crc(cmd, lba, expected, got)
    }

    /// Fails with [`Error::DataCrc`] if the CRC computed over a received block is not the one
    /// that came with it.
    fn check_data_crc(
        &mut self,
        cmd: u8,
        lba: Option<u32>,
        expected: u64,
        got: u64,
    ) -> Result<(), Error> {
        if expected != got {
            add(&mut self.stats.read_crc_errors, 1);
            return Err(Error::DataCrc {
                cmd,
                lba,
                expected,
                got,
            });
        }
        Ok(())
    }
}

fn check_status(cmd: u8, lba: Option<u32>, status: CardStatus) -> Result<CardStatus, Error> {
    if status.errors() == 0 {
        Ok(status)
    } else {
        Err(Error::CardStatus { cmd, lba, status })
    }
}
