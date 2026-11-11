//! Identification (Physical Layer Simplified Specification, section 4.2).

use super::{CapacityClass, CardInfo, Error, SdCard, UnsupportedReason, add, check_status};
use crate::bus::{Bus, BusError, ResponseBuf, as_bytes};
use crate::proto::{
    CardStatus, Cid, Command, Csd, CsdVersion, Ocr, Scr, decode_r3, decode_r6, decode_r7,
};

/// Check pattern sent with CMD8.
const CHECK_PATTERN: u8 = 0xAA;
/// Voltage-accepted code for 2.7-3.6 V.
const VOLTAGE_2_7_3_6: u8 = 0b0001;
/// Idle clock cycles before CMD0 (at least 74 are required).
const POWER_UP_CLOCKS: u32 = 80;
/// Pause between ACMD41 polls, in milliseconds.
const POLL_MS: u32 = 10;
/// Unanswered CMD55s, before any has been answered, after which the slot is taken to be empty.
const NO_CARD_MISSES: u32 = 3;
/// Times CMD3 is repeated when the card publishes RCA 0.
const RCA_TRIES: u32 = 4;
/// Capacity in 512-byte blocks above which a block-addressed card is SDXC (32 GB).
const SDHC_MAX_BLOCKS: u64 = 64 * 1024 * 1024;

impl<B: Bus> SdCard<B> {
    /// Identifies the card from CMD0, switches it to four-bit mode at the configured transfer
    /// clock, and reads its registers. May be called again at any time to start over.
    ///
    /// Polls ACMD41 every 10 ms for at most [`Config::init_timeout_ms`](super::Config) of delay
    /// in total. Clears [`info`](Self::info) first, so after a failure the driver is
    /// uninitialised.
    pub async fn init(&mut self) -> Result<CardInfo, Error> {
        self.info = None;
        self.in_progress = false;
        // The reset below ends any open application-command pair, so there is nothing to close.
        self.app_open = false;

        self.bus.set_clock(self.init_clock());
        self.bus.clock_idle(POWER_UP_CLOCKS).await;
        add(&mut self.stats.commands_sent, 1);
        let go_idle = Command::go_idle_state();
        if let Err(e) = self.bus.command(&go_idle.frame(), ResponseBuf::None).await {
            return Err(self.bus_error(go_idle.index, None, e));
        }

        let v2 = self.send_if_cond().await?;
        let ocr = self.power_up(v2).await?;

        let cid = Cid::parse(&self.command_long(Command::all_send_cid()).await?);
        let rca = self.publish_rca().await?;
        let csd = Csd::parse(&self.command_long(Command::send_csd(rca)).await?).map_err(|e| {
            Error::Unsupported {
                cmd: Command::send_csd(rca).index,
                reason: UnsupportedReason::Csd(e),
            }
        })?;
        if csd.version == CsdVersion::V3 {
            return Err(Error::Unsupported {
                cmd: 9,
                reason: UnsupportedReason::Sduc,
            });
        }
        let capacity_class = match (ocr.ccs(), csd.version) {
            (false, CsdVersion::V1) => CapacityClass::Standard,
            (true, CsdVersion::V2) if csd.capacity_blocks <= SDHC_MAX_BLOCKS => CapacityClass::High,
            (true, CsdVersion::V2) => CapacityClass::Extended,
            _ => {
                return Err(Error::Unsupported {
                    cmd: 9,
                    reason: UnsupportedReason::AddressingMismatch,
                });
            }
        };

        let clock_hz = self.bus.set_clock(self.data_clock());
        let timeout = self.config.write_timeout_ms;
        self.r1(Command::select_card(rca), None).await?;
        self.wait_ready(7, None, timeout).await?;
        if capacity_class == CapacityClass::Standard {
            self.r1(Command::set_blocklen(512), None).await?;
        }
        self.acmd(rca, Command::set_clr_card_detect(false), None)
            .await?;
        self.acmd(rca, Command::set_bus_width(true), None).await?;
        let scr = self.read_scr(rca).await?;
        if !scr.supports_4bit() {
            return Err(Error::Unsupported {
                cmd: 6,
                reason: UnsupportedReason::NoFourBit,
            });
        }

        let info = CardInfo {
            capacity_class,
            capacity_blocks: csd.capacity_blocks,
            rca,
            ocr,
            cid,
            csd,
            scr,
            clock_hz,
            uses_cmd23: scr.supports_cmd23(),
        };
        self.info = Some(info);
        Ok(info)
    }

    /// CMD8. True if the card answered (version 2 or later), false if it stayed silent.
    async fn send_if_cond(&mut self) -> Result<bool, Error> {
        let cmd = Command::send_if_cond(CHECK_PATTERN);
        let frame = match self.command_short(cmd, None).await {
            Ok(frame) => frame,
            Err(Error::Bus {
                error: BusError::Timeout,
                ..
            }) => return Ok(false),
            Err(e) => return Err(e),
        };
        let cond = decode_r7(&frame).map_err(|e| self.bad_response(cmd.index, None, e))?;
        if cond.voltage_accepted != VOLTAGE_2_7_3_6 {
            let reason = UnsupportedReason::Voltage {
                accepted: cond.voltage_accepted,
            };
            return Err(Error::Unsupported {
                cmd: cmd.index,
                reason,
            });
        }
        if cond.check_pattern != CHECK_PATTERN {
            let reason = UnsupportedReason::CheckPattern {
                sent: CHECK_PATTERN,
                got: cond.check_pattern,
            };
            return Err(Error::Unsupported {
                cmd: cmd.index,
                reason,
            });
        }
        Ok(true)
    }

    /// CMD55 + ACMD41 until the OCR reports power-up done. `v2` says whether CMD8 was answered,
    /// which is what allows the HCS bit.
    async fn power_up(&mut self, v2: bool) -> Result<Ocr, Error> {
        let acmd41 = Command::sd_send_op_cond(v2, Ocr::WINDOW_2_7_3_6);
        let timeout = self.config.init_timeout_ms;
        let mut waited = 0u32;
        let mut misses = 0u32;
        let mut answered = false;
        // A card that ignored CMD8 flagged ILLEGAL_COMMAND for the first R1 it sends.
        let mut forgive = if v2 { 0 } else { CardStatus::ILLEGAL_COMMAND };
        loop {
            let app = Command::app_cmd(0);
            match self.r1_raw(app, None).await {
                Ok(status) => {
                    answered = true;
                    check_status(app.index, None, CardStatus(status.0 & !forgive))?;
                    forgive = 0;
                }
                Err(Error::Bus {
                    error: BusError::Timeout,
                    ..
                }) if !answered => {
                    misses += 1;
                    if misses >= NO_CARD_MISSES {
                        return Err(Error::NoCard);
                    }
                    waited = self.poll_pause(waited, timeout).await;
                    continue;
                }
                Err(e) => return Err(e),
            }
            let frame = self.command_short(acmd41, None).await?;
            let ocr = decode_r3(&frame).map_err(|e| self.bad_response(acmd41.index, None, e))?;
            if ocr.power_up_done() {
                return Ok(ocr);
            }
            if waited >= timeout {
                return Err(Error::InitTimeout { waited_ms: waited });
            }
            waited = self.poll_pause(waited, timeout).await;
        }
    }

    /// Delays by one poll interval, but never past `timeout` in total. Returns the new total.
    async fn poll_pause(&mut self, waited: u32, timeout: u32) -> u32 {
        let step = POLL_MS.min(timeout.saturating_sub(waited));
        if step > 0 {
            self.bus.delay_ms(step).await;
        }
        waited.saturating_add(step)
    }

    /// CMD3, asking again while the card publishes the reserved address 0.
    async fn publish_rca(&mut self) -> Result<u16, Error> {
        let cmd = Command::send_relative_addr();
        for _ in 0..RCA_TRIES {
            let frame = self.command_short(cmd, None).await?;
            let (rca, status) =
                decode_r6(&frame).map_err(|e| self.bad_response(cmd.index, None, e))?;
            check_status(cmd.index, None, status)?;
            if rca != 0 {
                return Ok(rca);
            }
        }
        Err(Error::Unsupported {
            cmd: cmd.index,
            reason: UnsupportedReason::RcaZero,
        })
    }

    /// ACMD51: the 8-byte SCR as one two-word block.
    async fn read_scr(&mut self, rca: u16) -> Result<Scr, Error> {
        let cmd = Command::send_scr();
        self.acmd(rca, cmd, None).await?;
        let mut words = [0u32; 2];
        self.read_checked(cmd.index, None, &mut words).await?;
        let raw = as_bytes(&words)
            .first_chunk::<8>()
            .copied()
            .unwrap_or([0; 8]);
        Ok(Scr::parse(&raw))
    }
}
