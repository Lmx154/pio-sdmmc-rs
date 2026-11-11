//! The single clean-up attempt after a failed or abandoned operation.

use super::{Error, SdCard, add};
use crate::bus::Bus;
use crate::proto::{CardState, Command};

impl<B: Bus> SdCard<B> {
    /// If an earlier operation was abandoned part-way, cleans up after it first.
    pub(super) async fn resume(&mut self) -> Result<(), Error> {
        if self.in_progress || self.app_open {
            self.cleanup(None).await?;
            self.in_progress = false;
        }
        Ok(())
    }

    /// One best-effort attempt to bring the card back to the transfer state: if an application
    /// command was cut short, first CMD55 and ACMD23 with a block count of one to close it (the
    /// card takes CMD13 as ACMD13 while it still expects an application command; a failure here
    /// leaves the mark set and ends the attempt, and the next call tries again); then CMD13; if the card
    /// says it is still in a data phase, CMD12 (its failure is tolerated); then waits out any
    /// busy and asks with CMD13 once more. Error bits in the status belong to the failure being
    /// cleaned up after and are ignored. CMD12 is sent only to a card that says it is in a data
    /// phase, because it is illegal in any other state.
    pub(super) async fn cleanup(&mut self, lba: Option<u32>) -> Result<(), Error> {
        add(&mut self.stats.recoveries, 1);
        let rca = self.info.ok_or(Error::NotInitialised)?.rca;
        if self.app_open {
            // Relies on the rule that the command after CMD55 is taken as an application command
            // only if one with that index exists. The rule is from the SD Physical Layer
            // Simplified Specification, section on application-specific commands (CMD55 /
            // APP_CMD); it was not re-read for this change. The simulated card implements it
            // (src/sim/card.rs:240). Still to be confirmed on a real card.
            //
            // As for a failed CMD13 below, a failure here ends the attempt with
            // `UnexpectedState`, which replaces the error of the operation being cleaned up
            // after. The mark stays set, so the next call tries the pair again.
            let neutral = Command::set_wr_blk_erase_count(1);
            if let Err(e) = self.acmd_raw(rca, neutral, lba).await {
                let cmd = match e {
                    Error::Bus { cmd, .. } | Error::BadResponse { cmd, .. } => cmd,
                    _ => neutral.index,
                };
                return Err(Error::UnexpectedState {
                    cmd,
                    lba,
                    state: None,
                });
            }
        }
        let timeout = self.config.write_timeout_ms;
        let ask = Command::send_status(rca);
        let stop = Command::stop_transmission();
        let mut cmd = ask.index;
        let mut state = None;
        for round in 0..2 {
            cmd = ask.index;
            let Ok(status) = self.r1_raw(ask, lba).await else {
                break;
            };
            state = Some(status.state());
            if status.state() == CardState::Tran && status.ready_for_data() {
                return Ok(());
            }
            if round == 1 {
                break;
            }
            match status.state() {
                CardState::Data | CardState::Rcv => {
                    cmd = stop.index;
                    let _ = self.r1_raw(stop, lba).await;
                    let _ = self.wait_ready(cmd, lba, timeout).await;
                }
                CardState::Tran | CardState::Prg => {
                    if self.wait_ready(cmd, lba, timeout).await.is_err() {
                        break;
                    }
                }
                _ => break,
            }
        }
        Err(Error::UnexpectedState { cmd, lba, state })
    }
}
