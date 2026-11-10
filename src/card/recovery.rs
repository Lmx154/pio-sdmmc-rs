//! The single clean-up attempt after a failed or abandoned operation.

use super::{Error, SdCard, add};
use crate::bus::Bus;
use crate::proto::{CardState, Command};

impl<B: Bus> SdCard<B> {
    /// If an earlier operation was abandoned part-way, cleans up after it first.
    pub(super) async fn resume(&mut self) -> Result<(), Error> {
        if self.in_progress {
            self.cleanup(None).await?;
            self.in_progress = false;
        }
        Ok(())
    }

    /// One best-effort attempt to bring the card back to the transfer state: CMD13; if the card
    /// says it is still in a data phase, CMD12 (its failure is tolerated); then waits out any
    /// busy and asks with CMD13 once more. Error bits in the status belong to the failure being
    /// cleaned up after and are ignored. CMD12 is sent only to a card that says it is in a data
    /// phase, because it is illegal in any other state.
    pub(super) async fn cleanup(&mut self, lba: Option<u32>) -> Result<(), Error> {
        add(&mut self.stats.recoveries, 1);
        let rca = self.info.ok_or(Error::NotInitialised)?.rca;
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
