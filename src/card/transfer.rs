//! Block reads and writes (Physical Layer Simplified Specification, section 4.3).

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;

use super::{Block, CapacityClass, CardInfo, Error, SdCard, add};
use crate::bus::Bus;
use crate::proto::{CardState, Command, CrcStatus, crc16_4bit};

/// Most blocks one CMD23 can announce.
const MAX_BLOCKS_PER_COMMAND: usize = 0xFFFF;

/// The request's blocks, in the direction of transfer.
enum Buf<'a> {
    Read(&'a mut [Block]),
    Write(&'a [Block]),
}

#[derive(Clone, Copy)]
struct Target {
    rca: u16,
    block_addressed: bool,
    uses_cmd23: bool,
    capacity_blocks: u64,
}

impl Target {
    fn new(info: &CardInfo) -> Self {
        Self {
            rca: info.rca,
            block_addressed: info.capacity_class != CapacityClass::Standard,
            uses_cmd23: info.uses_cmd23,
            capacity_blocks: info.capacity_blocks,
        }
    }

    /// The command argument for block `lba`: a block number, or a byte address on SDSC.
    fn address(&self, lba: u32) -> u32 {
        if self.block_addressed {
            lba
        } else {
            lba.saturating_mul(512)
        }
    }
}

/// Runs the bus operation `bus` and the CPU-only `work` at the same time: polls `bus` once so
/// the hardware starts, runs `work`, then polls `bus` to completion. If `bus` is already done
/// after its first poll, only `work` remains. The bus operation is never started late and never
/// polled after it completes.
async fn overlap<F: Future, T>(bus: F, work: impl FnOnce() -> T) -> (F::Output, T) {
    let mut bus = pin!(bus);
    let first = poll_fn(|cx| Poll::Ready(bus.as_mut().poll(cx))).await;
    let worked = work();
    match first {
        Poll::Ready(output) => (output, worked),
        Poll::Pending => (bus.await, worked),
    }
}

fn block_count(n: usize) -> u16 {
    u16::try_from(n).unwrap_or(u16::MAX)
}

fn offset(lba: u32, i: usize) -> u32 {
    lba.saturating_add(u32::try_from(i).unwrap_or(u32::MAX))
}

impl<B: Bus> SdCard<B> {
    /// Reads `blocks.len()` blocks starting at block `lba`.
    ///
    /// A request longer than one CMD23 can announce (65535 blocks) is split. Fails with
    /// [`Error::OutOfRange`] before any bus traffic if the request reaches past the card. If a
    /// transfer fails part-way, one clean-up attempt follows (CMD13, then CMD12 if the card is
    /// still in a data phase, wait for idle, CMD13): if the card is back in the transfer state
    /// the original error is returned, otherwise [`Error::UnexpectedState`], and the remedy is
    /// [`init`](Self::init). Blocks before the failed one hold valid data.
    ///
    /// If an earlier application command (CMD55 and the command after it) was cut short, the
    /// first thing the clean-up does is close the pair with CMD55 and ACMD23 for one block, so
    /// the card does not take CMD13 for an application command. That costs two extra commands on
    /// the next call and only in that case; if those two fail too, the call returns
    /// [`Error::UnexpectedState`] and the next call tries again.
    pub async fn read_blocks(&mut self, lba: u32, blocks: &mut [Block]) -> Result<(), Error> {
        self.transfer(lba, Buf::Read(blocks)).await
    }

    /// Writes `blocks.len()` blocks starting at block `lba`. Splitting, range check and failure
    /// handling are as for [`read_blocks`](Self::read_blocks). After the last block the card's
    /// status is read and must show no error and the transfer state.
    pub async fn write_blocks(&mut self, lba: u32, blocks: &[Block]) -> Result<(), Error> {
        self.transfer(lba, Buf::Write(blocks)).await
    }

    async fn transfer(&mut self, lba: u32, mut buf: Buf<'_>) -> Result<(), Error> {
        let info = self.info.ok_or(Error::NotInitialised)?;
        let target = Target::new(&info);
        let total = match &buf {
            Buf::Read(b) => b.len(),
            Buf::Write(b) => b.len(),
        };
        let out_of_range = Error::OutOfRange {
            lba,
            blocks: total,
            capacity_blocks: target.capacity_blocks,
        };
        let end = u64::from(lba).saturating_add(total as u64);
        if end > target.capacity_blocks {
            return Err(out_of_range);
        }
        if !target.block_addressed
            && end.saturating_sub(1).saturating_mul(512) > u64::from(u32::MAX)
        {
            return Err(out_of_range);
        }
        self.resume().await?;

        self.in_progress = true;
        let mut done = 0usize;
        while done < total {
            let n = (total - done).min(MAX_BLOCKS_PER_COMMAND);
            let start = offset(lba, done);
            let result = match &mut buf {
                Buf::Read(b) => match b.get_mut(done..).and_then(|r| r.get_mut(..n)) {
                    Some(chunk) => self.read_chunk(target, start, chunk).await,
                    None => Err(out_of_range),
                },
                Buf::Write(b) => match b.get(done..).and_then(|r| r.get(..n)) {
                    Some(chunk) => self.write_chunk(target, start, chunk).await,
                    None => Err(out_of_range),
                },
            };
            if let Err(error) = result {
                self.cleanup(Some(start)).await?;
                self.in_progress = false;
                return Err(error);
            }
            done += n;
            match buf {
                Buf::Read(_) => add(&mut self.stats.blocks_read, n),
                Buf::Write(_) => add(&mut self.stats.blocks_written, n),
            }
        }
        self.in_progress = false;
        Ok(())
    }

    async fn read_chunk(&mut self, t: Target, lba: u32, chunk: &mut [Block]) -> Result<(), Error> {
        let n = chunk.len();
        let first = Some(lba);
        let cmd = if n == 1 {
            Command::read_single_block(t.address(lba))
        } else {
            if t.uses_cmd23 {
                self.r1(Command::set_block_count(block_count(n)), first)
                    .await?;
            }
            Command::read_multiple_block(t.address(lba))
        };
        self.r1(cmd, first).await?;
        let timeout = self.config.read_timeout_ms;
        // Block i is verified while block i+1 is being received.
        let mut received: Option<(u32, &Block, u64)> = None;
        for (i, block) in chunk.iter_mut().enumerate() {
            let (result, checked) =
                overlap(self.bus.read_block(block.words_mut(), timeout), || {
                    received.map(|(lba, block, got)| (lba, crc16_4bit(block.as_bytes()), got))
                })
                .await;
            if let Some((lba, expected, got)) = checked {
                self.check_data_crc(cmd.index, Some(lba), expected, got)?;
            }
            let this = offset(lba, i);
            match result {
                Ok(got) => received = Some((this, &*block, got)),
                Err(e) => return Err(self.bus_error(cmd.index, Some(this), e)),
            }
        }
        if let Some((lba, block, got)) = received {
            self.check_data_crc(cmd.index, Some(lba), crc16_4bit(block.as_bytes()), got)?;
        }
        if n > 1 && !t.uses_cmd23 {
            self.stop_and_wait(lba).await?;
        }
        self.expect_tran(t, lba, false).await
    }

    async fn write_chunk(&mut self, t: Target, lba: u32, chunk: &[Block]) -> Result<(), Error> {
        let n = chunk.len();
        let first = Some(lba);
        let cmd = if n == 1 {
            Command::write_block(t.address(lba))
        } else {
            if t.uses_cmd23 {
                self.r1(Command::set_block_count(block_count(n)), first)
                    .await?;
            } else {
                let pre_erase =
                    Command::set_wr_blk_erase_count(u32::try_from(n).unwrap_or(u32::MAX));
                self.acmd(t.rca, pre_erase, first).await?;
            }
            Command::write_multiple_block(t.address(lba))
        };
        self.r1(cmd, first).await?;
        let timeout = self.config.write_timeout_ms;
        // The CRC of block i+1 is computed while block i is on the wire.
        let mut crc = chunk.first().map_or(0, |b| crc16_4bit(b.as_bytes()));
        for (i, block) in chunk.iter().enumerate() {
            let this = offset(lba, i);
            let next = chunk.get(i + 1);
            let (result, next_crc) =
                overlap(self.bus.write_block(block.words(), crc, timeout), || {
                    next.map_or(0, |b| crc16_4bit(b.as_bytes()))
                })
                .await;
            crc = next_crc;
            let token = match result {
                Ok(token) => token,
                Err(e) => return Err(self.bus_error(cmd.index, Some(this), e)),
            };
            match CrcStatus::from_token(token) {
                CrcStatus::Accepted => {}
                token => {
                    add(&mut self.stats.write_rejections, 1);
                    return Err(Error::WriteRejected {
                        cmd: cmd.index,
                        lba: this,
                        token,
                    });
                }
            }
        }
        if n > 1 && !t.uses_cmd23 {
            self.stop_and_wait(lba).await?;
        }
        self.expect_tran(t, lba, true).await
    }

    /// CMD12 and the busy that follows it.
    async fn stop_and_wait(&mut self, lba: u32) -> Result<(), Error> {
        let stop = Command::stop_transmission();
        self.r1(stop, Some(lba)).await?;
        let timeout = self.config.write_timeout_ms;
        self.wait_ready(stop.index, Some(lba), timeout).await
    }

    /// CMD13: no error bits, and the transfer state. After a write the card may still be
    /// programming; then waits for it (at most `write_timeout_ms`) and asks once more.
    async fn expect_tran(&mut self, t: Target, lba: u32, after_write: bool) -> Result<(), Error> {
        let ask = Command::send_status(t.rca);
        let mut status = self.r1(ask, Some(lba)).await?;
        if after_write && (status.state() == CardState::Prg || !status.ready_for_data()) {
            let timeout = self.config.write_timeout_ms;
            self.wait_ready(ask.index, Some(lba), timeout).await?;
            status = self.r1(ask, Some(lba)).await?;
        }
        match status.state() {
            CardState::Tran => Ok(()),
            state => Err(Error::WrongState {
                cmd: ask.index,
                lba: Some(lba),
                state,
            }),
        }
    }
}
