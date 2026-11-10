//! Test-only code path for the on-target back-pressure check. Not part of the API.

use embassy_rp::pio::Instance;
use embassy_time::{Duration, Timer, with_timeout};

use super::driver::{Mapping, Op, PioBus};
use super::wire;
use crate::bus::BusError;

/// What the late-DMA read saw.
#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub struct LateDmaRead {
    /// The data CRC as received, as [`Bus::read_block`](crate::bus::Bus::read_block) returns it.
    pub crc: u64,
    /// RX FIFO level when the DMA was started (4 = full).
    pub level_at_dma_start: u8,
    /// FDEBUG.RXSTALL for this state machine after the read.
    pub rx_stalled: bool,
}

impl<PIO: Instance, const SM: usize> PioBus<'_, PIO, SM> {
    /// [`read_block`](crate::bus::Bus::read_block), except that the header goes in first and the
    /// DMA starts `delay` later, so the RX FIFO fills and the state machine stalls mid-block.
    #[doc(hidden)]
    pub async fn test_read_block_late_dma(
        &mut self,
        data: &mut [u32],
        timeout_ms: u32,
        delay: Duration,
    ) -> Result<LateDmaRead, BusError> {
        let w = data.len();
        let op = Op::new(self);
        let bus = &mut *op.bus;
        bus.select(Mapping::Dat);
        let _ = bus.sm.rx().stalled();
        bus.sm.tx().push(wire::read_header(w));
        Timer::after(delay).await;
        let level_at_dma_start = bus.sm.rx().level();
        let data_time =
            bus.bus_time(8 * w as u32 + 16 + 8) + Duration::from_millis(u64::from(timeout_ms));
        {
            let transfer = bus.sm.rx().dma_pull(&mut bus.dma, data, true);
            with_timeout(data_time, transfer)
                .await
                .map_err(|_| BusError::Timeout)?;
        }
        let tail = bus.bus_time(16 + 8);
        let hi = bus.pull_word(tail).await?;
        let lo = bus.pull_word(tail).await?;
        bus.pull_word(tail).await?;
        let rx_stalled = bus.sm.rx().stalled();
        op.done();
        Ok(LateDmaRead {
            crc: wire::crc_from_words(hi, lo),
            level_at_dma_start,
            rx_stalled,
        })
    }
}
