//! Shared fixtures for the card driver tests.
#![allow(dead_code)]

use embassy_futures::block_on;
use pio_sdmmc::card::{Block, Config, SdCard};
use pio_sdmmc::proto::crc7;
use pio_sdmmc::sim::{CardKind, SimCard, SimConfig, TraceEntry, TraceEvent};
use pio_sdmmc::{Bus, BusError, ResponseBuf};

pub const RCA: u16 = 0xAAAA;

/// 512 distinct-looking bytes that depend on `seed`.
pub fn pattern(seed: u8) -> [u8; 512] {
    let mut b = [0u8; 512];
    for (i, v) in b.iter_mut().enumerate() {
        *v = (i as u8)
            .wrapping_mul(7)
            .wrapping_add(seed.wrapping_mul(31))
            .wrapping_add((i >> 8) as u8);
    }
    b
}

pub fn blk(seed: u8) -> Block {
    pattern(seed).into()
}

pub fn blocks(first_seed: u8, n: usize) -> Vec<Block> {
    (0..n)
        .map(|i| blk(first_seed.wrapping_add(i as u8)))
        .collect()
}

pub fn sim_cfg(kind: CardKind, cmd23: bool) -> SimConfig {
    let capacity_blocks = match kind {
        CardKind::SdscV1 | CardKind::SdscV2 => 2_097_152,
        CardKind::Sdhc => 15_523_840,
        CardKind::Sdxc => 124_735_488,
    };
    SimConfig {
        kind,
        capacity_blocks,
        supports_cmd23: cmd23,
        ..SimConfig::default()
    }
}

pub fn fresh(kind: CardKind, cmd23: bool) -> SdCard<SimCard> {
    SdCard::new(SimCard::new(sim_cfg(kind, cmd23)), Config::default())
}

/// An initialised card of this kind.
pub fn ready(kind: CardKind, cmd23: bool) -> SdCard<SimCard> {
    let mut card = fresh(kind, cmd23);
    block_on(card.init()).expect("init");
    assert_eq!(card.bus().violations(), &[]);
    card
}

/// An initialised default (SDXC, CMD23) card.
pub fn default_ready() -> SdCard<SimCard> {
    ready(CardKind::Sdxc, true)
}

/// Command indices in a trace, in order.
pub fn indices(trace: &[TraceEntry]) -> Vec<u8> {
    trace
        .iter()
        .filter_map(|e| match e.event {
            TraceEvent::Command { index, .. } => Some(index),
            _ => None,
        })
        .collect()
}

/// `(index, arg)` of every command in a trace.
pub fn commands(trace: &[TraceEntry]) -> Vec<(u8, u32)> {
    trace
        .iter()
        .filter_map(|e| match e.event {
            TraceEvent::Command { index, arg, .. } => Some((index, arg)),
            _ => None,
        })
        .collect()
}

pub fn count_cmd(trace: &[TraceEntry], index: u8) -> usize {
    indices(trace).iter().filter(|&&i| i == index).count()
}

/// Runs `f` and returns what it did to the sim trace.
pub fn traced<T>(
    card: &mut SdCard<SimCard>,
    f: impl FnOnce(&mut SdCard<SimCard>) -> T,
) -> (T, Vec<TraceEntry>) {
    let before = card.bus().trace().len();
    let out = f(card);
    let trace = card.bus().trace()[before..].to_vec();
    (out, trace)
}

/// A bus that lets a test rewrite 48-bit responses on their way back.
pub struct Tamper {
    pub sim: SimCard,
    pub rewrite: fn(u8, &mut [u8; 6]),
}

impl Bus for Tamper {
    fn set_clock(&mut self, max_hz: u32) -> u32 {
        self.sim.set_clock(max_hz)
    }

    async fn clock_idle(&mut self, cycles: u32) {
        self.sim.clock_idle(cycles).await
    }

    async fn command(
        &mut self,
        frame: &[u8; 6],
        response: ResponseBuf<'_>,
    ) -> Result<(), BusError> {
        match response {
            ResponseBuf::Short(buf) => {
                let mut tmp = [0u8; 6];
                self.sim
                    .command(frame, ResponseBuf::Short(&mut tmp))
                    .await?;
                (self.rewrite)(frame[0] & 0x3F, &mut tmp);
                *buf = tmp;
                Ok(())
            }
            other => self.sim.command(frame, other).await,
        }
    }

    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError> {
        self.sim.read_block(data, timeout_ms).await
    }

    async fn write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError> {
        self.sim.write_block(data, crc, timeout_ms).await
    }

    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError> {
        self.sim.wait_ready(timeout_ms).await
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.sim.delay_ms(ms).await
    }
}

/// An R7 frame with the given payload bytes `[0, 0, vca, pattern]`.
pub fn r7(vca: u8, pattern: u8) -> [u8; 6] {
    let mut f = [8, 0, 0, vca, pattern, 0];
    f[5] = (crc7(&f[..5]) << 1) | 1;
    f
}
