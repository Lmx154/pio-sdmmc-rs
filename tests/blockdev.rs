//! `BlockDevice<512>` for `SdCard` against the simulated card.
#![cfg(feature = "block-device-driver-0-2")]

mod card_helpers;

use std::future::{Future, poll_fn};
use std::pin::pin;
use std::task::{Context, Poll, Waker};

use aligned::{A4, Aligned};
use block_device_driver::BlockDevice;
use card_helpers::*;
use embassy_futures::block_on;
use pio_sdmmc::card::{Block, Config, Error, SdCard};
use pio_sdmmc::sim::{CardKind, Fault, SimCard, Trigger};
use pio_sdmmc::{Bus, BusError, ResponseBuf};

type Aln = Aligned<A4, [u8; 512]>;

const LBA: u32 = 1000;
const SIZES: [usize; 6] = [1, 7, 8, 9, 16, 17];

fn aligned_blocks(first_seed: u8, n: usize) -> Vec<Aln> {
    (0..n)
        .map(|i| Aligned(pattern(first_seed.wrapping_add(i as u8))))
        .collect()
}

fn fill_old(card: &mut SdCard<SimCard>, first: u32, n: usize) {
    for i in 0..n {
        card.bus()
            .poke_block(first + i as u32, &pattern(200 + i as u8));
    }
}

fn contents(card: &mut SdCard<SimCard>, first: u32, n: usize) -> Vec<[u8; 512]> {
    (0..n)
        .map(|i| card.bus().peek_block(first + i as u32))
        .collect()
}

#[test]
fn trait_and_direct_calls_agree() {
    for n in SIZES {
        let mut via_trait = default_ready();
        let mut direct = default_ready();
        for card in [&mut via_trait, &mut direct] {
            fill_old(card, LBA - 2, n + 4);
        }

        let data = aligned_blocks(10, n);
        block_on(BlockDevice::write(&mut via_trait, LBA, &data)).unwrap();
        block_on(direct.write_blocks(LBA, &blocks(10, n))).unwrap();
        assert_eq!(
            contents(&mut via_trait, LBA - 2, n + 4),
            contents(&mut direct, LBA - 2, n + 4),
            "write of {n} blocks"
        );
        for i in 0..n {
            assert_eq!(
                via_trait.bus().peek_block(LBA + i as u32),
                pattern(10 + i as u8)
            );
        }

        let mut got = vec![Aligned([0u8; 512]); n];
        block_on(BlockDevice::read(&mut via_trait, LBA, &mut got)).unwrap();
        let mut want = vec![Block::new(); n];
        block_on(direct.read_blocks(LBA, &mut want)).unwrap();
        for (g, w) in got.iter().zip(&want) {
            assert_eq!(&g[..], w.as_bytes(), "read of {n} blocks");
        }
        assert_eq!(via_trait.bus().violations(), &[]);
    }
}

#[test]
fn seventeen_blocks_are_three_driver_operations() {
    // Chunks of 8, 8 and 1 block: two multi-block commands, and the last chunk is the driver's
    // own single-block command.
    let mut card = default_ready();
    let data = aligned_blocks(1, 17);
    let ((), trace) = traced(&mut card, |c| {
        block_on(BlockDevice::write(c, LBA, &data)).unwrap()
    });
    let cmds = commands(&trace);
    let data_cmds: Vec<_> = cmds.iter().filter(|c| matches!(c.0, 24 | 25)).collect();
    assert_eq!(
        data_cmds,
        [&(25, LBA), &(25, LBA + 8), &(24, LBA + 16)],
        "{cmds:?}"
    );
    assert_eq!(count_cmd(&trace, 23), 2);

    let mut out = vec![Aligned([0u8; 512]); 17];
    let ((), trace) = traced(&mut card, |c| {
        block_on(BlockDevice::read(c, LBA, &mut out)).unwrap()
    });
    let cmds = commands(&trace);
    let data_cmds: Vec<_> = cmds.iter().filter(|c| matches!(c.0, 17 | 18)).collect();
    assert_eq!(
        data_cmds,
        [&(18, LBA), &(18, LBA + 8), &(17, LBA + 16)],
        "{cmds:?}"
    );
    assert_eq!(out, data);
}

#[test]
fn size_is_capacity_in_bytes() {
    let mut card = fresh(CardKind::Sdxc, true);
    assert_eq!(
        block_on(BlockDevice::size(&mut card)),
        Err(Error::NotInitialised)
    );
    block_on(card.init()).unwrap();
    let blocks = card.info().unwrap().capacity_blocks;
    assert_eq!(blocks, 124_735_488);
    assert_eq!(block_on(BlockDevice::size(&mut card)), Ok(blocks * 512));
}

#[test]
fn not_initialised_before_init() {
    let mut card = fresh(CardKind::Sdxc, true);
    let mut buf = vec![Aligned([0u8; 512]); 1];
    assert_eq!(
        block_on(BlockDevice::read(&mut card, 0, &mut buf)),
        Err(Error::NotInitialised)
    );
    assert_eq!(
        block_on(BlockDevice::write(&mut card, 0, &buf)),
        Err(Error::NotInitialised)
    );
}

#[test]
fn range_past_the_end_writes_nothing() {
    let mut card = default_ready();
    let capacity = card.info().unwrap().capacity_blocks;
    // 12 blocks starting 10 before the end: the first chunk would fit, the request does not.
    let lba = (capacity - 10) as u32;
    fill_old(&mut card, lba, 10);
    let stored = card.bus().stored_blocks();
    let trace_len = card.bus().trace().len();

    let data = aligned_blocks(1, 12);
    let want = Error::OutOfRange {
        lba,
        blocks: 12,
        capacity_blocks: capacity,
    };
    assert_eq!(
        block_on(BlockDevice::write(&mut card, lba, &data)),
        Err(want)
    );
    let mut out = vec![Aligned([0u8; 512]); 12];
    assert_eq!(
        block_on(BlockDevice::read(&mut card, lba, &mut out)),
        Err(want)
    );
    assert_eq!(card.bus().trace().len(), trace_len, "no bus traffic");
    assert_eq!(card.bus().stored_blocks(), stored);
    for i in 0..10 {
        assert_eq!(card.bus().peek_block(lba + i), pattern(200 + i as u8));
    }
    // The last block of the card is addressable.
    block_on(BlockDevice::write(&mut card, lba + 9, &data[..1])).unwrap();
    assert_eq!(card.bus().peek_block(lba + 9), pattern(1));
}

#[test]
fn fault_in_second_chunk_keeps_first_chunk_only() {
    let mut card = default_ready();
    fill_old(&mut card, LBA, 17);
    // Block writes are counted from the injection: the 9th is the first block of chunk two.
    card.bus()
        .inject(Fault::WriteErrorToken, Trigger::block_write(9));
    let data = aligned_blocks(30, 17);
    let err = block_on(BlockDevice::write(&mut card, LBA, &data)).unwrap_err();
    assert!(
        matches!(err, Error::WriteRejected { lba, .. } if lba == LBA + 8),
        "{err:?}"
    );
    for i in 0..8 {
        assert_eq!(
            card.bus().peek_block(LBA + i),
            pattern(30 + i as u8),
            "block {i}"
        );
    }
    for i in 8..17 {
        assert_eq!(
            card.bus().peek_block(LBA + i),
            pattern(200 + i as u8),
            "block {i}"
        );
    }
    // Chunk three was never sent: a repeat of the whole write now succeeds.
    block_on(BlockDevice::write(&mut card, LBA, &data)).unwrap();
    for i in 0..17 {
        assert_eq!(card.bus().peek_block(LBA + i), pattern(30 + i as u8));
    }
}

/// Returns Pending `times` times, then Ready.
async fn stall(times: u32) {
    let mut left = times;
    poll_fn(|cx| {
        if left == 0 {
            Poll::Ready(())
        } else {
            left -= 1;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await
}

/// The simulated card, stalling before the `stall_at`-th block write (1-based). A data phase
/// needs two stalls for one of them to reach the caller.
struct StallingBus {
    sim: SimCard,
    writes: usize,
    stall_at: usize,
}

impl Bus for StallingBus {
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
        self.sim.command(frame, response).await
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
        self.writes += 1;
        if self.writes == self.stall_at {
            stall(2).await;
        }
        self.sim.write_block(data, crc, timeout_ms).await
    }

    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError> {
        self.sim.wait_ready(timeout_ms).await
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.sim.delay_ms(ms).await
    }
}

#[test]
fn dropped_write_then_repeated() {
    // The 3rd block write is inside chunk one, the 10th inside chunk two.
    for stall_at in [3, 10] {
        let sim = SimCard::new(sim_cfg(CardKind::Sdxc, true));
        let mut card = SdCard::new(
            StallingBus {
                sim,
                writes: 0,
                stall_at,
            },
            Config::default(),
        );
        block_on(card.init()).unwrap();
        let data = aligned_blocks(60, 17);
        {
            let mut fut = pin!(BlockDevice::write(&mut card, LBA, &data));
            let mut cx = Context::from_waker(Waker::noop());
            assert!(
                fut.as_mut().poll(&mut cx).is_pending(),
                "completed without reaching the stall at write {stall_at}"
            );
        }
        assert_eq!(card.bus().writes, stall_at);
        block_on(BlockDevice::write(&mut card, LBA, &data)).unwrap();
        let mut out = vec![Aligned([0u8; 512]); 17];
        block_on(BlockDevice::read(&mut card, LBA, &mut out)).unwrap();
        assert_eq!(out, data, "dropped at write {stall_at}");
        assert_eq!(card.bus().sim.violations(), &[]);
    }
}
