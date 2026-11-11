//! On-target test of the `BlockDevice<512>` implementation of `SdCard` (feature
//! `block-device-driver-0-2`) on MARV V2 Rev 1 (microSD J11, DAT0-3 = GPIO28-31, CLK = GPIO32,
//! CMD = GPIO33): size, write/read-back through the trait, agreement with the driver's own
//! `read_blocks`, a request across the 8-block chunk boundary, and trait-path against
//! direct-path throughput. Writes only inside the 1 MiB scratch area `sd_smoke` uses; the card
//! is expendable.
#![no_std]
#![no_main]

use aligned::{A4, Aligned};
use block_device_driver::BlockDevice;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_time::Instant;
use panic_probe as _;
use pio_sdmmc::pio::{PioBus, PioBusConfig};
use pio_sdmmc::{Block, Config, Error, SdCard};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

type Sd = SdCard<PioBus<'static, PIO0, 0>>;
type Aln = Aligned<A4, [u8; 512]>;

/// First block of the 1 MiB scratch area `sd_smoke` writes (2048 blocks).
const SCRATCH: u32 = 50_000_000;
const BUF_BLOCKS: usize = 256;
const REPS: u32 = 4;

struct Report {
    failures: u32,
}

macro_rules! check {
    ($rep:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {{
        let ok: bool = $cond;
        if ok {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $rep.failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
        ok
    }};
}

static ALN_BUF: StaticCell<[Aln; BUF_BLOCKS]> = StaticCell::new();
static BLK_BUF: StaticCell<[Block; BUF_BLOCKS]> = StaticCell::new();

/// Fills 512 bytes with a xorshift stream seeded from the block address and a salt.
fn fill_bytes(bytes: &mut [u8], lba: u32, salt: u32) {
    let mut x = (lba ^ salt).wrapping_mul(0x9E37_79B1) ^ 0xA5C3_5A3C | 1;
    for chunk in bytes.as_chunks_mut::<4>().0 {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *chunk = x.to_le_bytes();
    }
}

fn fill_aln(blocks: &mut [Aln], first_lba: u32, salt: u32) {
    for (i, b) in blocks.iter_mut().enumerate() {
        fill_bytes(&mut b[..], first_lba + i as u32, salt);
    }
}

fn fill_blk(blocks: &mut [Block], first_lba: u32, salt: u32) {
    for (i, b) in blocks.iter_mut().enumerate() {
        fill_bytes(b.as_bytes_mut(), first_lba + i as u32, salt);
    }
}

fn aln_differing(blocks: &[Aln], first_lba: u32, salt: u32) -> u32 {
    let mut want = [0u8; 512];
    let mut n = 0;
    for (i, b) in blocks.iter().enumerate() {
        fill_bytes(&mut want, first_lba + i as u32, salt);
        n += (b[..] != want[..]) as u32;
    }
    n
}

fn blk_differing(blocks: &[Block], first_lba: u32, salt: u32) -> u32 {
    let mut want = [0u8; 512];
    let mut n = 0;
    for (i, b) in blocks.iter().enumerate() {
        fill_bytes(&mut want, first_lba + i as u32, salt);
        n += (b.as_bytes() != &want[..]) as u32;
    }
    n
}

fn stat(what: &str, total_us: u32, blocks: u32) {
    // bytes per microsecond is MB/s.
    let mb_s = (blocks * 512) as f32 / total_us as f32;
    defmt::info!(
        "[STAT] {=str}: {=u32} blocks in {=u32} us: {=f32} MB/s",
        what,
        blocks,
        total_us,
        mb_s
    );
}

async fn identify(rep: &mut Report, card: &mut Sd) -> Option<u64> {
    let init = card.init().await;
    if !check!(rep, init.is_ok(), "{=str} init succeeds",) {
        return None;
    }
    let Some(info) = card.info() else {
        check!(rep, false, "{=str} card.info() after init");
        return None;
    };
    Some(info.capacity_blocks)
}

async fn size(rep: &mut Report, card: &mut Sd, capacity_blocks: u64) {
    let size = BlockDevice::size(card).await;
    defmt::info!(
        "[INFO] trait size() = {} bytes, init reported {} blocks",
        size.ok(),
        capacity_blocks
    );
    check!(
        rep,
        size == Ok(capacity_blocks * 512),
        "{=str} size() equals the capacity init reported, in bytes"
    );
}

async fn round_trip(
    rep: &mut Report,
    card: &mut Sd,
    aln: &mut [Aln],
    blk: &mut [Block],
    n: usize,
    lba: u32,
    salt: u32,
) {
    fill_aln(&mut aln[..n], lba, salt);
    let w = BlockDevice::write(card, lba, &aln[..n]).await;
    aln[..n].fill(Aligned([0; 512]));
    let r = BlockDevice::read(card, lba, &mut aln[..n]).await;
    let differing = aln_differing(&aln[..n], lba, salt);
    check!(
        rep,
        w.is_ok() && r.is_ok() && differing == 0,
        "{=str} {=usize}-block write then read through the trait: write {}, read {}, {} differing blocks",
        n,
        w.is_ok(),
        r.is_ok(),
        differing
    );
    blk[..n].fill(Block::new());
    let d = card.read_blocks(lba, &mut blk[..n]).await;
    let differing = blk_differing(&blk[..n], lba, salt);
    check!(
        rep,
        d.is_ok() && differing == 0,
        "{=str} {=usize} blocks read with read_blocks agree with the trait: read {}, {} differing blocks",
        n,
        d.is_ok(),
        differing
    );
}

async fn throughput(rep: &mut Report, card: &mut Sd, aln: &mut [Aln], blk: &mut [Block]) {
    let first = SCRATCH + 256;
    fill_aln(aln, first, 3);
    fill_blk(blk, first, 3);
    let mut t = [0u32; 4];
    let mut errors = 0u32;
    for rep_i in 0..REPS {
        let lba = first + rep_i * BUF_BLOCKS as u32;
        let t0 = Instant::now();
        errors += BlockDevice::write(card, lba, &aln[..]).await.is_err() as u32;
        t[0] += t0.elapsed().as_micros() as u32;
        let t0 = Instant::now();
        errors += BlockDevice::read(card, lba, &mut aln[..]).await.is_err() as u32;
        t[1] += t0.elapsed().as_micros() as u32;
        let t0 = Instant::now();
        errors += card.write_blocks(lba, &blk[..]).await.is_err() as u32;
        t[2] += t0.elapsed().as_micros() as u32;
        let t0 = Instant::now();
        errors += card.read_blocks(lba, &mut blk[..]).await.is_err() as u32;
        t[3] += t0.elapsed().as_micros() as u32;
    }
    let blocks = REPS * BUF_BLOCKS as u32;
    stat("trait write, 256 blocks per call", t[0], blocks);
    stat("direct write, 256 blocks per call", t[2], blocks);
    stat("trait read, 256 blocks per call", t[1], blocks);
    stat("direct read, 256 blocks per call", t[3], blocks);
    check!(
        rep,
        errors == 0,
        "{=str} throughput runs: {} errors",
        errors
    );
}

async fn error_path(rep: &mut Report, card: &mut Sd, aln: &mut [Aln], capacity_blocks: u64) {
    let r = BlockDevice::read(card, (capacity_blocks - 4) as u32, &mut aln[..8]).await;
    check!(
        rep,
        matches!(r, Err(Error::OutOfRange { .. })),
        "{=str} a trait read reaching past the end returns OutOfRange"
    );
    let next = BlockDevice::read(card, SCRATCH, &mut aln[..8]).await;
    check!(rep, next.is_ok(), "{=str} the next ordinary read succeeds");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut rep = Report { failures: 0 };
    defmt::info!(
        "[INFO] sd_blockdev: clk_sys = {} Hz",
        embassy_rp::clocks::clk_sys_freq()
    );

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let ch = dma::Channel::new(p.DMA_CH0, Irqs);
    let bus = PioBus::new(
        &mut common,
        sm0,
        ch,
        p.PIN_32,
        p.PIN_33,
        p.PIN_28,
        p.PIN_29,
        p.PIN_30,
        p.PIN_31,
        PioBusConfig::default(),
    );
    let mut card: Sd = SdCard::new(bus, Config::default());
    let aln = ALN_BUF.init([Aligned([0u8; 512]); BUF_BLOCKS]);
    let blk = BLK_BUF.init([Block::new(); BUF_BLOCKS]);

    if let Some(capacity_blocks) = identify(&mut rep, &mut card).await {
        size(&mut rep, &mut card, capacity_blocks).await;
        round_trip(&mut rep, &mut card, aln, blk, 64, SCRATCH, 1).await;
        round_trip(&mut rep, &mut card, aln, blk, 17, SCRATCH + 64, 2).await;
        throughput(&mut rep, &mut card, aln, blk).await;
        error_path(&mut rep, &mut card, aln, capacity_blocks).await;
    }
    hil_common::exit(rep.failures)
}
