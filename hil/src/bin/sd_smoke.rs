//! End-to-end test of the card driver on MARV V2 Rev 1 (microSD J11, DAT0-3 = GPIO28-31,
//! CLK = GPIO32, CMD = GPIO33): `pio_sdmmc::SdCard` over `PioBus`, the way a logging layer uses
//! it. Identification, a 1 MiB write/read-back compare, throughput at 1/8/64/256 blocks per call,
//! the latency of 4 KiB flushes, and one error path. The card is expendable: blocks are
//! overwritten.
#![no_std]
#![no_main]

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

const EXPECT_BLOCKS: u64 = 124_735_488;
const EXPECT_NAME: &[u8; 5] = b"SK64G";
/// First block of the 1 MiB scratch area, well inside the card.
const SCRATCH: u32 = 50_000_000;
const MIB_BLOCKS: usize = 2048;
const BUF_BLOCKS: usize = 256;
const FLUSHES: usize = 256;
const FLUSH_BLOCKS: usize = 8;

struct Report {
    failures: u32,
}

/// `check!(rep, cond, "{=str} format", args..)`: the first placeholder receives "[PASS]" or
/// "[FAIL]". Returns `cond`.
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

static WRITE_BUF: StaticCell<[Block; BUF_BLOCKS]> = StaticCell::new();
static READ_BUF: StaticCell<[Block; BUF_BLOCKS]> = StaticCell::new();

/// Fills each block with a xorshift stream seeded from its address.
fn fill(blocks: &mut [Block], first_lba: u32) {
    for (i, block) in blocks.iter_mut().enumerate() {
        let lba = first_lba.wrapping_add(i as u32);
        let mut x = lba.wrapping_mul(0x9E37_79B1) ^ 0xA5C3_5A3C | 1;
        for chunk in block.as_bytes_mut().as_chunks_mut::<4>().0 {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            *chunk = x.to_le_bytes();
        }
    }
}

fn us_since(t0: Instant) -> u32 {
    t0.elapsed().as_micros() as u32
}

struct Timing {
    total_us: u32,
    worst_us: u32,
    errors: u32,
}

/// 1 MiB in calls of `per_call` blocks: written from `buf` or read into it.
async fn run_mib(card: &mut Sd, buf: &mut [Block], per_call: usize, write: bool) -> Timing {
    let mut t = Timing {
        total_us: 0,
        worst_us: 0,
        errors: 0,
    };
    let start = Instant::now();
    for call in 0..MIB_BLOCKS / per_call {
        let lba = SCRATCH + (call * per_call) as u32;
        let t0 = Instant::now();
        let r = if write {
            card.write_blocks(lba, &buf[..per_call]).await
        } else {
            card.read_blocks(lba, &mut buf[..per_call]).await
        };
        t.worst_us = t.worst_us.max(us_since(t0));
        if r.is_err() {
            t.errors += 1;
            break;
        }
    }
    t.total_us = us_since(start);
    t
}

fn stat(what: &str, per_call: usize, t: &Timing) {
    // bytes per microsecond is MB/s.
    let mb_s = (MIB_BLOCKS * 512) as f32 / t.total_us as f32;
    let us_block = t.total_us as f32 / MIB_BLOCKS as f32;
    defmt::info!(
        "[STAT] {=str} 1 MiB in {=usize}-block calls: {=f32} MB/s, {=f32} us/block, longest call {=u32} us",
        what,
        per_call,
        mb_s,
        us_block,
        t.worst_us
    );
}

async fn identify(rep: &mut Report, card: &mut Sd) -> bool {
    let init = card.init().await;
    if !check!(rep, init.is_ok(), "{=str} init succeeds",) {
        if let Err(e) = init {
            defmt::error!("[INFO] init error: {}", e);
        }
        return false;
    }
    let Some(info) = card.info() else {
        check!(rep, false, "{=str} card.info() after init");
        return false;
    };
    defmt::info!(
        "[INFO] card: {} blocks, product {=[u8]:a}, clock {} Hz, CMD23 {}",
        info.capacity_blocks,
        &info.cid.product_name[..],
        info.clock_hz,
        info.uses_cmd23
    );
    check!(
        rep,
        info.capacity_blocks == EXPECT_BLOCKS,
        "{=str} capacity is {} blocks (expected 124735488)",
        info.capacity_blocks
    );
    check!(
        rep,
        &info.cid.product_name == EXPECT_NAME,
        "{=str} product name is SK64G"
    );
    check!(
        rep,
        info.clock_hz == 25_000_000,
        "{=str} data clock is {} Hz (expected 25000000)",
        info.clock_hz
    );
    check!(
        rep,
        info.uses_cmd23,
        "{=str} multi-block transfers use CMD23"
    );
    true
}

async fn correctness(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) {
    let mut errors = 0u32;
    for call in 0..MIB_BLOCKS / BUF_BLOCKS {
        let lba = SCRATCH + (call * BUF_BLOCKS) as u32;
        fill(wbuf, lba);
        if card.write_blocks(lba, wbuf).await.is_err() {
            errors += 1;
        }
    }
    let mut differing = 0u32;
    for call in 0..MIB_BLOCKS / BUF_BLOCKS {
        let lba = SCRATCH + (call * BUF_BLOCKS) as u32;
        fill(wbuf, lba);
        rbuf.fill(Block::new());
        if card.read_blocks(lba, rbuf).await.is_err() {
            errors += 1;
            continue;
        }
        differing += wbuf.iter().zip(rbuf.iter()).filter(|(a, b)| a != b).count() as u32;
    }
    check!(
        rep,
        errors == 0 && differing == 0,
        "{=str} 1 MiB write then read back: {} errors, {} differing blocks of {}",
        errors,
        differing,
        MIB_BLOCKS
    );
}

async fn speed(rep: &mut Report, card: &mut Sd, buf: &mut [Block]) {
    fill(buf, SCRATCH);
    for per_call in [1, 8, 64, 256] {
        let w = run_mib(card, buf, per_call, true).await;
        let r = run_mib(card, buf, per_call, false).await;
        stat("write", per_call, &w);
        stat("read", per_call, &r);
        check!(
            rep,
            w.errors == 0 && r.errors == 0,
            "{=str} speed run with {} blocks per call: {} write errors, {} read errors",
            per_call,
            w.errors,
            r.errors
        );
    }
}

async fn latency(rep: &mut Report, card: &mut Sd, buf: &mut [Block]) {
    fill(buf, SCRATCH);
    let mut worst_us = 0u32;
    let mut errors = 0u32;
    let start = Instant::now();
    for i in 0..FLUSHES {
        let lba = SCRATCH + (i * FLUSH_BLOCKS) as u32;
        let t0 = Instant::now();
        if card.write_blocks(lba, &buf[..FLUSH_BLOCKS]).await.is_err() {
            errors += 1;
            break;
        }
        worst_us = worst_us.max(us_since(t0));
    }
    let total_us = us_since(start);
    defmt::info!(
        "[STAT] {} consecutive {}-block (4 KiB) writes: mean {=f32} us/call, worst {=u32} us",
        FLUSHES,
        FLUSH_BLOCKS,
        total_us as f32 / FLUSHES as f32,
        worst_us
    );
    check!(
        rep,
        errors == 0,
        "{=str} 4 KiB flush run: {} errors",
        errors
    );
}

async fn error_path(rep: &mut Report, card: &mut Sd, buf: &mut [Block]) {
    let past_end = EXPECT_BLOCKS as u32;
    let r = card.read_blocks(past_end, &mut buf[..1]).await;
    let out_of_range = matches!(r, Err(Error::OutOfRange { .. }));
    check!(
        rep,
        out_of_range,
        "{=str} read past the end returns OutOfRange"
    );
    let next = card.read_blocks(SCRATCH, &mut buf[..8]).await;
    check!(rep, next.is_ok(), "{=str} the next ordinary read succeeds");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut rep = Report { failures: 0 };
    defmt::info!(
        "[INFO] sd_smoke: clk_sys = {} Hz",
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
    let wbuf = WRITE_BUF.init([Block::new(); BUF_BLOCKS]);
    let rbuf = READ_BUF.init([Block::new(); BUF_BLOCKS]);

    if identify(&mut rep, &mut card).await {
        correctness(&mut rep, &mut card, wbuf, rbuf).await;
        speed(&mut rep, &mut card, wbuf).await;
        latency(&mut rep, &mut card, wbuf).await;
        error_path(&mut rep, &mut card, rbuf).await;
    }
    hil_common::exit(rep.failures)
}
