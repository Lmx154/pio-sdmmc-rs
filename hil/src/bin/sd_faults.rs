//! Fault test of the card driver on the real card (MARV V2 Rev 1, microSD J11): futures of block
//! reads and writes are dropped at swept moments, and the very next operation must clean up by
//! itself. Also cuts an application-command pair (CMD55 + ACMD) in two on the real card.
//!
//! `FaultBus` wraps the real `PioBus` and implements `Bus` itself. It observes which kind of bus
//! operation a drop lands in, logs the commands of each call, counts CMD12, and has two switches
//! that exist only for X3: `stall_acmd` (the command that follows a CMD55 is never sent; the
//! future waits forever until the test drops it) and `tamper_scr` (during `init` the SCR read
//! back has its CMD23-support bit cleared, with a recomputed data CRC, so the driver takes the
//! ACMD23 + CMD12 route for multi-block writes, as it would for a card without CMD23). The card
//! itself is never lied to. The card is expendable: blocks are overwritten.
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_time::{Duration, Instant, with_timeout};
use panic_probe as _;
use pio_sdmmc::bus::{as_bytes, as_bytes_mut};
use pio_sdmmc::pio::{PioBus, PioBusConfig};
use pio_sdmmc::proto::{CardState, CardStatus, Command, crc16_4bit, decode_r1};
use pio_sdmmc::{Block, Bus, BusError, CardInfo, Config, Error, ResponseBuf, SdCard};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

type Sd = SdCard<FaultBus<PioBus<'static, PIO0, 0>>>;

const EXPECT_BLOCKS: u64 = 124_735_488;
const EXPECT_NAME: &[u8; 5] = b"SK64G";
/// First block of the test region (same place as sd_smoke); the region is `REGION` blocks.
const SCRATCH: u32 = 50_000_000;
const REGION: usize = 256;
/// Where the cancelled operations act, inside the region with a known block on each side.
const WORK: u32 = SCRATCH + 32;
const JUNK: u32 = 0x4000_0000;
/// Own bound on `init()` and on the whole-region transfers, in milliseconds.
const INIT_BOUND_MS: u64 = 10_000;

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Cmd,
    Data,
    Busy,
}

/// The real bus plus observation and the two X3 switches.
struct FaultBus<B: Bus> {
    inner: B,
    stall_acmd: bool,
    tamper_scr: bool,
    after_app: bool,
    cur: Option<Kind>,
    ops: u32,
    stalled: u32,
    cut_in: Option<(Kind, u32, u32)>,
    seen: [u32; 3],
    n_stop: u32,
    log: [(u8, u32); 8],
    log_n: usize,
}

impl<B: Bus> FaultBus<B> {
    fn new(inner: B) -> Self {
        Self {
            inner,
            stall_acmd: false,
            tamper_scr: false,
            after_app: false,
            cur: None,
            ops: 0,
            stalled: 0,
            cut_in: None,
            seen: [0; 3],
            n_stop: 0,
            log: [(0, 0); 8],
            log_n: 0,
        }
    }

    /// Forgets what the previous call did.
    fn begin_call(&mut self) {
        self.after_app = false;
        self.cur = None;
        self.ops = 0;
        self.stalled = 0;
        self.seen = [0; 3];
        self.log_n = 0;
    }

    /// Counts one bus operation of kind `k`; if it is the one `cut_in` names, how many
    /// polls it gets before the wrapper aborts it and parks the caller's future.
    fn cut_here(&mut self, k: Kind) -> Option<u32> {
        let i = match k {
            Kind::Cmd => 0,
            Kind::Data => 1,
            Kind::Busy => 2,
        };
        let n = self.seen.get_mut(i)?;
        *n += 1;
        match self.cut_in {
            Some((kk, nth, us)) if kk == k && nth == *n => Some(us),
            _ => None,
        }
    }

    /// 0 before the first bus operation, 1 in a command, 2 in a data block, 3 in a busy wait,
    /// 4 between two bus operations (the driver's own code).
    fn classify(&self) -> usize {
        match self.cur {
            Some(Kind::Cmd) => 1,
            Some(Kind::Data) => 2,
            Some(Kind::Busy) => 3,
            None if self.ops == 0 => 0,
            None => 4,
        }
    }

    /// The call began with CMD55 then CMD23 with argument 1: the clean-up's neutral pair.
    fn neutral_pair_first(&self) -> bool {
        self.log_n >= 2 && self.log[0].0 == 55 && self.log[1] == (23, 1)
    }
}

impl<B: Bus> Bus for FaultBus<B> {
    fn set_clock(&mut self, max_hz: u32) -> u32 {
        self.inner.set_clock(max_hz)
    }

    async fn clock_idle(&mut self, cycles: u32) {
        self.inner.clock_idle(cycles).await
    }

    async fn command(
        &mut self,
        frame: &[u8; 6],
        response: ResponseBuf<'_>,
    ) -> Result<(), BusError> {
        let idx = frame[0] & 0x3F;
        let arg = u32::from_be_bytes([frame[1], frame[2], frame[3], frame[4]]);
        self.ops += 1;
        if let Some(slot) = self.log.get_mut(self.log_n) {
            *slot = (idx, arg);
            self.log_n += 1;
        }
        if idx == 12 {
            self.n_stop += 1;
        }
        if self.stall_acmd && self.after_app && idx != 55 {
            self.stalled += 1;
            self.cur = None;
            core::future::pending::<()>().await;
        }
        self.cur = Some(Kind::Cmd);
        let r = match self.cut_here(Kind::Cmd) {
            Some(us) => match run_cut(Cut::Polls(us), self.inner.command(frame, response)).await {
                Some(r) => r,
                None => {
                    self.stalled += 1;
                    core::future::pending::<()>().await;
                    Err(BusError::Fault)
                }
            },
            None => self.inner.command(frame, response).await,
        };
        self.cur = None;
        self.after_app = r.is_ok() && idx == 55;
        r
    }

    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError> {
        self.ops += 1;
        self.cur = Some(Kind::Data);
        let mut r = self.inner.read_block(data, timeout_ms).await;
        self.cur = None;
        if self.tamper_scr && data.len() == 2 && r.is_ok() {
            if let Some(b) = as_bytes_mut(data).get_mut(3) {
                *b &= !0x02;
            }
            r = Ok(crc16_4bit(as_bytes(data)));
        }
        r
    }

    async fn write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError> {
        self.ops += 1;
        self.cur = Some(Kind::Data);
        let r = self.inner.write_block(data, crc, timeout_ms).await;
        self.cur = None;
        r
    }

    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError> {
        self.ops += 1;
        self.cur = Some(Kind::Busy);
        let r = match self.cut_here(Kind::Busy) {
            Some(us) => match run_cut(Cut::Polls(us), self.inner.wait_ready(timeout_ms)).await {
                Some(r) => r,
                None => {
                    self.stalled += 1;
                    core::future::pending::<()>().await;
                    Err(BusError::Fault)
                }
            },
            None => self.inner.wait_ready(timeout_ms).await,
        };
        self.cur = None;
        r
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.inner.delay_ms(ms).await
    }
}

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

static WRITE_BUF: StaticCell<[Block; REGION]> = StaticCell::new();
static READ_BUF: StaticCell<[Block; REGION]> = StaticCell::new();

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

fn diff_bytes(a: &[Block], b: &[Block]) -> u32 {
    a.iter()
        .zip(b)
        .map(|(x, y)| {
            if x == y {
                0
            } else {
                x.as_bytes()
                    .iter()
                    .zip(y.as_bytes())
                    .filter(|(p, q)| p != q)
                    .count() as u32
            }
        })
        .sum()
}

async fn bounded<F: Future>(ms: u64, f: F) -> Option<F::Output> {
    with_timeout(Duration::from_millis(ms), f).await.ok()
}

/// Counters of one section.
#[derive(Default)]
struct Sec {
    cancels: u32,
    completed: u32,
    /// Cancels by phase: before the first bus operation, command, data, busy wait, between.
    phase: [u32; 5],
    recoveries: u32,
    stops: u32,
    pairs: u32,
    worst_us: u32,
    failed: u32,
    wrong: u32,
    damage: u32,
    /// First operation after a cancel: status, read_blocks, write_blocks.
    next: [u32; 3],
}

#[derive(Clone, Copy)]
struct Case {
    sec: &'static str,
    op: &'static str,
    n: u32,
    d: u32,
}

fn print_sec(name: &str, s: &Sec) {
    defmt::info!(
        "[STAT] {=str}: cancels {} (ran to the end uncancelled {}); landed before-command {} command {} data {} busy {} between-ops {}",
        name,
        s.cancels,
        s.completed,
        s.phase[0],
        s.phase[1],
        s.phase[2],
        s.phase[3],
        s.phase[4]
    );
    defmt::info!(
        "[STAT] {=str}: clean-ups run {}, needed CMD12 {}, began with neutral CMD55+ACMD23(1) {}, longest first-operation-after-cancel (incl. its own work) {} us",
        name,
        s.recoveries,
        s.stops,
        s.pairs,
        s.worst_us
    );
    defmt::info!(
        "[STAT] {=str}: first operation after a cancel was status() {}, read_blocks {}, write_blocks {}",
        name,
        s.next[0],
        s.next[1],
        s.next[2]
    );
}

/// One CMD13 sent straight on the bus, with no driver clean-up.
async fn raw_status(card: &mut Sd) -> Option<CardStatus> {
    let rca = card.info()?.rca;
    let cmd = Command::send_status(rca);
    let mut frame = [0u8; 6];
    bounded(
        50,
        card.bus()
            .command(&cmd.frame(), ResponseBuf::Short(&mut frame)),
    )
    .await?
    .ok()?;
    decode_r1(13, &frame).ok()
}

/// Logs the first failing case in detail, with the card's status read raw.
async fn fail_case(
    rep: &mut Report,
    card: &mut Sd,
    sec: &mut Sec,
    case: Case,
    stage: &str,
    err: Option<Error>,
) {
    sec.failed += 1;
    let raw = raw_status(card).await;
    check!(
        rep,
        false,
        "{=str} {=str} first failing case: op {=str} n {} delay {} us, stage: {=str}, error {}",
        case.sec,
        case.op,
        case.n,
        case.d,
        stage,
        err
    );
    match raw {
        Some(s) => defmt::error!(
            "[INFO] raw CMD13 after the failure: status {=u32:#x}, state {=u32}, ready_for_data {=bool}, app_cmd {=bool}",
            s.0,
            (s.0 >> 9) & 0xF,
            s.ready_for_data(),
            s.app_cmd()
        ),
        None => defmt::error!("[INFO] raw CMD13 after the failure: no answer"),
    }
}

/// The next operation after a drop: `status()` (runs the driver's own clean-up first), timed.
async fn recover(rep: &mut Report, card: &mut Sd, sec: &mut Sec, case: Case) -> bool {
    let rec0 = card.stats().recoveries;
    let stop0 = card.bus().n_stop;
    card.bus().begin_call();
    let t0 = Instant::now();
    let r = bounded(3000, card.status()).await;
    let dt = us_since(t0);
    account(card, sec, rec0, stop0, dt);
    match r {
        Some(Ok(s)) if s.state() == CardState::Tran && s.ready_for_data() => true,
        Some(Ok(s)) => {
            defmt::error!("[INFO] status after recovery {=u32:#x}", s.0);
            fail_case(
                rep,
                card,
                sec,
                case,
                "status after clean-up not Tran/ready",
                None,
            )
            .await;
            false
        }
        Some(Err(e)) => {
            fail_case(
                rep,
                card,
                sec,
                case,
                "next operation (status) failed",
                Some(e),
            )
            .await;
            false
        }
        None => {
            fail_case(rep, card, sec, case, "next operation hung (3 s)", None).await;
            false
        }
    }
}

fn account(card: &mut Sd, sec: &mut Sec, rec0: u32, stop0: u32, dt: u32) {
    sec.worst_us = sec.worst_us.max(dt);
    sec.recoveries += card.stats().recoveries.wrapping_sub(rec0);
    let b = card.bus();
    if b.n_stop != stop0 {
        sec.stops += 1;
    }
    if b.neutral_pair_first() {
        sec.pairs += 1;
    }
}

/// The first operation after a cancel. Rotates between `status()`, a `read_blocks` of known
/// data at `read_lba` (checked byte-exact) and a `write_blocks` of the known pattern at
/// `write_lba`, so the driver's clean-up is reached from `resume()` of a status call and from
/// the start of a transfer. Nothing is sent in between.
#[allow(clippy::too_many_arguments)]
async fn after_cancel(
    rep: &mut Report,
    card: &mut Sd,
    sec: &mut Sec,
    case: Case,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    n: usize,
    read_lba: u32,
    write_lba: u32,
) -> bool {
    let kind = ((sec.next[0] + sec.next[1] + sec.next[2]) % 3) as usize;
    sec.next[kind] += 1;
    if kind == 0 {
        return recover(rep, card, sec, case).await;
    }
    let rec0 = card.stats().recoveries;
    let stop0 = card.bus().n_stop;
    card.bus().begin_call();
    let t0 = Instant::now();
    let r = if kind == 1 {
        rbuf[..n].fill(Block::new());
        bounded(3000, card.read_blocks(read_lba, &mut rbuf[..n])).await
    } else {
        fill(&mut wbuf[..n], write_lba);
        bounded(3000, card.write_blocks(write_lba, &wbuf[..n])).await
    };
    let dt = us_since(t0);
    account(card, sec, rec0, stop0, dt);
    let what = if kind == 1 {
        "first operation after the cancel (read_blocks) failed"
    } else {
        "first operation after the cancel (write_blocks) failed"
    };
    match r {
        Some(Ok(())) => {}
        Some(Err(e)) => {
            fail_case(rep, card, sec, case, what, Some(e)).await;
            return false;
        }
        None => {
            fail_case(
                rep,
                card,
                sec,
                case,
                "first operation after the cancel hung (3 s)",
                None,
            )
            .await;
            return false;
        }
    }
    if kind == 1 {
        fill(&mut wbuf[..n], read_lba);
        let w = diff_bytes(&rbuf[..n], &wbuf[..n]);
        if w != 0 {
            sec.wrong += w;
            fail_case(
                rep,
                card,
                sec,
                case,
                "wrong bytes in the read directly after the cancel",
                None,
            )
            .await;
            return false;
        }
    }
    true
}

/// Reads `n` blocks at `lba` and counts bytes that differ from the known pattern.
async fn read_known(
    rep: &mut Report,
    card: &mut Sd,
    sec: &mut Sec,
    case: Case,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    lba: u32,
    n: usize,
) -> bool {
    rbuf[..n].fill(Block::new());
    match bounded(3000, card.read_blocks(lba, &mut rbuf[..n])).await {
        Some(Ok(())) => {}
        Some(Err(e)) => {
            fail_case(rep, card, sec, case, "read of known data failed", Some(e)).await;
            return false;
        }
        None => {
            fail_case(rep, card, sec, case, "read of known data hung", None).await;
            return false;
        }
    }
    fill(&mut wbuf[..n], lba);
    let w = diff_bytes(&rbuf[..n], &wbuf[..n]);
    if w != 0 {
        sec.wrong += w;
        fail_case(rep, card, sec, case, "wrong bytes in known data", None).await;
        return false;
    }
    true
}

/// Rewrites `n` blocks at WORK with the known pattern, reads back one block before and one after
/// as well, and counts wrong bytes inside and damage outside.
async fn rewrite_verify(
    rep: &mut Report,
    card: &mut Sd,
    sec: &mut Sec,
    case: Case,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    n: usize,
) -> bool {
    fill(&mut wbuf[..n], WORK);
    match bounded(3000, card.write_blocks(WORK, &wbuf[..n])).await {
        Some(Ok(())) => {}
        Some(Err(e)) => {
            fail_case(rep, card, sec, case, "rewrite failed", Some(e)).await;
            return false;
        }
        None => {
            fail_case(rep, card, sec, case, "rewrite hung", None).await;
            return false;
        }
    }
    rbuf[..n + 2].fill(Block::new());
    match bounded(3000, card.read_blocks(WORK - 1, &mut rbuf[..n + 2])).await {
        Some(Ok(())) => {}
        Some(Err(e)) => {
            fail_case(rep, card, sec, case, "verify read failed", Some(e)).await;
            return false;
        }
        None => {
            fail_case(rep, card, sec, case, "verify read hung", None).await;
            return false;
        }
    }
    fill(&mut wbuf[..n + 2], WORK - 1);
    let inside = diff_bytes(&rbuf[1..n + 1], &wbuf[1..n + 1]);
    let outside =
        diff_bytes(&rbuf[..1], &wbuf[..1]) + diff_bytes(&rbuf[n + 1..n + 2], &wbuf[n + 1..n + 2]);
    sec.wrong += inside;
    sec.damage += outside;
    if inside != 0 || outside != 0 {
        defmt::error!(
            "[INFO] after the rewrite: {} wrong bytes inside the range, {} outside",
            inside,
            outside
        );
        fail_case(rep, card, sec, case, "wrong data after rewrite", None).await;
        return false;
    }
    true
}

async fn identify(rep: &mut Report, card: &mut Sd, place: &str) -> Option<CardInfo> {
    let Some(init) = bounded(INIT_BOUND_MS, card.init()).await else {
        check!(
            rep,
            false,
            "{=str} {=str} init hung (no answer within 10 s)",
            place
        );
        return None;
    };
    if !check!(rep, init.is_ok(), "{=str} init succeeds",) {
        if let Err(e) = init {
            defmt::error!("[INFO] init error: {}", e);
        }
        return None;
    }
    let info = card.info().copied()?;
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
    Some(info)
}

/// Writes the whole test region with the known pattern and reads it back.
async fn region_roundtrip(
    rep: &mut Report,
    card: &mut Sd,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    what: &str,
) {
    fill(wbuf, SCRATCH);
    let w = bounded(INIT_BOUND_MS, card.write_blocks(SCRATCH, wbuf)).await;
    let werr = !matches!(w, Some(Ok(())));
    rbuf.fill(Block::new());
    let r = bounded(INIT_BOUND_MS, card.read_blocks(SCRATCH, rbuf)).await;
    let rerr = !matches!(r, Some(Ok(())));
    if w.is_none() || r.is_none() {
        check!(
            rep,
            false,
            "{=str} {=str} region transfer hung (no answer within 10 s)",
            what
        );
    }
    let differing = wbuf.iter().zip(rbuf.iter()).filter(|(a, b)| a != b).count() as u32;
    check!(
        rep,
        !werr && !rerr && differing == 0,
        "{=str} {=str} write/read/verify of {} blocks: write error {}, read error {}, {} differing blocks",
        what,
        REGION,
        werr,
        rerr,
        differing
    );
}

async fn x0(
    rep: &mut Report,
    card: &mut Sd,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
) -> Option<CardInfo> {
    let info = identify(rep, card, "X0").await?;
    region_roundtrip(rep, card, wbuf, rbuf, "X0").await;
    Some(info)
}

/// Sweeps drop delays over one read size.
/// How a future is dropped: after a delay, or right after its k-th suspension (every await
/// point of the operation can be hit exactly this way, which a timer cannot promise).
#[derive(Clone, Copy)]
enum Cut {
    Time(u32),
    Polls(u32),
}

/// Runs `f` until it is dropped by `cut`; `None` if it was dropped.
async fn run_cut<F: Future>(cut: Cut, f: F) -> Option<F::Output> {
    match cut {
        Cut::Time(us) => with_timeout(Duration::from_micros(u64::from(us)), f)
            .await
            .ok(),
        Cut::Polls(limit) => {
            let mut f = pin!(f);
            let mut polls = 0u32;
            poll_fn(|cx| {
                polls += 1;
                match f.as_mut().poll(cx) {
                    Poll::Ready(v) => Poll::Ready(Some(v)),
                    Poll::Pending if polls >= limit => Poll::Ready(None),
                    Poll::Pending => Poll::Pending,
                }
            })
            .await
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn x1_sweep(
    rep: &mut Report,
    card: &mut Sd,
    sec: &mut Sec,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    tag: &'static str,
    n: usize,
    polls: bool,
    end: u32,
    step: u32,
) -> bool {
    let lba = SCRATCH + 16;
    let mut d = if polls { 1 } else { 0 };
    while d <= end {
        let cut = if polls { Cut::Polls(d) } else { Cut::Time(d) };
        let case = Case {
            sec: tag,
            op: "read_blocks",
            n: n as u32,
            d,
        };
        card.bus().begin_call();
        let r = run_cut(cut, card.read_blocks(lba, &mut rbuf[..n])).await;
        let done = r.is_some();
        match r {
            None => {
                let p = card.bus().classify();
                sec.cancels += 1;
                sec.phase[p] += 1;
                if !after_cancel(rep, card, sec, case, wbuf, rbuf, n, lba, lba).await {
                    return false;
                }
            }
            Some(Ok(())) => sec.completed += 1,
            Some(Err(e)) => {
                fail_case(rep, card, sec, case, "uncancelled read failed", Some(e)).await;
                return false;
            }
        }
        if !read_known(rep, card, sec, case, wbuf, rbuf, lba, n).await {
            return false;
        }
        if polls && done {
            break;
        }
        d += step;
    }
    true
}

async fn x1(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) -> bool {
    let mut st = Sec::default();
    let mut sp = Sec::default();
    let mut ok = true;
    for (n, end, step) in [(1usize, 1500u32, 7u32), (8, 1400, 7), (64, 4500, 22)] {
        let before = st.cancels;
        ok = x1_sweep(
            rep, card, &mut st, wbuf, rbuf, "X1 time", n, false, end, step,
        )
        .await;
        defmt::info!(
            "[INFO] X1 timed sweep n={} 0..{} us step {}: {} cancels",
            n,
            end,
            step,
            st.cancels - before
        );
        if !ok {
            break;
        }
    }
    if ok {
        for n in [1usize, 2, 8, 64] {
            let before = sp.cancels;
            ok = x1_sweep(rep, card, &mut sp, wbuf, rbuf, "X1 polls", n, true, 2000, 1).await;
            defmt::info!(
                "[INFO] X1 every-suspension sweep n={}: {} cancels, then the read ran to the end",
                n,
                sp.cancels - before
            );
            if !ok {
                break;
            }
        }
    }
    print_sec("X1 time", &st);
    print_sec("X1 polls", &sp);
    let cancels = st.cancels + sp.cancels;
    let failed = st.failed + sp.failed;
    let wrong = st.wrong + sp.wrong;
    let (nr, nw) = (st.next[1] + sp.next[1], st.next[2] + sp.next[2]);
    let runs = st.recoveries + sp.recoveries;
    let stops = st.stops + sp.stops;
    check!(
        rep,
        ok && cancels >= 200
            && failed == 0
            && wrong == 0
            && nr >= 60
            && nw >= 60
            && runs >= 300
            && stops >= 250,
        "{=str} X1 cancelled reads: {} cancels, {} failed next operations, {} wrong bytes; first operation after a cancel: {} status, {} read (min 60), {} write (min 60); clean-ups run {} (min 300), needing CMD12 {} (min 250)",
        cancels,
        failed,
        wrong,
        st.next[0] + sp.next[0],
        nr,
        nw,
        runs,
        stops
    );
    ok
}

#[allow(clippy::too_many_arguments)]
async fn x2_sweep(
    rep: &mut Report,
    card: &mut Sd,
    sec: &mut Sec,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    tag: &'static str,
    n: usize,
    polls: bool,
    end: u32,
    step: u32,
) -> bool {
    let mut d = if polls { 1 } else { 0 };
    while d <= end {
        let cut = if polls { Cut::Polls(d) } else { Cut::Time(d) };
        let case = Case {
            sec: tag,
            op: "write_blocks",
            n: n as u32,
            d,
        };
        fill(&mut wbuf[..n], WORK.wrapping_add(JUNK));
        card.bus().begin_call();
        let r = run_cut(cut, card.write_blocks(WORK, &wbuf[..n])).await;
        let done = r.is_some();
        match r {
            None => {
                let p = card.bus().classify();
                sec.cancels += 1;
                sec.phase[p] += 1;
                if !after_cancel(rep, card, sec, case, wbuf, rbuf, n, SCRATCH + 100, WORK).await {
                    return false;
                }
            }
            Some(Ok(())) => sec.completed += 1,
            Some(Err(e)) => {
                fail_case(rep, card, sec, case, "uncancelled write failed", Some(e)).await;
                return false;
            }
        }
        if !rewrite_verify(rep, card, sec, case, wbuf, rbuf, n).await {
            return false;
        }
        if polls && done {
            break;
        }
        d += step;
    }
    true
}

async fn x2(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) -> bool {
    let mut st = Sec::default();
    let mut sp = Sec::default();
    let mut ok = true;
    for (n, end, step) in [(1usize, 3000u32, 15u32), (8, 3500, 16), (64, 7000, 35)] {
        let before = st.cancels;
        ok = x2_sweep(
            rep, card, &mut st, wbuf, rbuf, "X2 time", n, false, end, step,
        )
        .await;
        defmt::info!(
            "[INFO] X2 timed sweep n={} 0..{} us step {}: {} cancels",
            n,
            end,
            step,
            st.cancels - before
        );
        if !ok {
            break;
        }
    }
    if ok {
        for n in [1usize, 2, 8, 64] {
            let before = sp.cancels;
            ok = x2_sweep(rep, card, &mut sp, wbuf, rbuf, "X2 polls", n, true, 2000, 1).await;
            defmt::info!(
                "[INFO] X2 every-suspension sweep n={}: {} cancels, then the write ran to the end",
                n,
                sp.cancels - before
            );
            if !ok {
                break;
            }
        }
    }
    print_sec("X2 time", &st);
    print_sec("X2 polls", &sp);
    let cancels = st.cancels + sp.cancels;
    let failed = st.failed + sp.failed;
    let wrong = st.wrong + sp.wrong;
    let damage = st.damage + sp.damage;
    let (nr, nw) = (st.next[1] + sp.next[1], st.next[2] + sp.next[2]);
    let runs = st.recoveries + sp.recoveries;
    let stops = st.stops + sp.stops;
    check!(
        rep,
        ok && cancels >= 200
            && failed == 0
            && wrong == 0
            && damage == 0
            && nr >= 60
            && nw >= 60
            && runs >= 300
            && stops >= 150,
        "{=str} X2 cancelled writes: {} cancels, {} failed next operations, {} wrong bytes after rewrite, {} damaged bytes outside the range; first operation after a cancel: {} status, {} read (min 60), {} write (min 60); clean-ups run {} (min 300), needing CMD12 {} (min 150)",
        cancels,
        failed,
        wrong,
        damage,
        st.next[0] + sp.next[0],
        nr,
        nw,
        runs,
        stops
    );
    ok
}

/// Tries to cancel reads and writes inside a bus command, which a drop at an await point has not
/// reached (those futures were never Pending there). No busy wait is tried here: none occurs on
/// the normal route. The wrapper aborts the named
/// bus operation after `us` polls and parks the caller's future, which the
/// test then drops: the real `PioBus` operation is dropped part-way, as in any other cancel.
async fn x_inbus(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) -> bool {
    let mut sr = Sec::default();
    let mut sw = Sec::default();
    let mut ok = true;
    // (write, blocks, kind, nth operation of that kind in the call, last delay, step)
    let plan = [
        (false, 1usize, Kind::Cmd, 1u32, 6u32, 1u32),
        (false, 1, Kind::Cmd, 2, 6, 1),
        (false, 8, Kind::Cmd, 1, 6, 1),
        (false, 8, Kind::Cmd, 2, 6, 1),
        (false, 8, Kind::Cmd, 3, 6, 1),
        (true, 1, Kind::Cmd, 1, 6, 1),
        (true, 1, Kind::Cmd, 2, 6, 1),
        (true, 8, Kind::Cmd, 1, 6, 1),
        (true, 8, Kind::Cmd, 2, 6, 1),
        (true, 8, Kind::Cmd, 3, 6, 1),
    ];
    'plan: for (write, n, kind, nth, last, step) in plan {
        let mut landed = 0u32;
        let mut us = 0;
        while us <= last {
            let case = Case {
                sec: if write { "X2c" } else { "X1c" },
                op: if write { "write_blocks" } else { "read_blocks" },
                n: n as u32,
                d: us,
            };
            let sec = if write { &mut sw } else { &mut sr };
            card.bus().begin_call();
            card.bus().cut_in = Some((kind, nth, us));
            let r = if write {
                fill(&mut wbuf[..n], WORK.wrapping_add(JUNK));
                run_cut(Cut::Time(30_000), card.write_blocks(WORK, &wbuf[..n])).await
            } else {
                run_cut(
                    Cut::Time(30_000),
                    card.read_blocks(SCRATCH + 16, &mut rbuf[..n]),
                )
                .await
            };
            let b = card.bus();
            b.cut_in = None;
            let (stalled, p) = (b.stalled, b.classify());
            match r {
                None if stalled == 1 => {
                    landed += 1;
                    sec.cancels += 1;
                    sec.phase[p] += 1;
                    if !recover(rep, card, sec, case).await {
                        ok = false;
                        break 'plan;
                    }
                }
                None => {
                    fail_case(
                        rep,
                        card,
                        sec,
                        case,
                        "future hung without the wrapper cut",
                        None,
                    )
                    .await;
                    ok = false;
                    break 'plan;
                }
                Some(Ok(())) => sec.completed += 1,
                Some(Err(e)) => {
                    fail_case(rep, card, sec, case, "uncut operation failed", Some(e)).await;
                    ok = false;
                    break 'plan;
                }
            }
            let good = if write {
                rewrite_verify(rep, card, sec, case, wbuf, rbuf, n).await
            } else {
                read_known(rep, card, sec, case, wbuf, rbuf, SCRATCH + 16, n).await
            };
            if !good {
                ok = false;
                break 'plan;
            }
            us += step;
        }
        defmt::info!(
            "[INFO] X{=str}c cut in bus op {} #{} of an n={} {=str}, polls 0..{}: {} landed",
            if write { "2" } else { "1" },
            if kind == Kind::Cmd {
                "command"
            } else {
                "busy wait (never reached: no busy wait occurs on this route)"
            },
            nth,
            n,
            if write { "write" } else { "read" },
            last,
            landed
        );
    }
    print_sec("X1c in-command cuts (reads)", &sr);
    print_sec("X2c in-command cuts (writes)", &sw);
    defmt::info!(
        "[INFO] X1c/X2c cuts inside a bus command: {} read cancels and {} write cancels landed of 70 tries (no command future was Pending on the polls tried, so no drop was seen to land inside a command; busy waits are not exercised by this section at all); {} failed next operations, {} wrong bytes, {} damaged bytes outside the range",
        sr.cancels,
        sw.cancels,
        sr.failed + sw.failed,
        sr.wrong + sw.wrong,
        sw.damage
    );
    ok
}

/// Starts a write with the command after CMD55 held back and drops it there. True if the
/// wrapper saw exactly CMD55 answered, then ACMD23(n) held back and never sent.
async fn cut_pair_write(card: &mut Sd, wbuf: &mut [Block], n: usize) -> bool {
    fill(&mut wbuf[..n], WORK.wrapping_add(JUNK));
    let b = card.bus();
    b.begin_call();
    b.stall_acmd = true;
    let r = with_timeout(
        Duration::from_millis(20),
        card.write_blocks(WORK, &wbuf[..n]),
    )
    .await;
    let b = card.bus();
    b.stall_acmd = false;
    r.is_err()
        && b.stalled == 1
        && b.cur.is_none()
        && b.log_n == 2
        && b.log[0].0 == 55
        && b.log[1] == (23, n as u32)
}

async fn x3(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) -> bool {
    card.bus().tamper_scr = true;
    let init = bounded(INIT_BOUND_MS, card.init()).await;
    card.bus().tamper_scr = false;
    if init.is_none() {
        defmt::error!("[INFO] X3 init hung (no answer within 10 s)");
    }
    let mode = matches!(&init, Some(Ok(i)) if !i.uses_cmd23);
    if !check!(
        rep,
        mode,
        "{=str} X3 init with the SCR CMD23 bit cleared in transit: driver now takes the ACMD23 route"
    ) {
        return false;
    }
    let mut sec = Sec::default();
    let mut cut_ok = 0u32;
    let mut ok = true;
    'outer: for n in [2usize, 8, 64] {
        for i in 0..14u32 {
            let case = Case {
                sec: "X3",
                op: "write_blocks cut after CMD55",
                n: n as u32,
                d: i,
            };
            if !cut_pair_write(card, wbuf, n).await {
                fail_case(
                    rep,
                    card,
                    &mut sec,
                    case,
                    "the pair was not cut as designed",
                    None,
                )
                .await;
                ok = false;
                break 'outer;
            }
            cut_ok += 1;
            sec.cancels += 1;
            sec.phase[4] += 1;
            if !after_cancel(
                rep,
                card,
                &mut sec,
                case,
                wbuf,
                rbuf,
                n,
                SCRATCH + 100,
                WORK,
            )
            .await
                || !rewrite_verify(rep, card, &mut sec, case, wbuf, rbuf, n).await
            {
                ok = false;
                break 'outer;
            }
        }
    }
    let cut_main = cut_ok;
    // The clean-up's own pair cut in two: the mark must stay and the next call must try again.
    let mut recut = 0u32;
    if ok {
        'inner: for i in 0..10u32 {
            let case = Case {
                sec: "X3b",
                op: "status, its clean-up pair cut",
                n: 8,
                d: i,
            };
            if !cut_pair_write(card, wbuf, 8).await {
                fail_case(rep, card, &mut sec, case, "the first cut failed", None).await;
                ok = false;
                break 'inner;
            }
            let b = card.bus();
            b.begin_call();
            b.stall_acmd = true;
            let r = with_timeout(Duration::from_millis(20), card.status()).await;
            let b = card.bus();
            b.stall_acmd = false;
            let recut_ok = r.is_err() && b.stalled == 1 && b.neutral_pair_first() && b.log_n == 2;
            if !recut_ok {
                fail_case(
                    rep,
                    card,
                    &mut sec,
                    case,
                    "the clean-up pair was not cut as designed",
                    None,
                )
                .await;
                ok = false;
                break 'inner;
            }
            recut += 1;
            if !after_cancel(
                rep,
                card,
                &mut sec,
                case,
                wbuf,
                rbuf,
                8,
                SCRATCH + 100,
                WORK,
            )
            .await
                || !rewrite_verify(rep, card, &mut sec, case, wbuf, rbuf, 8).await
            {
                ok = false;
                break 'inner;
            }
        }
    }
    let mut s3c = Sec::default();
    if ok {
        for n in [2usize, 8] {
            let before = s3c.cancels;
            ok = x2_sweep(
                rep,
                card,
                &mut s3c,
                wbuf,
                rbuf,
                "X3c polls",
                n,
                true,
                2000,
                1,
            )
            .await;
            defmt::info!(
                "[INFO] X3c every-suspension sweep of the ACMD23 route n={}: {} cancels",
                n,
                s3c.cancels - before
            );
            if !ok {
                break;
            }
        }
    }
    print_sec("X3", &sec);
    print_sec("X3c polls", &s3c);
    defmt::info!(
        "[STAT] X3: pair cut between CMD55 and ACMD23 by the bus wrapper {} times (write n=2/8/64), plus the clean-up's own pair cut {} times",
        cut_main,
        recut
    );
    check!(
        rep,
        ok && cut_main >= 20
            && sec.recoveries >= 20
            && sec.pairs >= 20
            && sec.pairs == sec.recoveries
            && sec.next[1] >= 10
            && sec.next[2] >= 10
            && sec.failed == 0
            && sec.wrong == 0
            && sec.damage == 0
            && s3c.failed == 0
            && s3c.wrong == 0
            && s3c.damage == 0,
        "{=str} X3 interrupted application command: {} deliberate cuts, {} clean-ups began with CMD55+ACMD23(1) of {} run (min 20 each), first operation after the cut: {} status, {} read, {} write (min 10 each), {} failed next operations, {} wrong bytes; plus {} every-suspension cancels ({} clean-ups began with the pair), {} failed, {} wrong bytes",
        cut_main + recut,
        sec.pairs,
        sec.recoveries,
        sec.next[0],
        sec.next[1],
        sec.next[2],
        sec.failed,
        sec.wrong,
        s3c.cancels,
        s3c.pairs,
        s3c.failed,
        s3c.wrong
    );
    ok
}

/// X4, application-command pair: after a cut, one CMD13 straight on the bus must be taken by
/// the real card as ACMD13 (R1 with APP_CMD set), which a normal CMD13 never shows.
async fn x4_app(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) -> bool {
    let mut seen = 0u32;
    let mut normal = 0u32;
    let mut sec = Sec::default();
    let mut recover_bad = 0u32;
    for i in 0..6u32 {
        let case = Case {
            sec: "X4a",
            op: "write cut after CMD55, raw CMD13",
            n: 8,
            d: i,
        };
        if !cut_pair_write(card, wbuf, 8).await {
            fail_case(
                rep,
                card,
                &mut sec,
                case,
                "the pair was not cut as designed",
                None,
            )
            .await;
            return false;
        }
        let s = raw_status(card).await;
        if let Some(s) = s {
            defmt::info!(
                "[INFO] X4a raw CMD13 after the cut: status {=u32:#x}, state {=u32}, app_cmd bit {=bool}",
                s.0,
                (s.0 >> 9) & 0xF,
                s.app_cmd()
            );
            if s.app_cmd() {
                seen += 1;
                let mut sink = [0u32; 16];
                let _ = bounded(200, card.bus().read_block(&mut sink, 100)).await;
            }
        }
        // The raw probe disturbed the card on purpose; the clean-up after it is reported, not judged.
        card.bus().begin_call();
        let r = bounded(3000, card.status()).await;
        let good = matches!(r, Some(Ok(s)) if s.state() == CardState::Tran && s.ready_for_data());
        if !good {
            recover_bad += 1;
            defmt::info!(
                "[INFO] X4a clean-up after the raw probe did not leave the card ready; re-initialising"
            );
            if !matches!(bounded(INIT_BOUND_MS, card.init()).await, Some(Ok(_))) {
                check!(
                    rep,
                    false,
                    "{=str} X4a card does not answer a fresh init after the probe"
                );
                return false;
            }
        }
        // A plain CMD13 on a card that has not been cut, for comparison.
        if let Some(s) = raw_status(card).await {
            if s.app_cmd() {
                normal += 1;
            }
        }
        if !rewrite_verify(rep, card, &mut sec, case, wbuf, rbuf, 8).await {
            return false;
        }
    }
    defmt::info!(
        "[STAT] X4a: raw CMD13 after the cut showed APP_CMD {} of 6; plain CMD13 without a cut showed APP_CMD {} of 6; clean-up after the probe not ready {}",
        seen,
        normal,
        recover_bad
    );
    check!(
        rep,
        seen >= 1 && normal == 0,
        "{=str} X4a positive control, application command: the card was left expecting an application command {} times (plain CMD13 {} times)",
        seen,
        normal
    );
    true
}

/// X4, data phase: cancelled multi-block write / read, then one CMD13 straight on the bus with no
/// clean-up; the card must be seen not ready or in a data state.
async fn x4_data(rep: &mut Report, card: &mut Sd, wbuf: &mut [Block], rbuf: &mut [Block]) -> bool {
    let mut sec = Sec::default();
    let mut states = [0u32; 9];
    let mut not_ready = 0u32;
    let mut probes = 0u32;
    for (write, n, first, step) in [(true, 8usize, 400u32, 55u32), (false, 64, 500, 150)] {
        for i in 0..20u32 {
            let d = first + i * step;
            let case = Case {
                sec: "X4b",
                op: if write {
                    "write cancel, raw CMD13"
                } else {
                    "read cancel, raw CMD13"
                },
                n: n as u32,
                d,
            };
            fill(&mut wbuf[..n], WORK.wrapping_add(JUNK));
            card.bus().begin_call();
            let landed = if write {
                with_timeout(
                    Duration::from_micros(u64::from(d)),
                    card.write_blocks(WORK, &wbuf[..n]),
                )
                .await
                .is_err()
            } else {
                with_timeout(
                    Duration::from_micros(u64::from(d)),
                    card.read_blocks(WORK, &mut rbuf[..n]),
                )
                .await
                .is_err()
            };
            if landed {
                sec.cancels += 1;
                sec.phase[card.bus().classify()] += 1;
                match raw_status(card).await {
                    Some(s) => {
                        probes += 1;
                        let st = (s.0 >> 9) & 0xF;
                        if let Some(c) = states.get_mut(st as usize) {
                            *c += 1;
                        }
                        if s.state() != CardState::Tran || !s.ready_for_data() {
                            not_ready += 1;
                        }
                    }
                    None => {
                        fail_case(
                            rep,
                            card,
                            &mut sec,
                            case,
                            "no answer to the raw CMD13",
                            None,
                        )
                        .await;
                        return false;
                    }
                }
                if !recover(rep, card, &mut sec, case).await {
                    return false;
                }
            } else {
                sec.completed += 1;
            }
            if !rewrite_verify(rep, card, &mut sec, case, wbuf, rbuf, n.min(64)).await {
                return false;
            }
        }
    }
    print_sec("X4b", &sec);
    defmt::info!(
        "[STAT] X4b: raw CMD13 with no clean-up: {} probes, card not in Tran/ready {}; by state: stby {} tran {} data {} rcv {} prg {} (other {})",
        probes,
        not_ready,
        states[3],
        states[4],
        states[5],
        states[6],
        states[7],
        states[0] + states[1] + states[2] + states[8]
    );
    check!(
        rep,
        not_ready >= 1 && sec.failed == 0,
        "{=str} X4b positive control, data phase: {} of {} raw status reads after a cancel found the card not ready (data, receive or programming)",
        not_ready,
        probes
    );
    true
}

async fn x5(
    rep: &mut Report,
    card: &mut Sd,
    wbuf: &mut [Block],
    rbuf: &mut [Block],
    first: &CardInfo,
) -> bool {
    card.bus().tamper_scr = false;
    let Some(info) = identify(rep, card, "X5").await else {
        return false;
    };
    check!(
        rep,
        info.cid == first.cid
            && info.capacity_blocks == first.capacity_blocks
            && info.csd == first.csd,
        "{=str} X5 card identity (CID, CSD, capacity) unchanged since X0"
    );
    check!(
        rep,
        info.uses_cmd23 && info.clock_hz == 25_000_000,
        "{=str} X5 CMD23 {} and data clock {} Hz as in X0",
        info.uses_cmd23,
        info.clock_hz
    );
    region_roundtrip(rep, card, wbuf, rbuf, "X5").await;
    let t0 = Instant::now();
    let r = bounded(3000, card.read_blocks(SCRATCH, &mut rbuf[..256])).await;
    let us = us_since(t0);
    check!(rep, matches!(r, Some(Ok(()))), "{=str} X5 256-block read");
    defmt::info!(
        "[STAT] X5 read of 256 blocks in one call: {=f32} MB/s ({} us); sd_smoke reference 9.1 MB/s",
        (256 * 512) as f32 / us as f32,
        us
    );
    let raw = raw_status(card).await;
    let healthy = matches!(raw, Some(s) if s.state() == CardState::Tran && s.ready_for_data() && s.errors() == 0);
    let st = *card.stats();
    defmt::info!(
        "[INFO] health: commands {} blocks read {} written {} crc errors {} write rejections {} timeouts {} bad responses {} recoveries {}",
        st.commands_sent,
        st.blocks_read,
        st.blocks_written,
        st.read_crc_errors,
        st.write_rejections,
        st.timeouts,
        st.bad_responses,
        st.recoveries
    );
    check!(
        rep,
        healthy,
        "{=str} X5 health: card in the transfer state, ready, no error bits"
    );
    true
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut rep = Report { failures: 0 };
    defmt::info!(
        "[INFO] sd_faults: clk_sys = {} Hz",
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
    let mut card: Sd = SdCard::new(FaultBus::new(bus), Config::default());
    let wbuf = WRITE_BUF.init([Block::new(); REGION]);
    let rbuf = READ_BUF.init([Block::new(); REGION]);

    if let Some(first) = x0(&mut rep, &mut card, wbuf, rbuf).await {
        let mut ok = x1(&mut rep, &mut card, wbuf, rbuf).await;
        if ok {
            ok = x2(&mut rep, &mut card, wbuf, rbuf).await;
        }
        if ok {
            ok = x_inbus(&mut rep, &mut card, wbuf, rbuf).await;
        }
        if ok {
            ok = x3(&mut rep, &mut card, wbuf, rbuf).await;
        }
        if ok {
            ok = x4_app(&mut rep, &mut card, wbuf, rbuf).await;
        }
        if ok {
            card.bus().tamper_scr = false;
            ok = matches!(bounded(INIT_BOUND_MS, card.init()).await, Some(Ok(_)));
            check!(rep, ok, "{=str} X4 re-init in normal mode");
        }
        if ok {
            ok = x4_data(&mut rep, &mut card, wbuf, rbuf).await;
        }
        if ok {
            x5(&mut rep, &mut card, wbuf, rbuf, &first).await;
        } else {
            card.bus().tamper_scr = false;
            card.bus().stall_acmd = false;
            let alive = matches!(bounded(INIT_BOUND_MS, card.init()).await, Some(Ok(_)));
            defmt::error!(
                "[INFO] stopped after the first failure; a fresh init afterwards: {=bool}",
                alive
            );
        }
    }
    hil_common::exit(rep.failures)
}
