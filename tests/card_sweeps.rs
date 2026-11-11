//! Sweeps over the block operations on the simulated card: a bus failure at every command or
//! data phase in turn, and the operation's future dropped at every await point in turn.
//!
//! `Probe` wraps the simulator and numbers every bus call. The driver's own awaits all wait on
//! bus calls (its `overlap` helper polls the bus future at most once more), so a Pending inserted
//! before or after each bus call visits every await point of the driver.

mod card_helpers;

use std::future::{Future, poll_fn};
use std::pin::pin;
use std::task::{Context, Poll, Waker};

use card_helpers::*;
use embassy_futures::block_on;
use pio_sdmmc::card::{Block, Config, Error, SdCard};
use pio_sdmmc::sim::{CardKind, Fault, SimCard, TraceEvent, Trigger};
use pio_sdmmc::{Bus, BusError, ResponseBuf};

const LBA: u32 = 40;

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Counts {
    /// Every bus await.
    awaits: usize,
    /// Calls that can fail: commands, block reads and writes, ready waits.
    fallible: usize,
    commands: usize,
    reads: usize,
    writes: usize,
    waits: usize,
}

struct Probe {
    sim: SimCard,
    n: Counts,
    /// Fail the `.0`-th fallible call (1-based); before it reaches the card, or (`.1`) after the
    /// card has done it.
    fail: Option<(usize, bool, BusError)>,
    /// Return Pending once at the `.0`-th await (1-based), before or (`.1`) after the card has
    /// done the operation.
    pend: Option<(usize, bool)>,
}

/// Returns Pending `times` times, then Ready. The data phases need two: the driver's `overlap`
/// polls the bus future once by itself and polls it again at once, so only the second Pending
/// reaches the caller.
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

impl Probe {
    fn new(cmd23: bool) -> Self {
        Probe {
            sim: SimCard::new(sim_cfg(CardKind::Sdxc, cmd23)),
            n: Counts::default(),
            fail: None,
            pend: None,
        }
    }

    fn tick(&mut self, fallible: bool) -> (usize, usize) {
        self.n.awaits += 1;
        if fallible {
            self.n.fallible += 1;
        }
        (self.n.awaits, self.n.fallible)
    }

    fn fail_at(&self, f: usize, after: bool) -> Option<BusError> {
        match self.fail {
            Some((k, a, e)) if k == f && a == after => Some(e),
            _ => None,
        }
    }
}

macro_rules! gated {
    ($self:ident, $times:expr, $call:expr) => {{
        let (a, f) = $self.tick(true);
        if $self.pend == Some((a, false)) {
            stall($times).await;
        }
        if let Some(e) = $self.fail_at(f, false) {
            return Err(e);
        }
        let out = $call;
        if $self.pend == Some((a, true)) {
            stall($times).await;
        }
        if let Some(e) = $self.fail_at(f, true) {
            return Err(e);
        }
        out
    }};
}

impl Bus for Probe {
    fn set_clock(&mut self, max_hz: u32) -> u32 {
        self.sim.set_clock(max_hz)
    }

    async fn clock_idle(&mut self, cycles: u32) {
        let (a, _) = self.tick(false);
        if self.pend == Some((a, false)) {
            stall(1).await;
        }
        self.sim.clock_idle(cycles).await;
        if self.pend == Some((a, true)) {
            stall(1).await;
        }
    }

    async fn command(
        &mut self,
        frame: &[u8; 6],
        response: ResponseBuf<'_>,
    ) -> Result<(), BusError> {
        self.n.commands += 1;
        gated!(self, 1, self.sim.command(frame, response).await)
    }

    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError> {
        self.n.reads += 1;
        gated!(self, 2, self.sim.read_block(data, timeout_ms).await)
    }

    async fn write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError> {
        self.n.writes += 1;
        gated!(self, 2, self.sim.write_block(data, crc, timeout_ms).await)
    }

    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError> {
        self.n.waits += 1;
        gated!(self, 1, self.sim.wait_ready(timeout_ms).await)
    }

    async fn delay_ms(&mut self, ms: u32) {
        let (a, _) = self.tick(false);
        if self.pend == Some((a, false)) {
            stall(1).await;
        }
        self.sim.delay_ms(ms).await;
        if self.pend == Some((a, true)) {
            stall(1).await;
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum Op {
    Read1,
    ReadN,
    Write1,
    WriteN,
}

const OPS: [Op; 4] = [Op::Read1, Op::ReadN, Op::Write1, Op::WriteN];

impl Op {
    fn blocks(self) -> usize {
        match self {
            Op::Read1 | Op::Write1 => 1,
            Op::ReadN | Op::WriteN => 4,
        }
    }

    fn is_write(self) -> bool {
        matches!(self, Op::Write1 | Op::WriteN)
    }
}

fn old(i: usize) -> [u8; 512] {
    pattern(100 + i as u8)
}

fn new(i: usize) -> [u8; 512] {
    pattern(1 + i as u8)
}

/// A card that is initialised, holds the old content and has fresh counters.
fn setup(cmd23: bool, op: Op) -> SdCard<Probe> {
    let mut card = SdCard::new(Probe::new(cmd23), Config::default());
    block_on(card.init()).expect("init");
    for i in 0..op.blocks() {
        let content = if op.is_write() { old(i) } else { new(i) };
        card.bus().sim.poke_block(LBA + i as u32, &content);
    }
    assert_eq!(card.bus().sim.violations(), &[]);
    card.bus().n = Counts::default();
    card
}

async fn run(card: &mut SdCard<Probe>, op: Op, out: &mut [Block]) -> Result<(), Error> {
    if op.is_write() {
        let data: Vec<Block> = (0..op.blocks()).map(|i| new(i).into()).collect();
        card.write_blocks(LBA, &data).await
    } else {
        card.read_blocks(LBA, out).await
    }
}

fn blank(op: Op) -> Vec<Block> {
    vec![Block::new(); op.blocks()]
}

/// The operation's full effect is visible, and the card broke no rule.
fn check_done(card: &mut SdCard<Probe>, op: Op, out: &[Block]) -> Result<(), String> {
    for (i, block) in out.iter().enumerate().take(op.blocks()) {
        let got = if op.is_write() {
            card.bus().sim.peek_block(LBA + i as u32)
        } else {
            block.as_bytes().try_into().unwrap()
        };
        if got != new(i) {
            return Err(format!("block {i} wrong"));
        }
    }
    if !card.bus().sim.violations().is_empty() {
        return Err(format!("violations {:?}", card.bus().sim.violations()));
    }
    Ok(())
}

/// After an interrupted write: every block is wholly old or wholly new, and every block the
/// driver reported as written is new.
fn check_not_mixed(card: &mut SdCard<Probe>, op: Op) -> Result<(), String> {
    if !op.is_write() {
        return Ok(());
    }
    let reported = card.stats().blocks_written as usize;
    for i in 0..op.blocks() {
        let b = card.bus().sim.peek_block(LBA + i as u32);
        if b != old(i) && b != new(i) {
            return Err(format!("block {i} is neither the old nor the new content"));
        }
        if i < reported && b != new(i) {
            return Err(format!("block {i} reported as written but is old"));
        }
    }
    Ok(())
}

fn label(op: Op, cmd23: bool) -> String {
    format!("{op:?} cmd23={cmd23}")
}

fn clean_counts(cmd23: bool, op: Op) -> Counts {
    let mut card = setup(cmd23, op);
    let mut out = blank(op);
    block_on(run(&mut card, op, &mut out)).unwrap();
    check_done(&mut card, op, &out).unwrap();
    card.bus().n
}

/// A bus failure at the `k`-th fallible call, then the next operation. Returns what went wrong.
fn bus_case(cmd23: bool, op: Op, k: usize, after: bool, err: BusError) -> Vec<String> {
    let mut findings = Vec::new();
    let at = format!("{} fail {k} after={after} {err:?}", label(op, cmd23));
    let mut card = setup(cmd23, op);
    card.bus().fail = Some((k, after, err));
    let mut out = blank(op);
    match block_on(run(&mut card, op, &mut out)) {
        Err(Error::Bus { error, .. }) if error == err => {}
        other => findings.push(format!("{at}: result {other:?}")),
    }
    card.bus().fail = None;
    if let Err(m) = check_not_mixed(&mut card, op) {
        findings.push(format!("{at}: {m}"));
    }
    if !card.bus().sim.violations().is_empty() {
        findings.push(format!(
            "{at}: violations after the failure {:?}",
            card.bus().sim.violations()
        ));
        card.bus().sim.take_violations();
    }
    // The documented recovery is inside the next call (and `init` if it reports
    // UnexpectedState).
    let mut out = blank(op);
    let mut r = block_on(run(&mut card, op, &mut out));
    if matches!(r, Err(Error::UnexpectedState { .. })) {
        findings.push(format!("{at}: needed init(): {r:?}"));
        block_on(card.init()).unwrap();
        r = block_on(run(&mut card, op, &mut out));
    }
    if let Err(e) = r {
        findings.push(format!("{at}: next operation failed: {e}"));
    } else if let Err(m) = check_done(&mut card, op, &out) {
        findings.push(format!("{at}: next operation: {m}"));
    }
    findings
}

/// The future dropped at the `k`-th await, then the same operation again.
fn drop_case(cmd23: bool, op: Op, k: usize, after: bool) -> Vec<String> {
    let mut findings = Vec::new();
    let at = format!("{} drop at await {k} after={after}", label(op, cmd23));
    let mut card = setup(cmd23, op);
    card.bus().pend = Some((k, after));
    let mut out = blank(op);
    if poll_once(run(&mut card, op, &mut out)).is_ready() {
        findings.push(format!("{at}: completed without reaching the await"));
        return findings;
    }
    card.bus().pend = None;
    if let Err(m) = check_not_mixed(&mut card, op) {
        findings.push(format!("{at}: {m}"));
    }
    let mut out = blank(op);
    match block_on(run(&mut card, op, &mut out)) {
        Ok(()) => {
            if let Err(m) = check_done(&mut card, op, &out) {
                findings.push(format!("{at}: retry: {m}"));
            }
        }
        Err(e) => findings.push(format!("{at}: retry failed: {e}")),
    }
    if let Err(m) = check_not_mixed(&mut card, op) {
        findings.push(format!("{at}: after retry: {m}"));
    }
    findings
}

fn poll_once<F: Future>(f: F) -> Poll<F::Output> {
    let mut f = pin!(f);
    f.as_mut().poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn bus_failure_at_every_phase() {
    let mut findings: Vec<String> = Vec::new();
    for cmd23 in [true, false] {
        for op in OPS {
            let counts = clean_counts(cmd23, op);
            eprintln!(
                "{}: {} fallible phases ({} commands, {} block reads, {} block writes, {} ready waits), {} awaits",
                label(op, cmd23),
                counts.fallible,
                counts.commands,
                counts.reads,
                counts.writes,
                counts.waits,
                counts.awaits
            );
            for k in 1..=counts.fallible {
                for after in [false, true] {
                    for err in [BusError::Timeout, BusError::Fault] {
                        findings.extend(bus_case(cmd23, op, k, after, err));
                    }
                }
            }
        }
    }
    assert!(
        findings.is_empty(),
        "{} findings:\n{}",
        findings.len(),
        findings.join("\n")
    );
}

#[test]
fn drop_at_every_await_point() {
    let mut findings: Vec<String> = Vec::new();
    for cmd23 in [true, false] {
        for op in OPS {
            let counts = clean_counts(cmd23, op);
            eprintln!(
                "{}: {} await points x before/after the card acts",
                label(op, cmd23),
                counts.awaits
            );
            // Control: the helper sees Pending exactly at the awaits that exist.
            let mut card = setup(cmd23, op);
            card.bus().pend = Some((counts.awaits + 1, false));
            let mut out = blank(op);
            assert!(poll_once(run(&mut card, op, &mut out)).is_ready());
            for k in 1..=counts.awaits {
                for after in [false, true] {
                    findings.extend(drop_case(cmd23, op, k, after));
                }
            }
        }
    }
    assert!(
        findings.is_empty(),
        "{} findings:\n{}",
        findings.len(),
        findings.join("\n")
    );
}

// Regression tests for a defect found by the two sweeps above. A multi-block write on a card
// without CMD23 starts with CMD55 and ACMD23. If the sequence stops after the card has accepted
// CMD55 (the answer lost, or the future dropped) and before ACMD23 has reached it, the card still
// expects an application command and would take the clean-up's CMD13 for ACMD13. The driver now
// remembers the open pair and closes it first (see `SdCard::cleanup`). The same applies when
// ACMD23 is lost before it reaches the card.

#[test]
fn driver_defect_bus_failure_between_cmd55_and_acmd23() {
    let mut findings = Vec::new();
    for (k, after) in [(1, true), (2, false)] {
        for err in [BusError::Timeout, BusError::Fault] {
            findings.extend(bus_case(false, Op::WriteN, k, after, err));
        }
    }
    assert!(
        findings.is_empty(),
        "{} findings:\n{}",
        findings.len(),
        findings.join("\n")
    );
}

#[test]
fn driver_defect_drop_between_cmd55_and_acmd23() {
    let mut findings = Vec::new();
    for (k, after) in [(1, true), (2, false)] {
        findings.extend(drop_case(false, Op::WriteN, k, after));
    }
    assert!(
        findings.is_empty(),
        "{} findings:\n{}",
        findings.len(),
        findings.join("\n")
    );
}

type Commands = Vec<(u8, u32)>;

/// The (index, argument) of every command the card has been sent since the trace was last taken.
fn commands(card: &mut SdCard<Probe>) -> Commands {
    card.bus()
        .sim
        .take_trace()
        .into_iter()
        .filter_map(|e| match e.event {
            TraceEvent::Command { index, arg, .. } => Some((index, arg)),
            _ => None,
        })
        .collect()
}

const RCA_ARG: u32 = (RCA as u32) << 16;

/// Without an interruption no command is added: the exact command trace of each operation.
#[test]
fn uninterrupted_operations_send_no_extra_command() {
    let a = |i| LBA + i;
    let expected: [(Op, bool, Commands); 8] = [
        (Op::Read1, false, vec![(17, a(0)), (13, RCA_ARG)]),
        (Op::Read1, true, vec![(17, a(0)), (13, RCA_ARG)]),
        (Op::ReadN, false, vec![(18, a(0)), (12, 0), (13, RCA_ARG)]),
        (Op::ReadN, true, vec![(23, 4), (18, a(0)), (13, RCA_ARG)]),
        (Op::Write1, false, vec![(24, a(0)), (13, RCA_ARG)]),
        (Op::Write1, true, vec![(24, a(0)), (13, RCA_ARG)]),
        (
            Op::WriteN,
            false,
            vec![(55, RCA_ARG), (23, 4), (25, a(0)), (12, 0), (13, RCA_ARG)],
        ),
        (Op::WriteN, true, vec![(23, 4), (25, a(0)), (13, RCA_ARG)]),
    ];
    for (op, cmd23, want) in expected {
        let mut card = setup(cmd23, op);
        card.bus().sim.take_trace();
        let before = card.stats().commands_sent as usize;
        let mut out = blank(op);
        block_on(run(&mut card, op, &mut out)).unwrap();
        check_done(&mut card, op, &out).unwrap();
        let got = commands(&mut card);
        assert_eq!(got, want, "{}", label(op, cmd23));
        assert_eq!(card.stats().commands_sent as usize - before, got.len());
        assert_eq!(card.stats().recoveries, 0);
    }
}

/// The pair is cut short by dropping the first write at CMD55 (answered) or before ACMD23.
const CUT: [(usize, bool); 2] = [(1, true), (2, false)];

fn cut_pair(card: &mut SdCard<Probe>, k: usize, after: bool) {
    card.bus().pend = Some((k, after));
    let mut out = blank(Op::WriteN);
    assert!(poll_once(run(card, Op::WriteN, &mut out)).is_pending());
    card.bus().pend = None;
    card.bus().n = Counts::default();
}

/// The neutral pair that closes an open pair is itself cut short; the call after that works and
/// the card breaks no rule.
#[test]
fn closing_pair_interrupted() {
    let mut findings: Vec<String> = Vec::new();
    for (cut_k, cut_after) in CUT {
        // The neutral pair is CMD55 (1) then ACMD23 (2) of the next call.
        for k in 1..=2usize {
            for after in [false, true] {
                for case in 0..3 {
                    let at = format!("cut {cut_k}/{cut_after}, neutral {k}/{after}, case {case}");
                    let mut card = setup(false, Op::WriteN);
                    cut_pair(&mut card, cut_k, cut_after);
                    let mut out = blank(Op::WriteN);
                    match case {
                        0 | 1 => {
                            let err = [BusError::Timeout, BusError::Fault][case];
                            card.bus().fail = Some((k, after, err));
                            let want = [55, 23][k - 1];
                            match block_on(run(&mut card, Op::WriteN, &mut out)) {
                                Err(Error::UnexpectedState {
                                    cmd, state: None, ..
                                }) if cmd == want => {}
                                other => findings.push(format!("{at}: result {other:?}")),
                            }
                            card.bus().fail = None;
                        }
                        _ => {
                            card.bus().pend = Some((k, after));
                            if poll_once(run(&mut card, Op::WriteN, &mut out)).is_ready() {
                                findings.push(format!("{at}: completed without the await"));
                            }
                            card.bus().pend = None;
                        }
                    }
                    let mut out = blank(Op::WriteN);
                    match block_on(run(&mut card, Op::WriteN, &mut out)) {
                        Ok(()) => {
                            if let Err(m) = check_done(&mut card, Op::WriteN, &out) {
                                findings.push(format!("{at}: {m}"));
                            }
                        }
                        Err(e) => findings.push(format!("{at}: next operation failed: {e}")),
                    }
                }
            }
        }
    }
    assert!(
        findings.is_empty(),
        "{} findings:\n{}",
        findings.len(),
        findings.join("\n")
    );
}

/// `init` after an interrupted pair works, and the mark is gone: the next write sends exactly
/// its usual commands.
#[test]
fn init_after_interrupted_pair_clears_the_mark() {
    for (k, after) in CUT {
        let mut card = setup(false, Op::WriteN);
        cut_pair(&mut card, k, after);
        block_on(card.init()).expect("init after the interrupted pair");
        let recoveries = card.stats().recoveries;
        assert_eq!(card.bus().sim.violations(), &[], "cut {k}/{after}");
        card.bus().sim.take_trace();
        let mut out = blank(Op::WriteN);
        block_on(run(&mut card, Op::WriteN, &mut out)).unwrap();
        check_done(&mut card, Op::WriteN, &out).unwrap();
        assert_eq!(
            commands(&mut card),
            vec![(55, RCA_ARG), (23, 4), (25, LBA), (12, 0), (13, RCA_ARG)],
            "cut {k}/{after}"
        );
        assert_eq!(card.stats().recoveries, recoveries);
    }
}

/// The neutral pair fails inside the clean-up of the operation's own failure: the clean-up's
/// error replaces the original one, as for a failed CMD13, and the next call closes the pair.
#[test]
fn closing_pair_failing_in_the_own_cleanup_returns_unexpected_state() {
    let mut card = setup(false, Op::WriteN);
    // The first CMD55 (the write's) and the second (the clean-up's) get no answer.
    card.bus()
        .sim
        .inject(Fault::NoResponse, Trigger::command(55, 1));
    card.bus()
        .sim
        .inject(Fault::NoResponse, Trigger::command(55, 2));
    let mut out = blank(Op::WriteN);
    match block_on(run(&mut card, Op::WriteN, &mut out)) {
        Err(Error::UnexpectedState {
            cmd: 55,
            state: None,
            ..
        }) => {}
        other => panic!("{other:?}"),
    }
    let mut out = blank(Op::WriteN);
    block_on(run(&mut card, Op::WriteN, &mut out)).unwrap();
    check_done(&mut card, Op::WriteN, &out).unwrap();
}
