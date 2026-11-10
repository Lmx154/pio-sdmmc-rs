//! A deterministic simulated SD card that implements [`Bus`](crate::bus::Bus), with scripted
//! fault injection. Host only.
//!
//! [`SimCard`] models the card side of the wire: it parses the command frames the host sends,
//! runs the card state machine of the SD Physical Layer Specification, and answers with real
//! response frames and data blocks in the wire formats of [`bus`](crate::bus). Time is virtual:
//! every call completes at once, and a microsecond counter advances by what the operation would
//! cost on a real bus at the current clock rate. Nothing depends on the wall clock or on
//! randomness, so the same calls always give the same results.
//!
//! The simulator is also a rule checker. Whenever the host breaks a rule of the SD protocol or of
//! the bus contract, it adds a [`Violation`] to a list and carries on; a test asserts the list is
//! empty. Every bus call is also logged in a [`TraceEntry`] list.
//!
//! Only native four-bit SD mode is modelled: no SPI, no 1.8 V, no CMD6.
//!
//! The simulator is written without any use of the crate's `proto` layer, so the two check each
//! other.

mod card;
mod config;
mod crc;
mod fault;
mod regs;
mod trace;

use std::collections::HashMap;

use crate::bus::{Bus, BusError, ResponseBuf};

pub use config::{CardKind, Cid, SimConfig};
pub use fault::{Fault, Match, Trigger, status};
pub use trace::{CardState, ResponseClass, TraceEntry, TraceEvent, Violation};

use fault::{Armed, Event};

/// What the card is waiting to move over the data lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    None,
    Read(ReadXfer),
    Write(WriteXfer),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadSource {
    Blocks,
    Scr,
    SdStatus,
    WrCount,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ReadXfer {
    source: ReadSource,
    next: u32,
    remaining: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WriteXfer {
    next: u32,
    remaining: Option<u32>,
    single: bool,
}

/// The simulated card and the bus it hangs on.
#[derive(Debug, Clone)]
pub struct SimCard {
    cfg: SimConfig,
    blocks: HashMap<u32, [u8; 512]>,

    now_us: u64,
    clock_hz: u32,
    removed: bool,
    idle_cycles: u64,
    first_command_seen: bool,

    state: CardState,
    busy_until: u64,
    status_err: u32,
    app_cmd: bool,
    four_bit: bool,
    cmd8_seen: bool,
    acmd41_started_at: Option<u64>,
    acmd41_polls: u32,
    block_count: Option<u32>,
    pending: Pending,
    last_write_ok: u32,
    write_failed: bool,
    total_written: u64,
    erase_start: Option<u32>,
    erase_end: Option<u32>,
    pre_erase_count: u32,
    card_detect_connected: bool,

    violations: Vec<Violation>,
    trace: Vec<TraceEntry>,
    faults: Vec<Armed>,
}

impl Default for SimCard {
    fn default() -> Self {
        Self::new(SimConfig::default())
    }
}

impl SimCard {
    /// Powers up a card with this configuration and an empty store.
    pub fn new(cfg: SimConfig) -> Self {
        assert!(cfg.rca != 0, "RCA 0 is reserved");
        assert!(cfg.base_clock_hz > 0, "base clock must be nonzero");
        let mut card = SimCard {
            cfg,
            blocks: HashMap::new(),
            now_us: 0,
            clock_hz: 0,
            removed: false,
            idle_cycles: 0,
            first_command_seen: false,
            state: CardState::Idle,
            busy_until: 0,
            status_err: 0,
            app_cmd: false,
            four_bit: false,
            cmd8_seen: false,
            acmd41_started_at: None,
            acmd41_polls: 0,
            block_count: None,
            pending: Pending::None,
            last_write_ok: 0,
            write_failed: false,
            total_written: 0,
            erase_start: None,
            erase_end: None,
            pre_erase_count: 0,
            card_detect_connected: true,
            violations: Vec::new(),
            trace: Vec::new(),
            faults: Vec::new(),
        };
        card.clock_hz = card.divide_clock(400_000);
        card
    }

    /// The configuration the card was built with.
    pub fn config(&self) -> &SimConfig {
        &self.cfg
    }

    /// Virtual time in microseconds since power-up of the simulation.
    pub fn now_us(&self) -> u64 {
        self.now_us
    }

    /// The current card clock in hertz.
    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    /// The card's state now (a finished programming or disconnect phase counts as finished).
    pub fn state(&self) -> CardState {
        self.effective_state()
    }

    /// True once ACMD6 has selected four-bit mode.
    pub fn is_four_bit(&self) -> bool {
        self.four_bit
    }

    /// Busy time still to run, in microseconds.
    pub fn busy_remaining_us(&self) -> u64 {
        self.busy_until.saturating_sub(self.now_us)
    }

    /// True while the card holds DAT0 low.
    pub fn is_busy(&self) -> bool {
        self.busy_remaining_us() > 0
    }

    /// True after a [`Fault::RemoveCard`] and before [`reinsert`](Self::reinsert).
    pub fn is_removed(&self) -> bool {
        self.removed
    }

    /// Blocks written without error by the last CMD24/CMD25 (what ACMD22 reports).
    pub fn last_write_ok_blocks(&self) -> u32 {
        self.last_write_ok
    }

    /// Number of blocks that hold stored data.
    pub fn stored_blocks(&self) -> usize {
        self.blocks.len()
    }

    /// Reads a block straight from the store, bypassing the bus. Never-written blocks are zeros.
    pub fn peek_block(&self, lba: u32) -> [u8; 512] {
        self.blocks.get(&lba).copied().unwrap_or([0; 512])
    }

    /// Writes a block straight into the store, bypassing the bus.
    pub fn poke_block(&mut self, lba: u32, data: &[u8; 512]) {
        self.blocks.insert(lba, *data);
    }

    /// The rule violations recorded so far.
    pub fn violations(&self) -> &[Violation] {
        &self.violations
    }

    /// Returns the recorded violations and empties the list.
    pub fn take_violations(&mut self) -> Vec<Violation> {
        std::mem::take(&mut self.violations)
    }

    /// The bus-call trace so far.
    pub fn trace(&self) -> &[TraceEntry] {
        &self.trace
    }

    /// Returns the trace and empties it.
    pub fn take_trace(&mut self) -> Vec<TraceEntry> {
        std::mem::take(&mut self.trace)
    }

    /// Arms a fault. Matches are counted from now on. Panics if the trigger cannot fire the
    /// fault, for example a block-read trigger on [`Fault::NoResponse`].
    pub fn inject(&mut self, fault: Fault, trigger: Trigger) {
        self.faults.push(Armed::new(fault, trigger));
    }

    /// Disarms every fault.
    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    /// Puts the card back after a removal, powered up fresh: idle state, no RCA, one-bit mode, no
    /// busy, and the power-up clock count restarted. Storage, faults, trace, violations and
    /// virtual time are kept.
    pub fn reinsert(&mut self) {
        self.removed = false;
        self.idle_cycles = 0;
        self.first_command_seen = false;
        self.last_write_ok = 0;
        self.card_detect_connected = true;
        self.reset_card();
    }

    fn divide_clock(&self, max_hz: u32) -> u32 {
        let base = self.cfg.base_clock_hz;
        let div = base.div_ceil(max_hz.max(1)).max(1);
        base / div
    }

    fn cycles_us(&self, cycles: u64) -> u64 {
        (cycles * 1_000_000).div_ceil(u64::from(self.clock_hz))
    }

    fn advance(&mut self, us: u64) {
        self.now_us += us;
    }

    fn advance_cycles(&mut self, cycles: u64) {
        let us = self.cycles_us(cycles);
        self.advance(us);
    }

    fn violate(&mut self, v: Violation) {
        self.violations.push(v);
    }

    fn check_clock(&mut self) {
        if self.clock_hz > 25_000_000 {
            self.violate(Violation::ClockAbove25M { hz: self.clock_hz });
        }
    }

    fn effective_state(&self) -> CardState {
        let busy = self.now_us < self.busy_until;
        match self.state {
            CardState::Prg if !busy => CardState::Tran,
            CardState::Dis if !busy => CardState::Stby,
            s => s,
        }
    }

    fn settle(&mut self) {
        self.state = self.effective_state();
    }

    fn fire(&mut self, ev: Event) -> Vec<Fault> {
        let mut out = Vec::new();
        for armed in &mut self.faults {
            if armed.fires(ev) {
                out.push(armed.fault);
            }
        }
        out
    }

    fn log(&mut self, at_us: u64, event: TraceEvent) {
        self.trace.push(TraceEntry { at_us, event });
    }
}

impl Bus for SimCard {
    fn set_clock(&mut self, max_hz: u32) -> u32 {
        let at = self.now_us;
        let actual = self.divide_clock(max_hz);
        self.clock_hz = actual;
        self.log(
            at,
            TraceEvent::SetClock {
                requested_hz: max_hz,
                actual_hz: actual,
            },
        );
        actual
    }

    async fn clock_idle(&mut self, cycles: u32) {
        let at = self.now_us;
        self.check_clock();
        self.idle_cycles = self.idle_cycles.saturating_add(u64::from(cycles));
        self.advance_cycles(u64::from(cycles));
        self.log(at, TraceEvent::ClockIdle { cycles });
    }

    async fn command(
        &mut self,
        frame: &[u8; 6],
        response: ResponseBuf<'_>,
    ) -> Result<(), BusError> {
        self.do_command(frame, response)
    }

    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError> {
        self.do_read_block(data, timeout_ms)
    }

    async fn write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError> {
        self.do_write_block(data, crc, timeout_ms)
    }

    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError> {
        let at = self.now_us;
        self.check_clock();
        let result = if self.removed {
            self.advance(u64::from(timeout_ms) * 1000);
            Err(BusError::Timeout)
        } else {
            let remaining = self.busy_remaining_us();
            let limit = u64::from(timeout_ms) * 1000;
            let min = self.cycles_us(1);
            if remaining <= limit {
                self.advance(remaining.max(min));
                Ok(())
            } else {
                self.advance(limit);
                Err(BusError::Timeout)
            }
        };
        self.settle();
        self.log(at, TraceEvent::WaitReady { timeout_ms, result });
        result
    }

    async fn delay_ms(&mut self, ms: u32) {
        let at = self.now_us;
        self.advance(u64::from(ms) * 1000);
        self.log(at, TraceEvent::DelayMs { ms });
    }
}
