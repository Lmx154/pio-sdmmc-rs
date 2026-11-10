//! Bus-level test of the PIO + DMA SD bus on MARV V2 Rev 1 (microSD J11, DAT0-3 = GPIO28-31,
//! CLK = GPIO32, CMD = GPIO33). Drives `pio_sdmmc::pio::PioBus` with raw `Bus` calls and `proto`
//! helpers; there is no card driver here. The card is expendable: blocks are overwritten.
//!
//! H0 hardware facts (JMP pin numbering and synchroniser bypass bits under GPIOBASE = 16, probed
//! through PIO0 SM0 registers before the bus exists), S2 identification at 400 kHz, S3 reads,
//! S4 writes, S5 late DMA (back-pressure), S6 speed at 25 MHz.
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::{Pin, pin};
use core::task::Poll;

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::program::{
    InstructionOperands, JmpCondition, MovDestination, MovOperation, MovSource, SetDestination,
};
use embassy_rp::pio::{self, Pio};
use embassy_time::{Duration, Instant, block_for};
use panic_probe as _;
use pio_sdmmc::bus::as_bytes;
use pio_sdmmc::pio::{PioBus, PioBusConfig};
use pio_sdmmc::proto::{
    self, CardState, CardStatus, Cid, Command, CrcStatus, Csd, ResponseError, Scr,
};
use pio_sdmmc::{Bus, BusError, ResponseBuf};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

type Sd = PioBus<'static, PIO0, 0>;

const CMD_GPIO: usize = 33;
const DAT0_GPIO: usize = 28;
const EXPECT_BLOCKS: u64 = 124_735_488;
const EXPECT_OCR: u32 = 0xC0FF_8000;
const READ_TIMEOUT_MS: u32 = 100;
const WRITE_TIMEOUT_MS: u32 = 500;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Fail {
    Bus(BusError),
    Response(ResponseError),
    Status(u32),
}

struct Rng(u32);

impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }

    fn fill(&mut self, block: &mut [u32]) {
        for w in block {
            *w = self.next();
        }
    }
}

fn cycles() -> u32 {
    DWT::cycle_count()
}

fn cycles_to_us(c: u32) -> u32 {
    c / (embassy_rp::clocks::clk_sys_freq() / 1_000_000)
}

fn crc_of(block: &[u32]) -> u64 {
    proto::crc16_4bit(as_bytes(block))
}

async fn r1(bus: &mut Sd, cmd: Command) -> Result<CardStatus, Fail> {
    let mut r = [0u8; 6];
    bus.command(&cmd.frame(), ResponseBuf::Short(&mut r))
        .await
        .map_err(Fail::Bus)?;
    let status = proto::decode_r1(cmd.index, &r).map_err(Fail::Response)?;
    if status.errors() != 0 {
        return Err(Fail::Status(status.0));
    }
    Ok(status)
}

async fn status(bus: &mut Sd, rca: u16) -> Result<CardStatus, Fail> {
    let mut r = [0u8; 6];
    let cmd = Command::send_status(rca);
    bus.command(&cmd.frame(), ResponseBuf::Short(&mut r))
        .await
        .map_err(Fail::Bus)?;
    proto::decode_r1(13, &r).map_err(Fail::Response)
}

async fn acmd(bus: &mut Sd, rca: u16, cmd: Command) -> Result<CardStatus, Fail> {
    r1(bus, Command::app_cmd(rca)).await?;
    r1(bus, cmd).await
}

async fn read_one(bus: &mut Sd, addr: u32, buf: &mut [u32]) -> Result<u64, Fail> {
    r1(bus, Command::read_single_block(addr)).await?;
    bus.read_block(buf, READ_TIMEOUT_MS)
        .await
        .map_err(Fail::Bus)
}

async fn write_one(bus: &mut Sd, addr: u32, buf: &[u32], crc: u64) -> Result<u8, Fail> {
    r1(bus, Command::write_block(addr)).await?;
    bus.write_block(buf, crc, WRITE_TIMEOUT_MS)
        .await
        .map_err(Fail::Bus)
}

/// Polls `fut` exactly once with the current task's context.
async fn poll_once<F: Future>(mut fut: Pin<&mut F>) -> Poll<F::Output> {
    poll_fn(|cx| Poll::Ready(fut.as_mut().poll(cx))).await
}

/// RX FIFO level of PIO0 SM0.
fn rx_level() -> u32 {
    (pac::PIO0.flevel().read().0 >> 4) & 0xF
}

/// Spins until the RX FIFO of PIO0 SM0 holds `n` words, for at most `ms`.
fn spin_until_rx(n: u32, ms: u64) -> bool {
    let t0 = Instant::now();
    while rx_level() < n {
        if t0.elapsed() > Duration::from_millis(ms) {
            return false;
        }
    }
    true
}

// ---- H0: raw probes through PIO0 SM0 registers, before the bus exists ----

fn exec(instr: u16) {
    pac::PIO0.sm(0).instr().write(|w| w.set_instr(instr));
}

fn pc() -> u8 {
    pac::PIO0.sm(0).addr().read().addr()
}

fn set_instr(dest: SetDestination, data: u8) -> u16 {
    InstructionOperands::SET {
        destination: dest,
        data,
    }
    .encode()
}

fn jmp_instr(condition: JmpCondition, address: u8) -> u16 {
    InstructionOperands::JMP { condition, address }.encode()
}

/// Gives `gpio` to PIO0 the way embassy's `make_pio_pin` does (no pulls; the board has 10k
/// pull-ups on CMD and DAT0-3).
fn pad_to_pio0(gpio: usize) {
    pac::PADS_BANK0.gpio(gpio).write(|w| {
        w.set_ie(true);
        w.set_od(false);
        w.set_iso(false);
        w.set_schmitt(true);
    });
    pac::IO_BANK0
        .gpio(gpio)
        .ctrl()
        .write(|w| w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::PIO0_0 as _));
}

/// SM0 disabled, divider 1, GPIOBASE 16, SET base = `gpio` (count 1), latched low, released.
fn probe_setup(gpio: usize) {
    pac::PIO0
        .ctrl()
        .modify(|w| w.set_sm_enable(w.sm_enable() & !1));
    pac::PIO0.gpiobase().write(|w| w.set_gpiobase(true));
    let sm = pac::PIO0.sm(0);
    sm.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm.pinctrl().write(|w| {
        w.set_set_base((gpio - 16) as u8);
        w.set_set_count(1);
    });
    exec(set_instr(SetDestination::PINDIRS, 0));
    exec(set_instr(SetDestination::PINS, 0));
}

/// With JMP_PIN = `field`: is `jmp pin` taken while the probed line is driven low, and while it
/// is released (pulled up)?
fn jmp_probe(field: u8) -> (bool, bool) {
    pac::PIO0.sm(0).execctrl().modify(|w| w.set_jmp_pin(field));
    let mut taken = [false; 2];
    for (i, drive) in [true, false].into_iter().enumerate() {
        exec(set_instr(SetDestination::PINDIRS, drive as u8));
        block_for(Duration::from_micros(50));
        exec(jmp_instr(JmpCondition::Always, 0));
        exec(jmp_instr(JmpCondition::PinHigh, 5));
        taken[i] = pc() == 5;
    }
    exec(set_instr(SetDestination::PINDIRS, 0));
    (taken[0], taken[1])
}

/// Writes `instrs` to SM0_INSTR back to back with DMA channel 1 (TREQ permanent), so that the
/// executed instructions follow each other by a cycle or two instead of a CPU store each.
fn exec_by_dma(instrs: &[u32]) {
    let ch = pac::DMA.ch(1);
    ch.read_addr().write_value(instrs.as_ptr() as u32);
    ch.write_addr()
        .write_value(pac::PIO0.sm(0).instr().as_ptr() as u32);
    ch.trans_count().write(|w| {
        w.set_mode(0.into());
        w.set_count(instrs.len() as u32);
    });
    ch.ctrl_trig().write(|w| {
        w.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(false);
        w.set_chain_to(1);
        w.set_en(true);
    });
    while ch.ctrl_trig().read().busy() {}
}

/// Of 16 tries, how often `jmp pin` already sees the line low when it executes `gap` exec'd
/// NOPs (`mov y, y`) after the `set pindirs, 1` that starts driving it low. JMP_PIN must address
/// the probed line.
fn latency_tries(gap: usize) -> u8 {
    let nop = u32::from(
        InstructionOperands::MOV {
            destination: MovDestination::Y,
            op: MovOperation::None,
            source: MovSource::Y,
        }
        .encode(),
    );
    let mut seq = [nop; 16];
    seq[0] = u32::from(set_instr(SetDestination::PINDIRS, 1));
    seq[gap + 1] = u32::from(jmp_instr(JmpCondition::PinHigh, 5));
    let mut seen = 0u8;
    for _ in 0..16 {
        exec(set_instr(SetDestination::PINDIRS, 0));
        block_for(Duration::from_micros(20));
        exec(jmp_instr(JmpCondition::Always, 0));
        exec_by_dma(&seq[..gap + 2]);
        if pc() == 0 {
            seen += 1;
        }
    }
    exec(set_instr(SetDestination::PINDIRS, 0));
    seen
}

/// Smallest gap at which all 16 tries see the new level, for each bypass mask.
fn latency_profile(mask: u32) -> ([u8; 12], u8) {
    pac::PIO0.input_sync_bypass().write_value(mask);
    let mut counts = [0u8; 12];
    for (gap, c) in counts.iter_mut().enumerate() {
        *c = latency_tries(gap);
    }
    let first = counts
        .iter()
        .position(|&c| c == 16)
        .map_or(255, |p| p as u8);
    (counts, first)
}

fn h0_probes(rep: &mut Report) {
    pad_to_pio0(CMD_GPIO);
    pad_to_pio0(DAT0_GPIO);
    for (name, gpio) in [("CMD/GPIO33", CMD_GPIO), ("DAT0/GPIO28", DAT0_GPIO)] {
        probe_setup(gpio);
        let relative = (gpio - 16) as u8;
        let raw = (gpio & 31) as u8;
        let (rel_low, rel_high) = jmp_probe(relative);
        let (raw_low, raw_high) = jmp_probe(raw);
        defmt::info!(
            "[INFO] H0 {=str}: JMP_PIN {} (GPIO - GPIOBASE): taken low={} high={}; JMP_PIN {} (embassy's value): taken low={} high={}",
            name,
            relative,
            rel_low,
            rel_high,
            raw,
            raw_low,
            raw_high
        );
        check!(
            rep,
            !rel_low && rel_high,
            "{=str} H0 {=str}: jmp pin with JMP_PIN = GPIO - 16 = {} follows the line (low: not taken, high: taken)",
            name,
            relative
        );
        check!(
            rep,
            !(!raw_low && raw_high),
            "{=str} H0 {=str}: jmp pin with JMP_PIN = {} (embassy's set_jmp_pin) does not follow the line",
            name,
            raw
        );

        // Synchroniser latency with the bypass bit counted from GPIOBASE, counted absolutely
        // (mod 32), and not set.
        pac::PIO0
            .sm(0)
            .execctrl()
            .modify(|w| w.set_jmp_pin(relative));
        let (none, t_none) = latency_profile(0);
        let (rel, t_rel) = latency_profile(1 << relative);
        let (abs, t_abs) = latency_profile(1 << raw);
        pac::PIO0.input_sync_bypass().write_value(1 << relative);
        let readback = pac::PIO0.input_sync_bypass().read();
        pac::PIO0.input_sync_bypass().write_value(0);
        defmt::info!(
            "[INFO] H0 {=str}: tries (of 16) seeing the new level at NOP gap 0..11: bypass none {=[u8]}",
            name,
            &none[..]
        );
        defmt::info!(
            "[INFO] H0 {=str}:   bypass bit {} (GPIO - 16) {=[u8]}; bypass bit {} (GPIO mod 32) {=[u8]}",
            name,
            relative,
            &rel[..],
            raw,
            &abs[..]
        );
        defmt::info!(
            "[INFO] H0 {=str}: first gap seen 16/16: none {}, bit {} {}, bit {} {}; INPUT_SYNC_BYPASS readback {:#010x}",
            name,
            t_none,
            relative,
            t_rel,
            raw,
            t_abs,
            readback
        );
        check!(
            rep,
            t_rel < t_none && t_abs == t_none,
            "{=str} H0 {=str}: bypass bit {} shortens the input path ({} -> {} NOPs), bit {} does not ({})",
            name,
            relative,
            t_none,
            t_rel,
            raw,
            t_abs
        );
    }
    pac::PIO0.gpiobase().write(|w| w.set_gpiobase(false));
}

// ---- S2 ----

struct Card {
    rca: u16,
    blocks: u64,
    cmd23: bool,
}

async fn s2_identify(rep: &mut Report, bus: &mut Sd) -> Option<Card> {
    let rate = bus.set_clock(400_000);
    check!(
        rep,
        rate <= 400_000 && rate > 390_000,
        "{=str} S2 set_clock(400 kHz) -> {} Hz",
        rate
    );
    bus.clock_idle(80).await;
    let r = bus
        .command(&Command::go_idle_state().frame(), ResponseBuf::None)
        .await;
    check!(rep, r.is_ok(), "{=str} S2 CMD0 sent: {}", r);

    let mut f = [0u8; 6];
    let r = bus
        .command(
            &Command::send_if_cond(0xAA).frame(),
            ResponseBuf::Short(&mut f),
        )
        .await;
    let if_cond = r
        .map_err(Fail::Bus)
        .and_then(|_| proto::decode_r7(&f).map_err(Fail::Response));
    if !check!(
        rep,
        matches!(if_cond, Ok(c) if c.check_pattern == 0xAA && c.voltage_accepted == 1),
        "{=str} S2 CMD8 R7 {=[u8]:#04x} -> {} (echo 0xAA, VHS 1, CRC7 valid)",
        &f[..],
        if_cond
    ) {
        return None;
    }

    let t0 = Instant::now();
    let mut ocr = None;
    let mut tries = 0;
    while t0.elapsed() < Duration::from_secs(2) {
        tries += 1;
        if r1(bus, Command::app_cmd(0)).await.is_ok() {
            let mut f = [0u8; 6];
            let cmd = Command::sd_send_op_cond(true, 0x00FF_8000);
            if bus
                .command(&cmd.frame(), ResponseBuf::Short(&mut f))
                .await
                .is_ok()
            {
                if let Ok(o) = proto::decode_r3(&f) {
                    if o.power_up_done() {
                        ocr = Some(o.0);
                        break;
                    }
                }
            }
        }
        bus.delay_ms(10).await;
    }
    if !check!(
        rep,
        ocr == Some(EXPECT_OCR),
        "{=str} S2 ACMD41 ready after {} tries, {} ms: OCR {:#010x} (expect {:#010x})",
        tries,
        t0.elapsed().as_millis(),
        ocr.unwrap_or(0),
        EXPECT_OCR
    ) {
        return None;
    }

    let mut f = [0u8; 17];
    let r = bus
        .command(&Command::all_send_cid().frame(), ResponseBuf::Long(&mut f))
        .await;
    let cid = r
        .map_err(Fail::Bus)
        .and_then(|_| proto::decode_r2(&f).map_err(Fail::Response))
        .map(|raw| Cid::parse(&raw));
    match cid {
        Ok(c) => {
            check!(
                rep,
                c.manufacturer_id == 0x03 && &c.oem_id == b"SD" && &c.product_name == b"SK64G",
                "{=str} S2 CMD2 CID: manufacturer {:#04x}, OEM {=[u8]:a}, product {=[u8]:a}, serial {:#010x}, {}-{}",
                c.manufacturer_id,
                &c.oem_id[..],
                &c.product_name[..],
                c.serial,
                c.year(),
                c.month
            );
        }
        Err(e) => {
            check!(rep, false, "{=str} S2 CMD2 failed: {}", e);
            return None;
        }
    }

    let mut f = [0u8; 6];
    let r = bus
        .command(
            &Command::send_relative_addr().frame(),
            ResponseBuf::Short(&mut f),
        )
        .await;
    let rca = match r
        .map_err(Fail::Bus)
        .and_then(|_| proto::decode_r6(&f).map_err(Fail::Response))
    {
        Ok((rca, s)) if rca != 0 => {
            check!(
                rep,
                true,
                "{=str} S2 CMD3 RCA {:#06x}, state {}",
                rca,
                s.state()
            );
            rca
        }
        other => {
            check!(rep, false, "{=str} S2 CMD3 failed: {}", other);
            return None;
        }
    };

    let mut f = [0u8; 17];
    let r = bus
        .command(&Command::send_csd(rca).frame(), ResponseBuf::Long(&mut f))
        .await;
    let csd = r
        .map_err(Fail::Bus)
        .and_then(|_| proto::decode_r2(&f).map_err(Fail::Response))
        .map(|raw| Csd::parse(&raw));
    let blocks = match csd {
        Ok(Ok(c)) => c.capacity_blocks,
        _ => 0,
    };
    check!(
        rep,
        blocks == EXPECT_BLOCKS,
        "{=str} S2 CMD9 CSD: {} blocks (expect {})",
        blocks,
        EXPECT_BLOCKS
    );
    if blocks == 0 {
        return None;
    }

    let r = r1(bus, Command::select_card(rca)).await;
    let ready = bus.wait_ready(500).await;
    check!(
        rep,
        r.is_ok() && ready.is_ok(),
        "{=str} S2 CMD7 select: {}, wait_ready {}",
        r,
        ready
    );
    let r = acmd(bus, rca, Command::set_bus_width(true)).await;
    check!(rep, r.is_ok(), "{=str} S2 ACMD6 four-bit: {}", r);

    // A command the card ignores times out and leaves the bus usable.
    let mut f = [0u8; 6];
    let t = Instant::now();
    let r = bus
        .command(
            &Command::send_if_cond(0xAA).frame(),
            ResponseBuf::Short(&mut f),
        )
        .await;
    let us = t.elapsed().as_micros();
    let exec_stalled = pac::PIO0.sm(0).execctrl().read().exec_stalled();
    check!(
        rep,
        r == Err(BusError::Timeout) && !exec_stalled,
        "{=str} S2 CMD8 in transfer state (ignored by the card): {} after {} us; EXEC_STALLED after abort = {}",
        r,
        us,
        exec_stalled
    );
    let s = status(bus, rca).await;
    check!(
        rep,
        matches!(s, Ok(st) if st.0 & CardStatus::ILLEGAL_COMMAND != 0 && st.state() == CardState::Tran),
        "{=str} S2 next command CMD13 succeeds: {} (ILLEGAL_COMMAND set, state tran)",
        s
    );
    let mut bad = Command::send_status(rca).frame();
    bad[5] ^= 0x02;
    let r = bus.command(&bad, ResponseBuf::Short(&mut f)).await;
    let s = status(bus, rca).await;
    check!(
        rep,
        r == Err(BusError::Timeout) && matches!(s, Ok(st) if st.0 & CardStatus::COM_CRC_ERROR != 0),
        "{=str} S2 CMD13 with a wrong CRC7: {}; next CMD13: {} (COM_CRC_ERROR set)",
        r,
        s
    );
    Some(Card {
        rca,
        blocks,
        cmd23: false,
    })
}

// ---- S3 ----

async fn s3_reads(rep: &mut Report, bus: &mut Sd, card: &mut Card) {
    let rate = bus.set_clock(25_000_000);
    check!(
        rep,
        rate == 25_000_000,
        "{=str} S3 set_clock(25 MHz) -> {} Hz",
        rate
    );
    let mut buf = [0u32; 128];
    let r = read_one(bus, 0, &mut buf).await;
    let calc = crc_of(&buf);
    let b = as_bytes(&buf);
    check!(
        rep,
        r == Ok(calc),
        "{=str} S3 block 0: CRC received {:#018x}, computed {:#018x}; bytes 510-511 = {:#04x} {:#04x}",
        r.unwrap_or(0),
        calc,
        b[510],
        b[511]
    );

    let mut scr = [0u32; 2];
    let r = match acmd(bus, card.rca, Command::send_scr()).await {
        Ok(_) => bus
            .read_block(&mut scr, READ_TIMEOUT_MS)
            .await
            .map_err(Fail::Bus),
        Err(e) => Err(e),
    };
    let calc = crc_of(&scr);
    let raw: [u8; 8] = as_bytes(&scr).try_into().unwrap();
    let s = Scr::parse(&raw);
    card.cmd23 = s.supports_cmd23();
    check!(
        rep,
        r == Ok(calc) && s.supports_4bit(),
        "{=str} S3 ACMD51 SCR {=[u8]:#04x}: CRC received {:#018x}, computed {:#018x}; spec {}, 4-bit {}, CMD23 {}",
        &raw[..],
        r.unwrap_or(0),
        calc,
        s.sd_spec,
        s.supports_4bit(),
        s.supports_cmd23()
    );

    let mut rng = Rng(0x2545_F491);
    let (mut bad, mut errors, mut total_us, mut worst_us) = (0u32, 0u32, 0u64, 0u64);
    const N: u32 = 200;
    for i in 0..N {
        let stride = card.blocks / u64::from(N);
        let addr = (u64::from(i) * stride + u64::from(rng.next()) % stride) as u32;
        let t = Instant::now();
        let r = read_one(bus, addr, &mut buf).await;
        let us = t.elapsed().as_micros();
        total_us += us;
        worst_us = worst_us.max(us);
        match r {
            Ok(crc) if crc == crc_of(&buf) => {}
            Ok(_) => bad += 1,
            Err(e) => {
                errors += 1;
                defmt::error!("[INFO] S3 read of block {} failed: {}", addr, e);
            }
        }
    }
    check!(
        rep,
        bad == 0 && errors == 0,
        "{=str} S3 {} single-block reads across the card: {} CRC mismatches, {} errors",
        N,
        bad,
        errors
    );
    defmt::info!(
        "[STAT] S3 single-block read (CMD17 + read_block, random address) at 25 MHz: mean {} us, worst {} us",
        total_us / u64::from(N),
        worst_us
    );
}

// ---- S4 ----

async fn s4_writes(rep: &mut Report, bus: &mut Sd, card: &Card) {
    let mut rng = Rng(0x9E37_79B9);
    let mut data = [0u32; 128];
    let mut back = [0u32; 128];
    let base = 2_000_000u32;

    rng.fill(&mut data);
    let crc = crc_of(&data);
    let t = Instant::now();
    let tok = write_one(bus, base, &data, crc).await;
    let write_us = t.elapsed().as_micros();
    let st = status(bus, card.rca).await;
    let rd = read_one(bus, base, &mut back).await;
    check!(
        rep,
        tok == Ok(0b010)
            && matches!(st, Ok(s) if s.state() == CardState::Tran)
            && rd == Ok(crc)
            && back == data,
        "{=str} S4 pattern write to block {}: token {} in {} us, then CMD13 {} (state tran), read-back identical = {}",
        base,
        tok,
        write_us,
        st,
        back == data
    );

    let (mut mismatches, mut errors, mut not_tran, mut total_us, mut worst_us) =
        (0u32, 0u32, 0u32, 0u64, 0u64);
    const N: u32 = 50;
    for _ in 0..N {
        let addr = 1_000_000 + rng.next() % (card.blocks as u32 - 2_000_000);
        rng.fill(&mut data);
        let crc = crc_of(&data);
        let t = Instant::now();
        let tok = write_one(bus, addr, &data, crc).await;
        let us = t.elapsed().as_micros();
        total_us += us;
        worst_us = worst_us.max(us);
        let st = status(bus, card.rca).await;
        if !matches!(st, Ok(s) if s.state() == CardState::Tran) {
            not_tran += 1;
        }
        let rd = read_one(bus, addr, &mut back).await;
        match (tok, rd) {
            (Ok(0b010), Ok(c)) if c == crc && back == data => {}
            (Ok(_), Ok(_)) => mismatches += 1,
            (t, r) => {
                errors += 1;
                defmt::error!("[INFO] S4 block {}: token {}, read {}", addr, t, r);
            }
        }
    }
    check!(
        rep,
        mismatches == 0 && errors == 0 && not_tran == 0,
        "{=str} S4 {} write/read-back cycles at random blocks: {} mismatches, {} errors, {} times not in tran state after write_block",
        N,
        mismatches,
        errors,
        not_tran
    );
    defmt::info!(
        "[STAT] S4 single-block write (CMD24 + write_block incl. busy) at 25 MHz: mean {} us, worst {} us",
        total_us / u64::from(N),
        worst_us
    );

    // Wrong CRC: token 101, block unchanged.
    let mut other = [0u32; 128];
    rng.fill(&mut other);
    let tok = write_one(bus, base, &other, crc_of(&other) ^ 1).await;
    let st = status(bus, card.rca).await;
    let rd = read_one(bus, base, &mut back).await;
    let first = {
        let mut d = [0u32; 128];
        let mut r = Rng(0x9E37_79B9);
        r.fill(&mut d);
        d
    };
    check!(
        rep,
        tok.map(CrcStatus::from_token) == Ok(CrcStatus::CrcError)
            && rd == Ok(crc_of(&first))
            && back == first,
        "{=str} S4 write with a wrong CRC: token {} (expect 0b101), CMD13 {}, block unchanged = {}",
        tok,
        st,
        back == first
    );

    // wait_ready after CMD7 deselect / select.
    let r0 = bus
        .command(&Command::select_card(0).frame(), ResponseBuf::None)
        .await;
    let s0 = status(bus, card.rca).await;
    let r1s = r1(bus, Command::select_card(card.rca)).await;
    let t = Instant::now();
    let ready = bus.wait_ready(500).await;
    let us = t.elapsed().as_micros();
    let s1 = status(bus, card.rca).await;
    check!(
        rep,
        r0.is_ok()
            && matches!(s0, Ok(s) if s.state() == CardState::Stby)
            && r1s.is_ok()
            && ready.is_ok()
            && matches!(s1, Ok(s) if s.state() == CardState::Tran),
        "{=str} S4 CMD7 deselect -> {} (stby), CMD7 select {} + wait_ready {} in {} us -> {} (tran)",
        s0,
        r1s,
        ready,
        us,
        s1
    );
}

// ---- S5 ----

async fn s5_late_dma(rep: &mut Report, bus: &mut Sd) {
    let mut rng = Rng(0x0BAD_5EED);
    let mut buf = [0u32; 128];
    let (mut bad, mut errors, mut stalled, mut full) = (0u32, 0u32, 0u32, 0u32);
    const N: u32 = 50;
    for _ in 0..N {
        let addr = rng.next() % 100_000_000;
        if r1(bus, Command::read_single_block(addr)).await.is_err() {
            errors += 1;
            continue;
        }
        match bus
            .test_read_block_late_dma(&mut buf, READ_TIMEOUT_MS, Duration::from_millis(3))
            .await
        {
            Ok(r) => {
                if r.crc != crc_of(&buf) {
                    bad += 1;
                }
                stalled += u32::from(r.rx_stalled);
                full += u32::from(r.level_at_dma_start == 4);
            }
            Err(e) => {
                errors += 1;
                defmt::error!("[INFO] S5 late-DMA read of block {} failed: {}", addr, e);
            }
        }
    }
    check!(
        rep,
        bad == 0 && errors == 0 && stalled == N,
        "{=str} S5 {} reads with the DMA started 3 ms after the header: {} CRC mismatches, {} errors; RXSTALL raised in {}, RX FIFO full at DMA start in {}",
        N,
        bad,
        errors,
        stalled,
        full
    );
}

// ---- S6 ----

const RUN_BLOCKS: u32 = 2048;
const RUN_BASE: u32 = 8_000_000;
const PATTERNS: usize = 8;

struct Timing {
    total: u64,
    worst: u32,
    sum: u64,
}

impl Timing {
    fn new() -> Self {
        Self {
            total: 0,
            worst: 0,
            sum: 0,
        }
    }

    fn add(&mut self, cyc: u32) {
        let us = cycles_to_us(cyc);
        self.sum += u64::from(us);
        self.worst = self.worst.max(us);
    }
}

fn stat_run(what: &str, t: &Timing) {
    let bytes = u64::from(RUN_BLOCKS) * 512;
    let total = bytes * 100 / t.total.max(1);
    let calls = bytes * 100 / t.sum.max(1);
    defmt::info!(
        "[STAT] S6 {=str} of {} blocks: {} us total (commands and test loop included) = {}.{=u64:02} MB/s; bus calls only {} us = {}.{=u64:02} MB/s; per block mean {} us, worst {} us",
        what,
        RUN_BLOCKS,
        t.total,
        total / 100,
        total % 100,
        t.sum,
        calls / 100,
        calls % 100,
        t.sum / u64::from(RUN_BLOCKS),
        t.worst
    );
}

/// The bus's own CPU time inside a call, split at the point where it waits for the hardware.
struct Split {
    n: u32,
    min: [u32; 2],
    max: [u32; 2],
}

impl Split {
    fn new() -> Self {
        Self {
            n: 0,
            min: [u32::MAX; 2],
            max: [0; 2],
        }
    }
}

/// First blocks of each run whose call is split; 8 in a row, so the code is in the XIP cache.
const SPLIT_FROM: u32 = 1024;
const SPLIT_COUNT: u32 = 8;

/// Polls `fut` once (everything up to its first wait), spins until the state machine has pushed
/// `rx_words` words (the hardware part is over), then polls it again (everything after the final
/// word), timing both polls with the cycle counter.
async fn split_call<F: Future>(fut: F, rx_words: u32, split: &mut Split) -> F::Output {
    let mut fut = pin!(fut);
    let a = cycles();
    let first = poll_once(fut.as_mut()).await;
    let before = cycles().wrapping_sub(a);
    if let Poll::Ready(r) = first {
        return r;
    }
    let ok = spin_until_rx(rx_words, 50);
    let b = cycles();
    let second = poll_once(fut.as_mut()).await;
    let after = cycles().wrapping_sub(b);
    match second {
        Poll::Ready(r) => {
            if ok {
                split.n += 1;
                for (k, v) in [before, after].into_iter().enumerate() {
                    split.min[k] = split.min[k].min(v);
                    split.max[k] = split.max[k].max(v);
                }
            }
            r
        }
        Poll::Pending => fut.await,
    }
}

fn stat_split(what: &str, s: &Split) {
    defmt::info!(
        "[STAT] S6 {=str}: bus CPU time per block ({} blocks): before the hardware runs {}..{} cycles = {}..{} us, after the final word {}..{} cycles = {}..{} us",
        what,
        s.n,
        s.min[0],
        s.max[0],
        cycles_to_us(s.min[0]),
        cycles_to_us(s.max[0]),
        s.min[1],
        s.max[1],
        cycles_to_us(s.min[1]),
        cycles_to_us(s.max[1])
    );
}

fn same(a: &[u32; 128], b: &[u32; 128]) -> bool {
    a.iter().zip(b).fold(0, |acc, (x, y)| acc | (x ^ y)) == 0
}

async fn s6_speed(
    rep: &mut Report,
    bus: &mut Sd,
    card: &Card,
    patterns: &mut [[u32; 128]; PATTERNS],
) {
    let mut crcs = [0u64; PATTERNS];
    let mut rng = Rng(0x1234_5678);
    for (p, c) in patterns.iter_mut().zip(crcs.iter_mut()) {
        rng.fill(p);
        *c = crc_of(p);
    }

    // (c) crc16_4bit on the target.
    let t = cycles();
    let mut acc = 0u64;
    for i in 0..1000 {
        acc ^= proto::crc16_4bit(as_bytes(&patterns[i % PATTERNS]));
    }
    let c = cycles().wrapping_sub(t);
    defmt::info!(
        "[STAT] S6 crc16_4bit of one 512-byte block: mean {} cycles = {} ns (1000 calls, clk_sys {} Hz; xor {:#x})",
        c / 1000,
        u64::from(c / 1000) * 1_000_000_000 / u64::from(embassy_rp::clocks::clk_sys_freq()),
        embassy_rp::clocks::clk_sys_freq(),
        acc
    );
    // The copy write_block makes into its staging buffer, for comparison with its overhead.
    let mut stage = [0u32; 132];
    let t = cycles();
    for i in 0..1000 {
        stage[2..130].copy_from_slice(&patterns[i % PATTERNS]);
        core::hint::black_box(&mut stage);
    }
    let c = cycles().wrapping_sub(t);
    defmt::info!(
        "[STAT] S6 copy of one 512-byte block into a staging buffer: mean {} cycles (1000 copies)",
        c / 1000
    );

    // (a) multi-block write.
    let mode = if card.cmd23 {
        "CMD23 + CMD25"
    } else {
        "ACMD23 + CMD25 + CMD12"
    };
    let mut tw = Timing::new();
    let (mut bad_tokens, mut errors) = (0u32, 0u32);
    let mut ws = Split::new();
    let t0 = Instant::now();
    let pre = if card.cmd23 {
        r1(bus, Command::set_block_count(RUN_BLOCKS as u16)).await
    } else {
        acmd(bus, card.rca, Command::set_wr_blk_erase_count(RUN_BLOCKS)).await
    };
    let start = r1(bus, Command::write_multiple_block(RUN_BASE)).await;
    if pre.is_err() || start.is_err() {
        check!(
            rep,
            false,
            "{=str} S6 multi-block write start: {} / {}",
            pre,
            start
        );
        return;
    }
    for i in 0..RUN_BLOCKS {
        let p = i as usize % PATTERNS;
        let c0 = cycles();
        let tok = if (SPLIT_FROM..SPLIT_FROM + SPLIT_COUNT).contains(&i) {
            split_call(
                bus.write_block(&patterns[p], crcs[p], WRITE_TIMEOUT_MS),
                1,
                &mut ws,
            )
            .await
        } else {
            bus.write_block(&patterns[p], crcs[p], WRITE_TIMEOUT_MS)
                .await
        };
        tw.add(cycles().wrapping_sub(c0));
        match tok {
            Ok(0b010) => {}
            Ok(_) => bad_tokens += 1,
            Err(_) => errors += 1,
        }
    }
    let stop = if card.cmd23 {
        Ok(())
    } else {
        match r1(bus, Command::stop_transmission()).await {
            Ok(_) => bus.wait_ready(WRITE_TIMEOUT_MS).await.map_err(Fail::Bus),
            Err(e) => Err(e),
        }
    };
    tw.total = t0.elapsed().as_micros();
    let st = status(bus, card.rca).await;
    check!(
        rep,
        bad_tokens == 0
            && errors == 0
            && stop.is_ok()
            && matches!(st, Ok(s) if s.state() == CardState::Tran),
        "{=str} S6 multi-block write ({=str}) at block {}: {} bad tokens, {} errors, stop {}, then CMD13 {}",
        mode,
        RUN_BASE,
        bad_tokens,
        errors,
        stop,
        st
    );
    stat_run("multi-block write", &tw);

    // (b) multi-block read of the same blocks.
    let mode = if card.cmd23 {
        "CMD23 + CMD18"
    } else {
        "CMD18 + CMD12"
    };
    let mut tr = Timing::new();
    let mut buf = [0u32; 128];
    let (mut bad, mut errors) = (0u32, 0u32);
    let mut rs = Split::new();
    let t0 = Instant::now();
    let pre = if card.cmd23 {
        r1(bus, Command::set_block_count(RUN_BLOCKS as u16))
            .await
            .map(drop)
    } else {
        Ok(())
    };
    let start = r1(bus, Command::read_multiple_block(RUN_BASE)).await;
    if pre.is_err() || start.is_err() {
        check!(
            rep,
            false,
            "{=str} S6 multi-block read start: {} / {}",
            pre,
            start
        );
        return;
    }
    for i in 0..RUN_BLOCKS {
        let p = i as usize % PATTERNS;
        let c0 = cycles();
        let r = if (SPLIT_FROM..SPLIT_FROM + SPLIT_COUNT).contains(&i) {
            split_call(bus.read_block(&mut buf, READ_TIMEOUT_MS), 3, &mut rs).await
        } else {
            bus.read_block(&mut buf, READ_TIMEOUT_MS).await
        };
        tr.add(cycles().wrapping_sub(c0));
        match r {
            // Data equal to what was written and the received CRC equal to the CRC of that data.
            Ok(crc) if crc == crcs[p] && same(&buf, &patterns[p]) => {}
            Ok(_) => bad += 1,
            Err(_) => errors += 1,
        }
    }
    let stop = if card.cmd23 {
        Ok(())
    } else {
        match r1(bus, Command::stop_transmission()).await {
            Ok(_) => bus.wait_ready(READ_TIMEOUT_MS).await.map_err(Fail::Bus),
            Err(e) => Err(e),
        }
    };
    tr.total = t0.elapsed().as_micros();
    let st = status(bus, card.rca).await;
    check!(
        rep,
        bad == 0
            && errors == 0
            && stop.is_ok()
            && matches!(st, Ok(s) if s.state() == CardState::Tran),
        "{=str} S6 multi-block read ({=str}): {} CRC/data mismatches, {} errors, stop {}, then CMD13 {}",
        mode,
        bad,
        errors,
        stop,
        st
    );
    stat_run("multi-block read", &tr);
    stat_split("read_block", &rs);
    stat_split("write_block", &ws);
    defmt::info!(
        "[STAT] S6 wire time per block at 25 MHz: read 1042 clocks = 41.7 us (12.29 MB/s), write 1049 clocks + card busy = 42.0 us + busy"
    );
}

static PATTERNS_BUF: static_cell::StaticCell<[[u32; 128]; PATTERNS]> =
    static_cell::StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
    let mut rep = Report { failures: 0 };
    defmt::info!(
        "[INFO] sd_bus: clk_sys = {} Hz",
        embassy_rp::clocks::clk_sys_freq()
    );

    h0_probes(&mut rep);

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let ch = dma::Channel::new(p.DMA_CH0, Irqs);
    let mut bus: Sd = PioBus::new(
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
    defmt::info!(
        "[INFO] bus up: GPIOBASE16 = {}, INPUT_SYNC_BYPASS = {:#010x}, SM0 EXECCTRL.JMP_PIN = {}",
        pac::PIO0.gpiobase().read().gpiobase(),
        pac::PIO0.input_sync_bypass().read(),
        pac::PIO0.sm(0).execctrl().read().jmp_pin()
    );

    if let Some(mut card) = s2_identify(&mut rep, &mut bus).await {
        s3_reads(&mut rep, &mut bus, &mut card).await;
        s4_writes(&mut rep, &mut bus, &card).await;
        s5_late_dma(&mut rep, &mut bus).await;
        s6_speed(
            &mut rep,
            &mut bus,
            &card,
            PATTERNS_BUF.init([[0; 128]; PATTERNS]),
        )
        .await;
    }
    hil_common::exit(rep.failures)
}
