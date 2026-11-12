//! The bus type: one state machine, one DMA channel, two pin mappings.

use embassy_rp::Peri;
use embassy_rp::dma;
use embassy_rp::gpio::{Drive, Level, SlewRate};
use embassy_rp::pio::program::{InstructionOperands, OutDestination};
use embassy_rp::pio::{
    Common, Config, Direction, Instance, Pin, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_time::{Duration, Timer, with_timeout};

use super::wire;
use crate::bus::{Bus, BusError, ResponseBuf};

/// The clock rate a new bus runs at: the identification limit, 400 kHz or just below.
pub const IDENTIFICATION_HZ: u32 = 400_000;

/// Time added to every computed bus time before it counts as a timeout.
const SLACK_US: u64 = 1_000;

/// Header, start word, 128 data words and two CRC words of the largest write.
const STAGE_WORDS: usize = 132;

/// `out null, 32`, side 0.
const OUT_NULL_32: u16 = InstructionOperands::OUT {
    destination: OutDestination::NULL,
    bit_count: 32,
}
.encode();

/// Pad settings of the bus pins. The defaults are embassy-rp's own: 4 mA, slow slew.
#[derive(Debug)]
pub struct PioBusConfig {
    /// Drive strength of CLK.
    pub clk_drive: Drive,
    /// Slew rate of CLK.
    pub clk_slew: SlewRate,
    /// Drive strength of CMD and DAT0-3 while the host drives them.
    pub line_drive: Drive,
    /// Slew rate of CMD and DAT0-3 while the host drives them.
    pub line_slew: SlewRate,
}

impl Default for PioBusConfig {
    fn default() -> Self {
        Self {
            clk_drive: Drive::_4mA,
            clk_slew: SlewRate::Slow,
            line_drive: Drive::_4mA,
            line_slew: SlewRate::Slow,
        }
    }
}

/// Which line(s) OUT, SET, IN and JMP PIN address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Mapping {
    /// CMD only, one pin.
    Cmd,
    /// DAT0-3; JMP PIN is DAT0.
    Dat,
}

/// The four-bit SD bus on one PIO state machine and one DMA channel. See [the module
/// documentation](super) for the program, the timing and the constraints on sharing the PIO
/// block.
pub struct PioBus<'d, PIO: Instance, const SM: usize> {
    pub(super) sm: StateMachine<'d, PIO, SM>,
    pub(super) dma: dma::Channel<'d>,
    cmd_cfg: Config<'d, PIO>,
    dat_cfg: Config<'d, PIO>,
    mapping: Mapping,
    /// Absolute address of `hdr`, the program origin.
    hdr: u8,
    /// Absolute address of `wait1`.
    wait1: u8,
    /// Microseconds per 1024 SD clocks at the current rate, rounded up.
    us_per_kclk: u64,
    stage: [u32; STAGE_WORDS],
    resp: [u32; 17],
}

impl<'d, PIO: Instance, const SM: usize> PioBus<'d, PIO, SM> {
    /// Loads the program into `common`, takes over `sm`, `dma` and the six pins, and parks the bus
    /// idle (CLK low, CMD and DAT released) at [`IDENTIFICATION_HZ`].
    ///
    /// DAT0-3 must be consecutive GPIOs. Either all six pins are below 32 or all are 16 or above
    /// (the GPIOBASE window); a mix panics.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: dma::Channel<'d>,
        clk: Peri<'d, impl PioPin + 'd>,
        cmd: Peri<'d, impl PioPin + 'd>,
        dat0: Peri<'d, impl PioPin + 'd>,
        dat1: Peri<'d, impl PioPin + 'd>,
        dat2: Peri<'d, impl PioPin + 'd>,
        dat3: Peri<'d, impl PioPin + 'd>,
        config: PioBusConfig,
    ) -> Self {
        let prg = embassy_rp::pio::program::pio_asm!(
            ".side_set 1",
            ".define H 2",
            ".define L 4",
            ".wrap_target",
            "hdr:",
            "    out x, 16          side 0",
            "    out y, 16          side 0",
            "    jmp x-- drive      side 0",
            "    jmp rx             side 0",
            "drive:",
            "    set pindirs, 15    side 0",
            "tx:",
            "    out pins, 4        side 0 [L-1]",
            "    jmp x-- tx         side 1 [H-1]",
            "    mov pins, !null    side 0 [L-1]",
            "    set pindirs, 0     side 1 [H-1]",
            "rx:",
            "    mov x, y           side 0",
            "    jmp x-- wait0      side 0",
            "    jmp done           side 0",
            "wait0:",
            "    nop                side 0 [L-1]",
            "    jmp pin wait0      side 1 [H-1]",
            "rd:",
            "    nop                side 0 [L-1]",
            "    in pins, 4         side 1 [H-2]",
            "    jmp x-- rd         side 1",
            "public wait1:",
            "    nop                side 0 [L-1]",
            "    jmp pin done       side 1 [H-1]",
            "    jmp wait1          side 1",
            "done:",
            "    push               side 0",
            ".wrap",
        );
        let loaded = common.load_program(&prg.program);
        let hdr = loaded.origin;
        let wait1 = (hdr + prg.public_defines.wait1 as u8) % 32;

        let mut clk = common.make_pio_pin(clk);
        let mut lines: [Pin<'d, PIO>; 5] = [
            common.make_pio_pin(cmd),
            common.make_pio_pin(dat0),
            common.make_pio_pin(dat1),
            common.make_pio_pin(dat2),
            common.make_pio_pin(dat3),
        ];
        clk.set_drive_strength(config.clk_drive);
        clk.set_slew_rate(config.clk_slew);
        for line in &mut lines {
            line.set_drive_strength(copy_drive(&config.line_drive));
            line.set_slew_rate(copy_slew(&config.line_slew));
        }
        let [cmd, d0, d1, d2, d3] = &lines;
        let dat = [d0, d1, d2, d3];

        // The PIO sees GPIO `base + n` as pin `n`; set_config picks the same base.
        let base = if core::iter::once(&clk)
            .chain(lines.iter())
            .any(|p| p.pin() >= 32)
        {
            16
        } else {
            0
        };

        let clk_sys = embassy_rp::clocks::clk_sys_freq();
        let (div, _) = wire::clock_divider(clk_sys, IDENTIFICATION_HZ);

        let mut common_cfg = Config::default();
        common_cfg.use_program(&loaded, &[&clk]);
        common_cfg.clock_divider = div.into();
        common_cfg.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        common_cfg.shift_in = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        let mut cmd_cfg = common_cfg;
        cmd_cfg.set_out_pins(&[cmd]);
        cmd_cfg.set_set_pins(&[cmd]);
        cmd_cfg.set_in_pins(&[cmd]);
        set_jmp_pin(&mut cmd_cfg, cmd.pin() - base);

        let mut dat_cfg = common_cfg;
        dat_cfg.set_out_pins(&dat);
        dat_cfg.set_set_pins(&dat);
        dat_cfg.set_in_pins(&dat);
        set_jmp_pin(&mut dat_cfg, d0.pin() - base);

        sm.set_enable(false);
        sm.set_config(&cmd_cfg);
        // Latched levels: lines high (so `set pindirs, 15` drives idle), CLK low.
        sm.set_pins(Level::High, &[cmd, d0, d1, d2, d3]);
        sm.set_pins(Level::Low, &[&clk]);
        sm.set_pin_dirs(Direction::In, &[cmd, d0, d1, d2, d3]);
        sm.set_pin_dirs(Direction::Out, &[&clk]);
        // INPUT_SYNC_BYPASS is indexed like every other PIO pin number: relative to GPIOBASE.
        let bypass = lines.iter().fold(0u32, |m, p| m | 1 << (p.pin() - base));
        common.set_input_sync_bypass(bypass, bypass);

        let mut bus = Self {
            sm,
            dma,
            cmd_cfg,
            dat_cfg,
            mapping: Mapping::Cmd,
            hdr,
            wait1,
            us_per_kclk: wire::micros_per_1024_clocks(div, clk_sys),
            stage: [0; STAGE_WORDS],
            resp: [0; 17],
        };
        bus.abort_to_idle();
        bus
    }

    /// Switches the pin mapping while parked. A no-op when it is already selected.
    pub(super) fn select(&mut self, mapping: Mapping) {
        if self.mapping != mapping {
            let cfg = match mapping {
                Mapping::Cmd => &self.cmd_cfg,
                Mapping::Dat => &self.dat_cfg,
            };
            self.sm.set_enable(false);
            // Also exec-jumps to the origin, `hdr`, where the state machine was parked anyway.
            self.sm.set_config(cfg);
            self.sm.set_enable(true);
            self.mapping = mapping;
        }
    }

    /// Forces the bus idle: CLK low, the mapped lines released, FIFOs and shift registers empty,
    /// parked at `hdr`. Any DMA transfer must already be dropped.
    pub(super) fn abort_to_idle(&mut self) {
        self.sm.set_enable(false);
        // SAFETY: the state machine is disabled; `set pindirs, 0` (side 0) only releases the
        // mapped lines and drives CLK low.
        unsafe { self.sm.set_pindir(0) };
        // Clears the ISR and both shift counters; a zero output count means a full OSR.
        self.sm.restart();
        self.sm.clear_fifos();
        // SAFETY: after the restart the output shift count is 0, so this OUT cannot stall; it
        // empties the OSR so that the next `out x, 16` pulls a fresh header.
        unsafe { self.sm.exec_instr(OUT_NULL_32) };
        // SAFETY: `hdr` is the program origin, where every operation starts.
        unsafe { self.sm.exec_jmp(self.hdr) };
        self.sm.set_enable(true);
    }

    /// The time `clocks` SD clocks take at the current rate (rounded up), plus the slack.
    pub(super) fn bus_time(&self, clocks: u32) -> Duration {
        Duration::from_micros(((u64::from(clocks) * self.us_per_kclk) >> 10) + 1 + SLACK_US)
    }

    /// Sends `stage[..len]` to the TX FIFO by DMA. Done when the last word is in the FIFO.
    async fn push_stage(
        &mut self,
        len: usize,
        bswap: bool,
        timeout: Duration,
    ) -> Result<(), BusError> {
        #[allow(clippy::indexing_slicing)]
        // bound: `len` comes from `wire::stage_*`, which wrote `stage[..len]`, so len <= STAGE_WORDS
        let words = &self.stage[..len];
        let transfer = self.sm.tx().dma_push(&mut self.dma, words, bswap);
        with_timeout(timeout, transfer)
            .await
            .map_err(|_| BusError::Timeout)
    }

    /// Takes one word from the RX FIFO. A word already there is taken without arming a timer.
    pub(super) async fn pull_word(&mut self, timeout: Duration) -> Result<u32, BusError> {
        if let Some(word) = self.sm.rx().try_pull() {
            return Ok(word);
        }
        with_timeout(timeout, self.sm.rx().wait_pull())
            .await
            .map_err(|_| BusError::Timeout)
    }

    /// Runs `words * 8` clocks with CMD high (plus the end nibble), nothing received.
    async fn idle_op(&mut self, words: usize) -> Result<(), BusError> {
        let len = wire::stage_idle(&mut self.stage, words);
        let timeout = self.bus_time(8 * words as u32 + 8);
        self.push_stage(len, false, timeout).await?;
        self.pull_word(timeout).await.map(drop)
    }
}

/// Puts the raw JMP_PIN field. `Config::set_jmp_pin` would store the absolute GPIO number, but
/// the hardware counts it from GPIOBASE like every other pin selection.
fn set_jmp_pin<PIO: Instance>(cfg: &mut Config<'_, PIO>, pin: u8) {
    let mut exec = cfg.get_exec();
    exec.jmp_pin = pin;
    // SAFETY: only the JMP PIN condition source changes; wrap and side-set settings are the ones
    // `use_program` wrote.
    unsafe { cfg.set_exec(exec) };
}

fn copy_drive(d: &Drive) -> Drive {
    match d {
        Drive::_2mA => Drive::_2mA,
        Drive::_4mA => Drive::_4mA,
        Drive::_8mA => Drive::_8mA,
        Drive::_12mA => Drive::_12mA,
    }
}

fn copy_slew(s: &SlewRate) -> SlewRate {
    match s {
        SlewRate::Fast => SlewRate::Fast,
        SlewRate::Slow => SlewRate::Slow,
    }
}

fn millis(ms: u32) -> Duration {
    Duration::from_millis(u64::from(ms))
}

/// One operation in progress. Dropped without [`Op::done`] (an error, a timeout or a dropped
/// future) it forces the bus idle.
pub(super) struct Op<'a, 'd, PIO: Instance, const SM: usize> {
    pub(super) bus: &'a mut PioBus<'d, PIO, SM>,
    ok: bool,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Op<'a, 'd, PIO, SM> {
    pub(super) fn new(bus: &'a mut PioBus<'d, PIO, SM>) -> Self {
        Self { bus, ok: false }
    }

    pub(super) fn done(mut self) {
        self.ok = true;
    }
}

impl<PIO: Instance, const SM: usize> Drop for Op<'_, '_, PIO, SM> {
    fn drop(&mut self) {
        if !self.ok {
            self.bus.abort_to_idle();
        }
    }
}

fn check_block_len(words: usize) {
    assert!(
        matches!(words, 1 | 2 | 16 | 128),
        "data block of {} words",
        words
    );
}

impl<PIO: Instance, const SM: usize> Bus for PioBus<'_, PIO, SM> {
    fn set_clock(&mut self, max_hz: u32) -> u32 {
        let clk_sys = embassy_rp::clocks::clk_sys_freq();
        let (div, rate) = wire::clock_divider(clk_sys, max_hz);
        self.us_per_kclk = wire::micros_per_1024_clocks(div, clk_sys);
        self.cmd_cfg.clock_divider = div.into();
        self.dat_cfg.clock_divider = div.into();
        self.sm.set_clock_divider(div.into());
        rate
    }

    async fn clock_idle(&mut self, cycles: u32) {
        let op = Op::new(self);
        op.bus.select(Mapping::Cmd);
        let mut words = cycles.div_ceil(8) as usize;
        while words > 0 {
            let n = words.min(STAGE_WORDS - 1);
            if op.bus.idle_op(n).await.is_err() {
                return;
            }
            words -= n;
        }
        op.done();
    }

    async fn command(
        &mut self,
        frame: &[u8; 6],
        response: ResponseBuf<'_>,
    ) -> Result<(), BusError> {
        let recv = match response {
            ResponseBuf::None => 0,
            ResponseBuf::Short(_) => wire::SHORT_RECV_NIBBLES,
            ResponseBuf::Long(_) => wire::LONG_RECV_NIBBLES,
        };
        let op = Op::new(self);
        let bus = &mut *op.bus;
        bus.select(Mapping::Cmd);
        let len = wire::stage_command(&mut bus.stage, frame, recv);
        let frame_time = bus.bus_time(u32::from(wire::CMD_SEND_NIBBLES) + 8);
        let out: &mut [u8] = match response {
            ResponseBuf::None => {
                bus.push_stage(len, false, frame_time).await?;
                bus.pull_word(frame_time).await?;
                // A command without a response is followed by at least eight clocks (bus.rs).
                bus.idle_op(1).await?;
                op.done();
                return Ok(());
            }
            ResponseBuf::Short(out) => out,
            ResponseBuf::Long(out) => out,
        };
        bus.push_stage(len, false, frame_time).await?;
        // From here on the state machine needs nothing from the CPU until its final word: at
        // most 4 FIFO words and the OSR of the frame remain, then N_CR (at most 64 clocks), the
        // response and one more clock.
        let reply_time = bus.bus_time(40 + 8 + 64 + u32::from(recv) + 8);
        let words = wire::recv_words(recv);
        bus.pull_response(words, reply_time).await?;
        #[allow(clippy::indexing_slicing)]
        // bound: words = recv_words(recv) <= 17 = resp.len() for recv = 47 or 135
        let received = &bus.resp[..words];
        wire::unpack_response(received, out);
        op.done();
        Ok(())
    }

    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError> {
        let w = data.len();
        check_block_len(w);
        let op = Op::new(self);
        let bus = &mut *op.bus;
        bus.select(Mapping::Dat);
        let data_time = bus.bus_time(8 * w as u32 + 16 + 8) + millis(timeout_ms);
        {
            let (rx, tx) = bus.sm.rx_tx();
            // Armed before the header goes in: the data words go straight to `data`.
            let transfer = rx.dma_pull(&mut bus.dma, data, true);
            tx.push(wire::read_header(w));
            with_timeout(data_time, transfer)
                .await
                .map_err(|_| BusError::Timeout)?;
        }
        // CRC high, CRC low and the final word; they fit in the FIFO, so they are already there
        // or arrive within 17 clocks.
        let tail = bus.bus_time(16 + 8);
        let hi = bus.pull_word(tail).await?;
        let lo = bus.pull_word(tail).await?;
        bus.pull_word(tail).await?;
        op.done();
        Ok(wire::crc_from_words(hi, lo))
    }

    async fn write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError> {
        let w = data.len();
        check_block_len(w);
        let op = Op::new(self);
        let bus = &mut *op.bus;
        bus.select(Mapping::Dat);
        let len = wire::stage_write(&mut bus.stage, data, crc);
        let stream_time = bus.bus_time(8 + 8 * w as u32 + 16 + 8);
        bus.push_stage(len, true, stream_time).await?;
        // The rest of the stream, the CRC status, then DAT0 busy for at most `timeout_ms`.
        let last = bus.pull_word(stream_time + millis(timeout_ms)).await?;
        op.done();
        Ok(wire::token(last))
    }

    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError> {
        let op = Op::new(self);
        let bus = &mut *op.bus;
        bus.select(Mapping::Dat);
        // SAFETY: the state machine is parked at `hdr` with empty FIFOs and OSR; `wait1` clocks
        // until DAT0 is high and ends at `done` with one final word, like every operation.
        unsafe { bus.sm.exec_jmp(bus.wait1) };
        bus.pull_word(bus.bus_time(8) + millis(timeout_ms)).await?;
        op.done();
        Ok(())
    }

    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(millis(ms)).await;
    }
}

impl<PIO: Instance, const SM: usize> PioBus<'_, PIO, SM> {
    /// Receives the `words` words of a response into `resp` by DMA.
    async fn pull_response(&mut self, words: usize, timeout: Duration) -> Result<(), BusError> {
        #[allow(clippy::indexing_slicing)]
        // bound: the only caller passes words = recv_words(recv) <= 17 = resp.len()
        let resp = &mut self.resp[..words];
        let transfer = self.sm.rx().dma_pull(&mut self.dma, resp, false);
        with_timeout(timeout, transfer)
            .await
            .map_err(|_| BusError::Timeout)
    }
}
