//! Driver-level tests for the simulator fault kinds, and the unsupported-card reasons, that
//! `card_failures.rs` and `card_identify.rs` do not reach. Every test states where the fault is
//! injected, asserts the exact error, that the simulated card kept to the protocol, that nothing
//! was reported as written that was not, and that the next operation works.

mod card_helpers;

use card_helpers::*;
use embassy_futures::block_on;
use pio_sdmmc::bus::{as_bytes, as_bytes_mut};
use pio_sdmmc::card::{Block, Config, Error, SdCard, UnsupportedReason};
use pio_sdmmc::proto::{
    CardState, CardStatus, CrcStatus, RegisterError, ResponseError, crc7, crc16_4bit,
};
use pio_sdmmc::sim::{Fault, SimCard, Trigger, status};
use pio_sdmmc::{Bus, BusError, ResponseBuf};

/// A bus that rewrites responses and data on their way back, a limited number of times.
struct Mangle {
    sim: SimCard,
    budget: u32,
    short: fn(u8, &mut [u8; 6]) -> bool,
    long: fn(u8, &mut [u8; 17]) -> bool,
    data: fn(&mut [u32]) -> bool,
}

fn no_short(_: u8, _: &mut [u8; 6]) -> bool {
    false
}
fn no_long(_: u8, _: &mut [u8; 17]) -> bool {
    false
}
fn no_data(_: &mut [u32]) -> bool {
    false
}

impl Mangle {
    fn new(budget: u32) -> Self {
        Mangle {
            sim: SimCard::new(sim_cfg(pio_sdmmc::sim::CardKind::Sdxc, true)),
            budget,
            short: no_short,
            long: no_long,
            data: no_data,
        }
    }
}

impl Bus for Mangle {
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
        let index = frame[0] & 0x3F;
        match response {
            ResponseBuf::Short(buf) => {
                let mut tmp = [0u8; 6];
                self.sim
                    .command(frame, ResponseBuf::Short(&mut tmp))
                    .await?;
                if self.budget > 0 && (self.short)(index, &mut tmp) {
                    self.budget -= 1;
                }
                *buf = tmp;
                Ok(())
            }
            ResponseBuf::Long(buf) => {
                let mut tmp = [0u8; 17];
                self.sim.command(frame, ResponseBuf::Long(&mut tmp)).await?;
                if self.budget > 0 && (self.long)(index, &mut tmp) {
                    self.budget -= 1;
                }
                *buf = tmp;
                Ok(())
            }
            other => self.sim.command(frame, other).await,
        }
    }

    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError> {
        let crc = self.sim.read_block(data, timeout_ms).await?;
        if self.budget > 0 && (self.data)(data) {
            self.budget -= 1;
            return Ok(crc16_4bit(as_bytes(data)));
        }
        Ok(crc)
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

fn reseal_short(f: &mut [u8; 6]) {
    f[5] = (crc7(&f[..5]) << 1) | 1;
}

fn reseal_long(f: &mut [u8; 17]) {
    f[16] = (crc7(&f[1..16]) << 1) | 1;
}

/// CMD13 reports the card as deselected (CURRENT_STATE 3), everything else as it was.
fn state_stby(index: u8, f: &mut [u8; 6]) -> bool {
    if index != 13 {
        return false;
    }
    f[3] = (f[3] & !0x1E) | (3 << 1);
    reseal_short(f);
    true
}

fn rca_zero(index: u8, f: &mut [u8; 6]) -> bool {
    if index != 3 {
        return false;
    }
    f[1] = 0;
    f[2] = 0;
    reseal_short(f);
    true
}

/// ACMD41 reports the OCR without the capacity bit.
fn ocr_without_ccs(index: u8, f: &mut [u8; 6]) -> bool {
    if index != 41 {
        return false;
    }
    f[1] &= !0x40;
    true
}

/// The CSD says "version 3" (SDUC).
fn csd_sduc(index: u8, f: &mut [u8; 17]) -> bool {
    if index != 9 {
        return false;
    }
    f[1] = (f[1] & 0x3F) | (2 << 6);
    reseal_long(f);
    true
}

/// The CSD says a structure version that does not exist.
fn csd_unknown(index: u8, f: &mut [u8; 17]) -> bool {
    if index != 9 {
        return false;
    }
    f[1] |= 0xC0;
    reseal_long(f);
    true
}

/// The SCR lists no four-bit bus width.
fn scr_no_four_bit(d: &mut [u32]) -> bool {
    if d.len() != 2 {
        return false;
    }
    as_bytes_mut(d)[1] &= 0xF0;
    true
}

fn mangled_ready() -> SdCard<Mangle> {
    let mut card = SdCard::new(Mangle::new(0), Config::default());
    block_on(card.init()).unwrap();
    card
}

fn unsupported(setup: impl FnOnce(&mut Mangle), cmd: u8, reason: UnsupportedReason) {
    let mut bus = Mangle::new(100);
    setup(&mut bus);
    let mut card = SdCard::new(bus, Config::default());
    assert_eq!(
        block_on(card.init()),
        Err(Error::Unsupported { cmd, reason })
    );
    assert!(card.info().is_none());
    assert_eq!(
        block_on(card.read_blocks(0, &mut [Block::new()])),
        Err(Error::NotInitialised)
    );
    assert_eq!(card.bus().sim.violations(), &[]);
    // The documented remedy is init() again; with the card behaving, it works.
    card.bus().budget = 0;
    let info = block_on(card.init()).unwrap();
    assert_eq!(card.info(), Some(&info));
    let mut b = [Block::new()];
    block_on(card.read_blocks(0, &mut b)).unwrap();
    assert_eq!(card.bus().sim.violations(), &[]);
}

#[test]
fn sduc_card_is_unsupported() {
    unsupported(|b| b.long = csd_sduc, 9, UnsupportedReason::Sduc);
}

#[test]
fn unknown_csd_structure_is_unsupported() {
    unsupported(
        |b| b.long = csd_unknown,
        9,
        UnsupportedReason::Csd(RegisterError::UnknownCsdStructure(3)),
    );
}

#[test]
fn ocr_and_csd_disagreeing_on_addressing_is_unsupported() {
    unsupported(
        |b| b.short = ocr_without_ccs,
        9,
        UnsupportedReason::AddressingMismatch,
    );
}

#[test]
fn scr_without_four_bit_is_unsupported() {
    unsupported(
        |b| b.data = scr_no_four_bit,
        6,
        UnsupportedReason::NoFourBit,
    );
}

#[test]
fn card_publishing_rca_zero_is_unsupported() {
    let mut bus = Mangle::new(100);
    bus.short = rca_zero;
    let mut card = SdCard::new(bus, Config::default());
    assert_eq!(
        block_on(card.init()),
        Err(Error::Unsupported {
            cmd: 3,
            reason: UnsupportedReason::RcaZero
        })
    );
    // CMD3 was asked for 4 times, and each answer was rewritten.
    assert_eq!(card.bus().budget, 96);
    assert!(card.info().is_none());
    assert_eq!(card.bus().sim.violations(), &[]);
    card.bus().budget = 0;
    block_on(card.init()).unwrap();
}

#[test]
fn card_in_the_wrong_state_after_a_read() {
    let mut card = mangled_ready();
    card.bus().sim.poke_block(40, &pattern(3));
    card.bus().budget = 1;
    card.bus().short = state_stby;
    let mut b = [Block::new()];
    assert_eq!(
        block_on(card.read_blocks(40, &mut b)),
        Err(Error::WrongState {
            cmd: 13,
            lba: Some(40),
            state: CardState::Stby
        })
    );
    // Clean-up asked again, saw the transfer state, and returned the original error.
    assert_eq!(card.stats().recoveries, 1);
    assert_eq!(card.bus().sim.violations(), &[]);
    block_on(card.read_blocks(40, &mut b)).unwrap();
    assert_eq!(b[0].as_bytes(), &pattern(3)[..]);
    assert_eq!(card.bus().sim.violations(), &[]);
}

#[test]
fn card_in_the_wrong_state_after_a_write() {
    let mut card = mangled_ready();
    card.bus().budget = 1;
    card.bus().short = state_stby;
    assert_eq!(
        block_on(card.write_blocks(9, &[blk(1)])),
        Err(Error::WrongState {
            cmd: 13,
            lba: Some(9),
            state: CardState::Stby
        })
    );
    assert_eq!(card.stats().blocks_written, 0);
    assert_eq!(card.stats().recoveries, 1);
    assert_eq!(card.bus().sim.violations(), &[]);
    block_on(card.write_blocks(9, &[blk(2)])).unwrap();
    assert_eq!(card.bus().sim.peek_block(9), pattern(2));
    assert_eq!(card.stats().blocks_written, 1);
    assert_eq!(card.bus().sim.violations(), &[]);
}

#[test]
fn flipped_response_bit_on_a_read_command() {
    for (bit, n) in [(0u32, 1usize), (1, 1), (2, 1), (20, 1), (47, 1), (47, 4)] {
        let cmd = if n == 1 { 17 } else { 18 };
        let mut card = default_ready();
        for i in 0..n as u32 {
            card.bus().poke_block(40 + i, &pattern(i as u8 + 1));
        }
        card.bus()
            .inject(Fault::FlipResponseBit { bit }, Trigger::command(cmd, 1));
        let mut out = vec![Block::new(); n];
        let err = block_on(card.read_blocks(40, &mut out)).unwrap_err();
        let Error::BadResponse {
            cmd: c,
            lba: Some(40),
            error,
        } = err
        else {
            panic!("bit {bit}: {err}");
        };
        assert_eq!(c, cmd);
        match bit {
            0 => assert_eq!(error, ResponseError::StartBit),
            1 => assert_eq!(error, ResponseError::TransmissionBit),
            47 => assert_eq!(error, ResponseError::EndBit),
            _ => assert!(
                matches!(error, ResponseError::Crc { expected, got } if expected != got),
                "bit {bit}: {error:?}"
            ),
        }
        assert_eq!(
            out,
            vec![Block::new(); n],
            "bit {bit}: nothing was delivered"
        );
        assert_eq!(card.stats().bad_responses, 1);
        assert_eq!(card.stats().recoveries, 1);
        assert_eq!(card.bus().violations(), &[], "bit {bit}");
        block_on(card.read_blocks(40, &mut out)).unwrap();
        for (i, b) in out.iter().enumerate() {
            assert_eq!(b.as_bytes(), &pattern(i as u8 + 1)[..]);
        }
        assert_eq!(card.bus().violations(), &[], "bit {bit}");
    }
}

#[test]
fn flipped_response_bit_on_a_write_command_writes_nothing() {
    for n in [1usize, 3] {
        let cmd = if n == 1 { 24 } else { 25 };
        let mut card = default_ready();
        card.bus()
            .inject(Fault::FlipResponseBit { bit: 30 }, Trigger::command(cmd, 1));
        let data = blocks(10, n);
        let err = block_on(card.write_blocks(9, &data)).unwrap_err();
        assert!(
            matches!(
                err,
                Error::BadResponse {
                    cmd: c,
                    lba: Some(9),
                    error: ResponseError::Crc { .. }
                } if c == cmd
            ),
            "{err}"
        );
        for i in 0..n as u32 {
            assert_eq!(card.bus().peek_block(9 + i), [0u8; 512]);
        }
        assert_eq!(card.stats().blocks_written, 0);
        assert_eq!(card.stats().bad_responses, 1);
        assert_eq!(card.bus().violations(), &[], "n={n}");
        block_on(card.write_blocks(9, &data)).unwrap();
        for (i, b) in data.iter().enumerate() {
            assert_eq!(card.bus().peek_block(9 + i as u32), b.as_bytes());
        }
        assert_eq!(card.stats().blocks_written, n as u32);
        assert_eq!(card.bus().violations(), &[], "n={n}");
    }
}

#[test]
fn flipped_response_bit_during_identification() {
    let mut card = fresh(pio_sdmmc::sim::CardKind::Sdxc, true);
    card.bus()
        .inject(Fault::FlipResponseBit { bit: 0 }, Trigger::command(8, 1));
    assert_eq!(
        block_on(card.init()),
        Err(Error::BadResponse {
            cmd: 8,
            lba: None,
            error: ResponseError::StartBit
        })
    );
    assert!(card.info().is_none());
    assert_eq!(card.stats().bad_responses, 1);
    assert_eq!(card.bus().violations(), &[]);
    // The remedy is init() again; the fault was one-shot.
    block_on(card.init()).unwrap();
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn no_response_to_a_read_command() {
    let mut card = default_ready();
    card.bus().poke_block(40, &pattern(4));
    card.bus()
        .inject(Fault::NoResponse, Trigger::command(17, 1));
    let mut b = [Block::new()];
    assert_eq!(
        block_on(card.read_blocks(40, &mut b)),
        Err(Error::Bus {
            cmd: 17,
            lba: Some(40),
            error: BusError::Timeout
        })
    );
    assert_eq!(b[0], Block::new());
    assert_eq!(card.stats().timeouts, 1);
    assert_eq!(card.bus().violations(), &[]);
    block_on(card.read_blocks(40, &mut b)).unwrap();
    assert_eq!(b[0].as_bytes(), &pattern(4)[..]);
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn flipped_crc_bit_on_a_read() {
    for (lba, n, bit) in [(40u32, 1usize, 0u32), (40, 1, 63), (40, 4, 17), (40, 4, 64)] {
        let mut card = default_ready();
        for i in 0..n as u32 {
            card.bus().poke_block(lba + i, &pattern(i as u8 + 1));
        }
        let hit = n.min(2) as u32;
        card.bus()
            .inject(Fault::FlipCrcBit { bit }, Trigger::block_read(hit));
        let mut out = vec![Block::new(); n];
        let err = block_on(card.read_blocks(lba, &mut out)).unwrap_err();
        let want_cmd = if n == 1 { 17 } else { 18 };
        let bad = lba + hit - 1;
        let good = crc16_4bit(&pattern((hit - 1) as u8 + 1));
        assert_eq!(
            err,
            Error::DataCrc {
                cmd: want_cmd,
                lba: Some(bad),
                expected: good,
                got: good ^ (1u64 << (bit % 64))
            },
            "bit {bit}, {n} block(s)"
        );
        for (i, b) in out.iter().take((hit - 1) as usize).enumerate() {
            assert_eq!(b.as_bytes(), &pattern(i as u8 + 1)[..]);
        }
        assert_eq!(card.stats().read_crc_errors, 1);
        assert_eq!(card.stats().recoveries, 1);
        assert_eq!(card.bus().violations(), &[]);
        block_on(card.read_blocks(lba, &mut out)).unwrap();
        for (i, b) in out.iter().enumerate() {
            assert_eq!(b.as_bytes(), &pattern(i as u8 + 1)[..]);
        }
        assert_eq!(card.bus().violations(), &[]);
    }
}

#[test]
fn write_error_token() {
    for (n, cmd23) in [(1usize, true), (3, true), (3, false)] {
        let mut card = ready(pio_sdmmc::sim::CardKind::Sdxc, cmd23);
        let old = blocks(100, n);
        for (i, b) in old.iter().enumerate() {
            card.bus()
                .poke_block(500 + i as u32, b.as_bytes().try_into().unwrap());
        }
        // The second block of a request (the only one of a single-block write) is refused.
        let hit = n.min(2) as u32;
        card.bus()
            .inject(Fault::WriteErrorToken, Trigger::block_write(hit));
        let data = blocks(20, n);
        let err = block_on(card.write_blocks(500, &data)).unwrap_err();
        assert_eq!(
            err,
            Error::WriteRejected {
                cmd: if n == 1 { 24 } else { 25 },
                lba: 500 + hit - 1,
                token: CrcStatus::WriteError
            },
            "n={n} cmd23={cmd23}"
        );
        for i in 0..n as u32 {
            let want = if i + 1 < hit {
                &data[i as usize]
            } else {
                &old[i as usize]
            };
            assert_eq!(card.bus().peek_block(500 + i), want.as_bytes(), "block {i}");
        }
        assert_eq!(card.stats().write_rejections, 1);
        assert_eq!(card.stats().blocks_written, 0);
        assert_eq!(card.bus().violations(), &[], "n={n} cmd23={cmd23}");
        // The ERROR bit the token raised did not outlive the clean-up.
        assert_eq!(block_on(card.status()).unwrap().errors(), 0);
        block_on(card.write_blocks(500, &data)).unwrap();
        for (i, b) in data.iter().enumerate() {
            assert_eq!(card.bus().peek_block(500 + i as u32), b.as_bytes());
        }
        assert_eq!(card.bus().violations(), &[], "n={n} cmd23={cmd23}");
    }
}

#[test]
fn busy_after_cmd12_shorter_than_the_limit_is_waited_out() {
    let mut card = ready(pio_sdmmc::sim::CardKind::Sdxc, false);
    for i in 0..4u32 {
        card.bus().poke_block(60 + i, &pattern(i as u8));
    }
    card.bus().inject(
        Fault::BusyAfterCommand { us: 100_000 },
        Trigger::command(12, 1),
    );
    let mut out = vec![Block::new(); 4];
    block_on(card.read_blocks(60, &mut out)).unwrap();
    for (i, b) in out.iter().enumerate() {
        assert_eq!(b.as_bytes(), &pattern(i as u8)[..]);
    }
    assert_eq!(card.stats().recoveries, 0);
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn busy_after_cmd12_longer_than_the_limit_on_a_read() {
    let mut card = ready(pio_sdmmc::sim::CardKind::Sdxc, false);
    for i in 0..4u32 {
        card.bus().poke_block(60 + i, &pattern(i as u8));
    }
    card.bus().inject(
        Fault::BusyAfterCommand { us: 600_000 },
        Trigger::command(12, 1),
    );
    let mut out = vec![Block::new(); 4];
    let err = block_on(card.read_blocks(60, &mut out)).unwrap_err();
    assert_eq!(
        err,
        Error::Bus {
            cmd: 12,
            lba: Some(60),
            error: BusError::Timeout
        }
    );
    // The data had all arrived and was checked before the stop.
    for (i, b) in out.iter().enumerate() {
        assert_eq!(b.as_bytes(), &pattern(i as u8)[..]);
    }
    assert_eq!(card.stats().timeouts, 1);
    assert_eq!(card.stats().recoveries, 1);
    assert_eq!(card.bus().violations(), &[]);
    block_on(card.read_blocks(60, &mut out)).unwrap();
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn busy_after_cmd12_longer_than_the_limit_on_a_write() {
    let mut card = ready(pio_sdmmc::sim::CardKind::Sdxc, false);
    let data = blocks(30, 4);
    card.bus().inject(
        Fault::BusyAfterCommand { us: 600_000 },
        Trigger::command(12, 1),
    );
    let err = block_on(card.write_blocks(70, &data)).unwrap_err();
    assert_eq!(
        err,
        Error::Bus {
            cmd: 12,
            lba: Some(70),
            error: BusError::Timeout
        }
    );
    // Not reported as written, though the card did take the blocks.
    assert_eq!(card.stats().blocks_written, 0);
    assert_eq!(card.stats().timeouts, 1);
    assert_eq!(card.bus().violations(), &[]);
    block_on(card.write_blocks(70, &data)).unwrap();
    for (i, b) in data.iter().enumerate() {
        assert_eq!(card.bus().peek_block(70 + i as u32), b.as_bytes());
    }
    assert_eq!(card.stats().blocks_written, 4);
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn status_error_after_a_write() {
    for n in [1usize, 3] {
        let mut card = default_ready();
        card.bus().inject(
            Fault::StatusError {
                mask: status::WP_VIOLATION,
            },
            Trigger::block_write(n as u32),
        );
        let data = blocks(40, n);
        let err = block_on(card.write_blocks(80, &data)).unwrap_err();
        match err {
            Error::CardStatus {
                cmd: 13,
                lba: Some(80),
                status,
            } => assert_eq!(status.errors(), CardStatus::WP_VIOLATION),
            other => panic!("n={n}: {other}"),
        }
        // The write was not confirmed, so none of it is reported as written.
        assert_eq!(card.stats().blocks_written, 0);
        assert_eq!(card.stats().recoveries, 1);
        assert_eq!(card.bus().violations(), &[], "n={n}");
        assert_eq!(block_on(card.status()).unwrap().errors(), 0);
        block_on(card.write_blocks(80, &data)).unwrap();
        for (i, b) in data.iter().enumerate() {
            assert_eq!(card.bus().peek_block(80 + i as u32), b.as_bytes());
        }
        assert_eq!(card.stats().blocks_written, n as u32);
        assert_eq!(card.bus().violations(), &[], "n={n}");
    }
}
