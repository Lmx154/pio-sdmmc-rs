//! The card's side of the protocol: command handling, data transfers and virtual-time costs.

use super::crc::{crc7, data_crc};
use super::fault::status::{
    ADDRESS_ERROR, BLOCK_LEN_ERROR, COM_CRC_ERROR, ERASE_PARAM, ERASE_SEQ_ERROR, ERROR, ERROR_MASK,
    ILLEGAL_COMMAND, OUT_OF_RANGE,
};
use super::fault::{Event, Fault};
use super::trace::{CardState, ResponseClass, TraceEvent, Violation};
use super::{Pending, ReadSource, ReadXfer, SimCard, WriteXfer, regs};
use crate::bus::{BusError, ResponseBuf};

/// Cycles from the end of a command frame to the start of its response.
const N_CR_TYPICAL: u64 = 2;
/// Cycles the bus clocks after a frame that has no response.
const NO_RESPONSE_CLOCKS: u64 = 8;
/// Cycles a host waits for a response start bit before giving up.
const N_CR_MAX: u64 = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Cmd {
    Std(u8),
    App(u8),
}

#[derive(Debug)]
enum Reply {
    Silent,
    R1,
    R1b,
    R2([u8; 16]),
    R3(u32),
    R6(u16),
    R7(u32),
}

enum Legal {
    Yes,
    Unknown,
    WrongState,
}

pub(super) struct Outcome {
    response: Option<Vec<u8>>,
    busy_us: u64,
}

fn class_of(buf: &ResponseBuf<'_>) -> ResponseClass {
    match buf {
        ResponseBuf::None => ResponseClass::None,
        ResponseBuf::Short(_) => ResponseClass::Short,
        ResponseBuf::Long(_) => ResponseClass::Long,
    }
}

fn is_acmd(index: u8) -> bool {
    matches!(index, 6 | 13 | 22 | 23 | 41 | 42 | 51)
}

fn expected_class(cmd: Cmd, deselect: bool) -> Option<ResponseClass> {
    match cmd {
        Cmd::Std(0) => Some(ResponseClass::None),
        Cmd::Std(2 | 9 | 10) => Some(ResponseClass::Long),
        Cmd::Std(7) => Some(if deselect {
            ResponseClass::None
        } else {
            ResponseClass::Short
        }),
        Cmd::Std(3 | 8 | 12 | 13 | 16 | 17 | 18 | 23 | 24 | 25 | 32 | 33 | 38 | 55)
        | Cmd::App(6 | 13 | 22 | 23 | 41 | 42 | 51) => Some(ResponseClass::Short),
        _ => None,
    }
}

fn short_frame(first: u8, payload: [u8; 4]) -> Vec<u8> {
    let mut f = [first, payload[0], payload[1], payload[2], payload[3], 0];
    f[5] = (crc7(&f[..5]) << 1) | 1;
    f.to_vec()
}

fn bytes_to_words(bytes: &[u8], words: &mut [u32]) {
    for (w, c) in words.iter_mut().zip(bytes.as_chunks::<4>().0) {
        *w = u32::from_ne_bytes(*c);
    }
}

fn words_to_bytes(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|w| w.to_ne_bytes()).collect()
}

impl SimCard {
    fn my_rca(&self) -> u16 {
        match self.state {
            CardState::Idle | CardState::Ready | CardState::Ident => 0,
            _ => self.cfg.rca,
        }
    }

    /// Puts the card in the idle state with everything volatile cleared. Storage is kept.
    pub(super) fn reset_card(&mut self) {
        self.state = CardState::Idle;
        self.busy_until = self.now_us;
        self.status_err = 0;
        self.app_cmd = false;
        self.four_bit = false;
        self.cmd8_seen = false;
        self.acmd41_started_at = None;
        self.acmd41_polls = 0;
        self.block_count = None;
        self.pending = Pending::None;
        self.write_failed = false;
        self.erase_start = None;
        self.erase_end = None;
    }

    fn lba_from_arg(&self, arg: u32) -> Result<u32, u32> {
        if self.cfg.kind.is_high_capacity() {
            if arg >= self.cfg.capacity_blocks {
                Err(OUT_OF_RANGE)
            } else {
                Ok(arg)
            }
        } else if !arg.is_multiple_of(512) {
            Err(ADDRESS_ERROR)
        } else if arg / 512 >= self.cfg.capacity_blocks {
            Err(OUT_OF_RANGE)
        } else {
            Ok(arg / 512)
        }
    }

    fn legality(&self, cmd: Cmd, st: CardState) -> Legal {
        use CardState::*;
        const ALL: &[CardState] = &[Idle, Ready, Ident, Stby, Tran, Data, Rcv, Prg, Dis];
        let allowed: &[CardState] = match cmd {
            Cmd::Std(0) | Cmd::Std(55) => ALL,
            Cmd::Std(2) => &[Ready],
            Cmd::Std(3) => &[Ident, Stby],
            Cmd::Std(7) => &[Stby, Tran, Data, Prg, Dis],
            Cmd::Std(8) if self.cfg.kind.answers_cmd8() => &[Idle],
            Cmd::Std(9 | 10) => &[Stby],
            Cmd::Std(12) => &[Data, Rcv],
            Cmd::Std(13) => &[Stby, Tran, Data, Rcv, Prg, Dis],
            Cmd::Std(16 | 17 | 18 | 23 | 24 | 25 | 32 | 33 | 38) => &[Tran],
            Cmd::App(6 | 13 | 22 | 23 | 42 | 51) => &[Tran],
            Cmd::App(41) => &[Idle],
            _ => return Legal::Unknown,
        };
        if allowed.contains(&st) {
            Legal::Yes
        } else {
            Legal::WrongState
        }
    }

    pub(super) fn do_command(
        &mut self,
        frame: &[u8; 6],
        response: ResponseBuf<'_>,
    ) -> Result<(), BusError> {
        let at = self.now_us;
        let index = frame[0] & 0x3F;
        let arg = u32::from_be_bytes([frame[1], frame[2], frame[3], frame[4]]);
        let given = class_of(&response);
        self.check_clock();
        if !self.first_command_seen {
            self.first_command_seen = true;
            if self.idle_cycles < 74 {
                self.violate(Violation::CommandBeforePowerUpClocks {
                    idle_cycles: self.idle_cycles,
                });
            }
        }
        let outcome = self.process_command(frame, index, arg, given);
        let cycles = match (&outcome.response, given) {
            (_, ResponseClass::None) => 48 + NO_RESPONSE_CLOCKS,
            (Some(b), _) => 48 + N_CR_TYPICAL + 8 * b.len() as u64,
            (None, _) => 48 + N_CR_MAX,
        };
        self.advance_cycles(cycles);
        if outcome.busy_us > 0 {
            self.busy_until = self.busy_until.max(self.now_us) + outcome.busy_us;
        }
        let result = match (response, outcome.response.as_deref()) {
            (ResponseBuf::None, _) => Ok(()),
            (ResponseBuf::Short(buf), Some(b)) if b.len() == 6 => {
                buf.copy_from_slice(b);
                Ok(())
            }
            (ResponseBuf::Long(buf), Some(b)) if b.len() == 17 => {
                buf.copy_from_slice(b);
                Ok(())
            }
            _ => Err(BusError::Timeout),
        };
        self.log(
            at,
            TraceEvent::Command {
                frame: *frame,
                index,
                arg,
                response: outcome.response,
                result,
            },
        );
        result
    }

    fn process_command(
        &mut self,
        frame: &[u8; 6],
        index: u8,
        arg: u32,
        given: ResponseClass,
    ) -> Outcome {
        let silent = Outcome {
            response: None,
            busy_us: 0,
        };
        if self.removed {
            return silent;
        }
        if frame[0] & 0xC0 != 0x40 {
            self.violate(Violation::MalformedFrame { frame: *frame });
            return silent;
        }
        if frame[5] & 1 != 1 || crc7(&frame[..5]) != frame[5] >> 1 {
            self.status_err |= COM_CRC_ERROR;
            return silent;
        }
        let fired = self.fire(Event::Command(index));
        if fired.contains(&Fault::RemoveCard) {
            self.removed = true;
            return silent;
        }
        self.settle();
        let st0 = self.state;
        let now = self.now_us;

        let cmd = if self.app_cmd && is_acmd(index) {
            Cmd::App(index)
        } else {
            Cmd::Std(index)
        };
        let deselect = cmd == Cmd::Std(7) && (arg >> 16) as u16 != self.my_rca();
        let basic = matches!(cmd, Cmd::Std(0 | 12 | 13));
        if matches!(st0, CardState::Idle | CardState::Ready | CardState::Ident)
            && self.clock_hz > 400_000
        {
            self.violate(Violation::CommandClockAbove400k {
                index,
                state: st0,
                hz: self.clock_hz,
            });
        }
        let remaining = self.busy_remaining_us();
        if remaining > 0 && !basic && !deselect {
            self.violate(Violation::CommandWhileBusy {
                index,
                remaining_us: remaining,
            });
        }
        if self.pending != Pending::None && !basic {
            self.violate(Violation::CommandWhileTransferPending { index });
        }
        if let Some(expected) = expected_class(cmd, deselect)
            && expected != given
        {
            self.violate(Violation::ResponseBufferMismatch {
                index,
                expected,
                given,
            });
        }

        let mut busy_us = 0u64;
        let mut never_ready = false;
        for f in &fired {
            match *f {
                Fault::StatusError { mask } => self.status_err |= mask,
                Fault::BusyAfterCommand { us } => busy_us += us,
                Fault::Acmd41NeverReady => never_ready = true,
                _ => {}
            }
        }

        let reply = self.execute(cmd, arg, st0, now, never_ready, &mut busy_us);
        let (mut response, clear) = self.build_response(&reply, index, st0, now);
        if !matches!(cmd, Cmd::Std(55)) || matches!(reply, Reply::Silent) {
            self.app_cmd = false;
        }
        let mut sent = response.is_some();
        for f in &fired {
            match *f {
                Fault::NoResponse => {
                    response = None;
                    sent = false;
                }
                Fault::FlipResponseBit { bit } => {
                    if let Some(r) = response.as_mut() {
                        let bit = bit as usize % (r.len() * 8);
                        #[allow(clippy::indexing_slicing)]
                        // bound: bit < r.len() * 8 after the modulo
                        let byte = &mut r[bit / 8];
                        *byte ^= 0x80 >> (bit % 8);
                    }
                }
                _ => {}
            }
        }
        if sent {
            self.status_err &= !clear;
        }
        Outcome { response, busy_us }
    }

    fn build_response(
        &self,
        reply: &Reply,
        index: u8,
        st0: CardState,
        now: u64,
    ) -> (Option<Vec<u8>>, u32) {
        let ready = u32::from(self.busy_until <= now);
        let status = (self.status_err & ERROR_MASK)
            | ((st0 as u32) << 9)
            | (ready << 8)
            | (u32::from(self.app_cmd) << 5);
        match reply {
            Reply::Silent => (None, 0),
            Reply::R1 | Reply::R1b => (Some(short_frame(index, status.to_be_bytes())), ERROR_MASK),
            Reply::R2(reg) => {
                let mut f = vec![0x3F];
                f.extend_from_slice(reg);
                (Some(f), 0)
            }
            Reply::R3(ocr) => {
                let o = ocr.to_be_bytes();
                (Some(vec![0x3F, o[0], o[1], o[2], o[3], 0xFF]), 0)
            }
            Reply::R6(rca) => {
                let s16 = (((status >> 23) & 1) << 15)
                    | (((status >> 22) & 1) << 14)
                    | (((status >> 19) & 1) << 13)
                    | (status & 0x1FFF);
                let r = rca.to_be_bytes();
                let s = (s16 as u16).to_be_bytes();
                (
                    Some(short_frame(3, [r[0], r[1], s[0], s[1]])),
                    COM_CRC_ERROR | ILLEGAL_COMMAND | ERROR,
                )
            }
            Reply::R7(echo) => (Some(short_frame(8, echo.to_be_bytes())), 0),
        }
    }

    fn illegal(&mut self) -> Reply {
        self.status_err |= ILLEGAL_COMMAND;
        Reply::Silent
    }

    fn execute(
        &mut self,
        cmd: Cmd,
        arg: u32,
        st0: CardState,
        now: u64,
        never_ready: bool,
        busy_us: &mut u64,
    ) -> Reply {
        if !matches!(cmd, Cmd::Std(18 | 23 | 25 | 55)) {
            self.block_count = None;
        }
        let (index, app) = match cmd {
            Cmd::Std(i) => (i, false),
            Cmd::App(i) => (i, true),
        };
        if cmd == Cmd::Std(23) && !self.cfg.supports_cmd23 {
            self.violate(Violation::Cmd23NotSupported);
            return self.illegal();
        }
        match self.legality(cmd, st0) {
            Legal::Unknown => return self.illegal(),
            Legal::WrongState => {
                self.violate(Violation::CommandInWrongState {
                    index,
                    app,
                    state: st0,
                });
                return self.illegal();
            }
            Legal::Yes => {}
        }
        let rca_ok = (arg >> 16) as u16 == self.cfg.rca;
        let hc = self.cfg.kind.is_high_capacity();

        match cmd {
            Cmd::Std(0) => {
                self.reset_card();
                Reply::Silent
            }
            Cmd::Std(2) => {
                self.state = CardState::Ident;
                Reply::R2(regs::cid(&self.cfg.cid))
            }
            Cmd::Std(3) => {
                self.state = CardState::Stby;
                Reply::R6(self.cfg.rca)
            }
            Cmd::Std(7) => match (st0, rca_ok) {
                (CardState::Stby, true) => {
                    self.state = CardState::Tran;
                    *busy_us += self.cfg.cmd7_busy_us;
                    Reply::R1b
                }
                (CardState::Dis, true) => {
                    self.state = CardState::Prg;
                    *busy_us += self.cfg.cmd7_busy_us;
                    Reply::R1b
                }
                (CardState::Stby | CardState::Dis, false) => Reply::Silent,
                (CardState::Tran | CardState::Data, false) => {
                    self.pending = Pending::None;
                    self.state = CardState::Stby;
                    Reply::Silent
                }
                (CardState::Prg, false) => {
                    self.state = CardState::Dis;
                    Reply::Silent
                }
                _ => self.illegal(),
            },
            Cmd::Std(8) => {
                if (arg >> 8) & 0xF != 1 {
                    return Reply::Silent;
                }
                self.cmd8_seen = true;
                Reply::R7(arg & 0xFFF)
            }
            Cmd::Std(9) => {
                if !rca_ok {
                    return Reply::Silent;
                }
                Reply::R2(regs::csd(&self.cfg))
            }
            Cmd::Std(10) => {
                if !rca_ok {
                    return Reply::Silent;
                }
                Reply::R2(regs::cid(&self.cfg.cid))
            }
            Cmd::Std(12) => {
                self.state = if self.pending_is_write() {
                    CardState::Prg
                } else {
                    CardState::Tran
                };
                self.pending = Pending::None;
                self.write_failed = false;
                *busy_us += self.cfg.cmd12_busy_us;
                Reply::R1b
            }
            Cmd::Std(13) => {
                if !rca_ok {
                    return Reply::Silent;
                }
                Reply::R1
            }
            Cmd::Std(16) => {
                if !hc && arg != 512 {
                    self.status_err |= BLOCK_LEN_ERROR;
                }
                Reply::R1
            }
            Cmd::Std(17 | 18) => {
                match self.lba_from_arg(arg) {
                    Err(bit) => self.status_err |= bit,
                    Ok(lba) => {
                        let remaining = if index == 17 {
                            Some(1)
                        } else {
                            self.block_count.take()
                        };
                        self.pending = Pending::Read(ReadXfer {
                            source: ReadSource::Blocks,
                            next: lba,
                            remaining,
                        });
                        self.state = CardState::Data;
                    }
                }
                self.block_count = None;
                Reply::R1
            }
            Cmd::Std(23) => {
                let count = arg & 0xFFFF;
                if count == 0 {
                    self.violate(Violation::ZeroBlockCount);
                    self.block_count = None;
                } else {
                    self.block_count = Some(count);
                }
                Reply::R1
            }
            Cmd::Std(24 | 25) => {
                match self.lba_from_arg(arg) {
                    Err(bit) => self.status_err |= bit,
                    Ok(lba) => {
                        let single = index == 24;
                        let remaining = if single {
                            Some(1)
                        } else {
                            self.block_count.take()
                        };
                        self.pending = Pending::Write(WriteXfer {
                            next: lba,
                            remaining,
                            single,
                        });
                        self.state = CardState::Rcv;
                        self.last_write_ok = 0;
                        self.write_failed = false;
                    }
                }
                self.block_count = None;
                Reply::R1
            }
            Cmd::Std(32) => {
                match self.lba_from_arg(arg) {
                    Err(bit) => self.status_err |= bit,
                    Ok(lba) => self.erase_start = Some(lba),
                }
                Reply::R1
            }
            Cmd::Std(33) => {
                match self.lba_from_arg(arg) {
                    Err(bit) => self.status_err |= bit,
                    Ok(lba) => self.erase_end = Some(lba),
                }
                Reply::R1
            }
            Cmd::Std(38) => {
                let range = (self.erase_start.take(), self.erase_end.take());
                match range {
                    (Some(start), Some(end)) if start <= end => {
                        self.blocks.retain(|k, _| *k < start || *k > end);
                        self.state = CardState::Prg;
                        *busy_us += self.cfg.erase_busy_us;
                        Reply::R1b
                    }
                    (Some(_), Some(_)) => {
                        self.status_err |= ERASE_PARAM;
                        Reply::R1
                    }
                    _ => {
                        self.status_err |= ERASE_SEQ_ERROR;
                        Reply::R1
                    }
                }
            }
            Cmd::Std(55) => {
                if !matches!(st0, CardState::Idle | CardState::Ready | CardState::Ident) && !rca_ok
                {
                    return Reply::Silent;
                }
                self.app_cmd = true;
                Reply::R1
            }
            Cmd::App(6) => {
                match arg & 3 {
                    0 => self.four_bit = false,
                    2 => self.four_bit = true,
                    _ => self.status_err |= ERROR,
                }
                Reply::R1
            }
            Cmd::App(13) => self.start_special_read(ReadSource::SdStatus),
            Cmd::App(22) => self.start_special_read(ReadSource::WrCount),
            Cmd::App(23) => {
                self.pre_erase_count = arg & 0x7F_FFFF;
                Reply::R1
            }
            Cmd::App(41) => self.acmd41(arg, now, never_ready),
            Cmd::App(42) => {
                self.card_detect_connected = arg & 1 == 1;
                Reply::R1
            }
            Cmd::App(51) => self.start_special_read(ReadSource::Scr),
            _ => self.illegal(),
        }
    }

    fn pending_is_write(&self) -> bool {
        matches!(self.pending, Pending::Write(_))
    }

    fn start_special_read(&mut self, source: ReadSource) -> Reply {
        self.pending = Pending::Read(ReadXfer {
            source,
            next: 0,
            remaining: Some(1),
        });
        self.state = CardState::Data;
        Reply::R1
    }

    fn acmd41(&mut self, arg: u32, now: u64, never_ready: bool) -> Reply {
        let window = arg & 0x00FF_8000;
        let hcs = arg & (1 << 30) != 0;
        if window == 0 {
            return Reply::R3(self.cfg.ocr & 0x00FF_8000);
        }
        if hcs && !self.cmd8_seen {
            self.violate(Violation::HcsWithoutCmd8);
        }
        let hcs_effective = hcs && self.cmd8_seen;
        let started = *self.acmd41_started_at.get_or_insert(now);
        self.acmd41_polls = self.acmd41_polls.saturating_add(1);
        let hc = self.cfg.kind.is_high_capacity();
        let ready = !never_ready
            && self.acmd41_polls > self.cfg.acmd41_polls
            && now - started >= u64::from(self.cfg.acmd41_ms) * 1000
            && window & self.cfg.ocr != 0
            && (!hc || hcs_effective);
        if ready {
            self.state = CardState::Ready;
            let mut ocr = (self.cfg.ocr & 0x00FF_FFFF) | 0x8000_0000;
            if hc && hcs_effective {
                ocr |= 1 << 30;
            }
            Reply::R3(ocr)
        } else {
            Reply::R3(self.cfg.ocr & 0x00FF_FFFF)
        }
    }

    fn read_source_bytes(&self, r: &ReadXfer) -> Vec<u8> {
        match r.source {
            ReadSource::Blocks => self.peek_block(r.next).to_vec(),
            ReadSource::Scr => regs::scr(&self.cfg).to_vec(),
            ReadSource::SdStatus => regs::sd_status(self.four_bit).to_vec(),
            ReadSource::WrCount => self.last_write_ok.to_be_bytes().to_vec(),
        }
    }

    pub(super) fn do_read_block(
        &mut self,
        data: &mut [u32],
        timeout_ms: u32,
    ) -> Result<u64, BusError> {
        let at = self.now_us;
        let words = data.len();
        self.check_clock();
        let mut lba = None;
        let result = self.read_inner(data, timeout_ms, &mut lba);
        self.log(
            at,
            TraceEvent::ReadBlock {
                lba,
                words,
                timeout_ms,
                result,
            },
        );
        result
    }

    fn read_inner(
        &mut self,
        data: &mut [u32],
        timeout_ms: u32,
        lba_out: &mut Option<u32>,
    ) -> Result<u64, BusError> {
        let timeout_us = u64::from(timeout_ms) * 1000;
        if self.removed {
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        }
        self.settle();
        let Pending::Read(r) = self.pending else {
            self.violate(Violation::DataWithoutCommand { write: false });
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        };
        let mut bytes = self.read_source_bytes(&r);
        if r.source == ReadSource::Blocks {
            *lba_out = Some(r.next);
        }
        if data.len() * 4 != bytes.len() {
            self.violate(Violation::BlockLengthMismatch {
                write: false,
                expected_words: bytes.len() / 4,
                got_words: data.len(),
            });
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        }
        if !self.four_bit {
            self.violate(Violation::DataBeforeFourBit { write: false });
        }
        if self.is_busy() {
            self.violate(Violation::DataWhileBusy { write: false });
        }
        let fired = self.fire(Event::Read(*lba_out));
        if fired.contains(&Fault::RemoveCard) {
            self.removed = true;
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        }
        for f in &fired {
            if let Fault::StatusError { mask } = *f {
                self.status_err |= mask;
            }
        }
        if r.source == ReadSource::Blocks && r.next >= self.cfg.capacity_blocks {
            self.status_err |= OUT_OF_RANGE;
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        }
        let mut crc = data_crc(&bytes);
        for f in &fired {
            match *f {
                Fault::FlipDataBit { bit } => {
                    let bit = bit as usize % (bytes.len() * 8);
                    #[allow(clippy::indexing_slicing)]
                    // bound: bit < bytes.len() * 8 after the modulo
                    let byte = &mut bytes[bit / 8];
                    *byte ^= 1 << (bit % 8);
                }
                Fault::FlipCrcBit { bit } => crc ^= 1u64 << (bit % 64),
                _ => {}
            }
        }
        bytes_to_words(&bytes, data);
        self.advance_cycles(20 + 2 * bytes.len() as u64);
        let remaining = r.remaining.map(|n| n - 1);
        if remaining == Some(0) {
            self.pending = Pending::None;
            self.state = CardState::Tran;
        } else {
            self.pending = Pending::Read(ReadXfer {
                next: r.next + 1,
                remaining,
                ..r
            });
        }
        Ok(crc)
    }

    pub(super) fn do_write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError> {
        let at = self.now_us;
        let words = data.len();
        self.check_clock();
        let mut lba = None;
        let result = self.write_inner(data, crc, timeout_ms, &mut lba);
        self.log(
            at,
            TraceEvent::WriteBlock {
                lba,
                words,
                crc,
                timeout_ms,
                result,
            },
        );
        result
    }

    fn write_inner(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
        lba_out: &mut Option<u32>,
    ) -> Result<u8, BusError> {
        const WRITE_CLOCKS: u64 = 2 + 1 + 2 * 512 + 16 + 1 + 2 + 5;
        let timeout_us = u64::from(timeout_ms) * 1000;
        if self.removed {
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        }
        self.settle();
        let Pending::Write(w) = self.pending else {
            self.violate(Violation::DataWithoutCommand { write: true });
            self.advance_cycles(20 + 2 * (data.len() as u64 * 4));
            return Err(BusError::Timeout);
        };
        *lba_out = Some(w.next);
        if self.write_failed {
            self.violate(Violation::WriteAfterErrorToken);
            self.advance_cycles(20 + 2 * (data.len() as u64 * 4));
            return Err(BusError::Timeout);
        }
        if data.len() != 128 {
            self.violate(Violation::BlockLengthMismatch {
                write: true,
                expected_words: 128,
                got_words: data.len(),
            });
            self.advance_cycles(20 + 2 * (data.len() as u64 * 4));
            return Err(BusError::Timeout);
        }
        if !self.four_bit {
            self.violate(Violation::DataBeforeFourBit { write: true });
        }
        if self.is_busy() {
            self.violate(Violation::DataWhileBusy { write: true });
        }
        let fired = self.fire(Event::Write(w.next));
        if fired.contains(&Fault::RemoveCard) {
            self.removed = true;
            self.advance(timeout_us);
            return Err(BusError::Timeout);
        }
        let mut busy_override = None;
        for f in &fired {
            match *f {
                Fault::StatusError { mask } => self.status_err |= mask,
                Fault::BusyAfterWrite { us } => busy_override = Some(us),
                _ => {}
            }
        }
        let bytes = words_to_bytes(data);
        let crc_error = data_crc(&bytes) != crc || fired.contains(&Fault::DataCrcErrorToken);
        let out_of_range = w.next >= self.cfg.capacity_blocks;
        let write_error = fired.contains(&Fault::WriteErrorToken);
        self.advance_cycles(WRITE_CLOCKS);
        let token = if crc_error {
            0b101
        } else if out_of_range || write_error {
            self.status_err |= if out_of_range { OUT_OF_RANGE } else { ERROR };
            0b110
        } else {
            0b010
        };
        if token != 0b010 {
            if w.single {
                self.pending = Pending::None;
                self.state = CardState::Tran;
            } else {
                self.write_failed = true;
            }
            return Ok(token);
        }
        let mut block = [0u8; 512];
        block.copy_from_slice(&bytes);
        self.blocks.insert(w.next, block);
        self.last_write_ok = self.last_write_ok.saturating_add(1);
        self.total_written += 1;
        let every = u64::from(self.cfg.long_pause_every_blocks);
        let busy =
            busy_override.unwrap_or(if every > 0 && self.total_written.is_multiple_of(every) {
                self.cfg.long_pause_us
            } else {
                self.cfg.write_busy_us
            });
        let remaining = w.remaining.map(|n| n - 1);
        if remaining == Some(0) {
            self.pending = Pending::None;
            self.state = CardState::Prg;
        } else {
            self.pending = Pending::Write(WriteXfer {
                next: w.next + 1,
                remaining,
                ..w
            });
        }
        self.busy_until = self.now_us + busy;
        if busy > timeout_us {
            self.advance(timeout_us);
            self.settle();
            return Err(BusError::Timeout);
        }
        self.advance(busy);
        self.settle();
        Ok(token)
    }
}
