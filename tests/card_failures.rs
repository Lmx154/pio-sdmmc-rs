//! One test per failure path: the returned error, no rule violations, and a clean next operation.

mod card_helpers;

use card_helpers::*;
use embassy_futures::block_on;
use pio_sdmmc::card::{Block, Error};
use pio_sdmmc::proto::{CardState, CardStatus, CrcStatus};
use pio_sdmmc::sim::{Fault, Match, Trigger, status};
use pio_sdmmc::{Bus, BusError};

#[test]
fn flipped_data_bit_on_a_read() {
    for (lba, n) in [(40u32, 1usize), (40, 4)] {
        let mut card = default_ready();
        for i in 0..n as u32 {
            card.bus().poke_block(lba + i, &pattern(i as u8 + 1));
        }
        // The second block read in a request, or the only one.
        let hit = n.min(2) as u32;
        card.bus()
            .inject(Fault::FlipDataBit { bit: 100 }, Trigger::block_read(hit));
        let mut out = vec![Block::new(); n];
        let err = block_on(card.read_blocks(lba, &mut out)).unwrap_err();
        let cmd = if n == 1 { 17 } else { 18 };
        match err {
            Error::DataCrc {
                cmd: c,
                lba: Some(l),
                expected,
                got,
            } => {
                assert_eq!(c, cmd);
                assert_eq!(l, lba + hit - 1);
                assert_ne!(expected, got);
            }
            other => panic!("{other}"),
        }
        // The blocks before the bad one are good.
        for (i, b) in out.iter().take((hit - 1) as usize).enumerate() {
            assert_eq!(b.as_bytes(), &pattern(i as u8 + 1)[..]);
        }
        assert_eq!(card.stats().read_crc_errors, 1);
        assert_eq!(card.stats().recoveries, 1);
        assert_eq!(card.bus().violations(), &[]);
        // The next ordinary operation works.
        block_on(card.read_blocks(lba, &mut out)).unwrap();
        for (i, b) in out.iter().enumerate() {
            assert_eq!(b.as_bytes(), &pattern(i as u8 + 1)[..]);
        }
        assert_eq!(card.bus().violations(), &[]);
    }
}

#[test]
fn error_token_on_a_multi_block_write() {
    for cmd23 in [true, false] {
        let mut card = ready(pio_sdmmc::sim::CardKind::Sdxc, cmd23);
        let data = blocks(20, 5);
        // The third block of the request is refused.
        card.bus()
            .inject(Fault::DataCrcErrorToken, Trigger::block_write(3));
        let err = block_on(card.write_blocks(500, &data)).unwrap_err();
        let cmd = 25;
        assert_eq!(
            err,
            Error::WriteRejected {
                cmd,
                lba: 502,
                token: CrcStatus::CrcError
            },
            "cmd23={cmd23}"
        );
        for i in 0..2u32 {
            assert_eq!(
                card.bus().peek_block(500 + i),
                pattern(20 + i as u8),
                "cmd23={cmd23}"
            );
        }
        for i in 2..5u32 {
            assert_eq!(card.bus().peek_block(500 + i), [0u8; 512], "cmd23={cmd23}");
        }
        assert_eq!(card.stats().write_rejections, 1);
        assert_eq!(card.bus().violations(), &[], "cmd23={cmd23}");
        block_on(card.write_blocks(500, &data)).unwrap();
        for (i, b) in data.iter().enumerate() {
            assert_eq!(card.bus().peek_block(500 + i as u32), b.as_bytes());
        }
        assert_eq!(card.bus().violations(), &[], "cmd23={cmd23}");
    }
}

#[test]
fn busy_longer_than_the_write_timeout() {
    let mut card = default_ready();
    // 600 ms of busy against a 500 ms limit: the write times out, the card finishes during
    // clean-up.
    card.bus().inject(
        Fault::BusyAfterWrite { us: 600_000 },
        Trigger::block_write(1),
    );
    let err = block_on(card.write_blocks(9, &[blk(1)])).unwrap_err();
    assert_eq!(
        err,
        Error::Bus {
            cmd: 24,
            lba: Some(9),
            error: BusError::Timeout
        }
    );
    assert_eq!(card.stats().timeouts, 1);
    assert_eq!(card.bus().violations(), &[]);
    block_on(card.write_blocks(10, &[blk(2)])).unwrap();
    assert_eq!(card.bus().peek_block(10), pattern(2));
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn card_stuck_busy_is_reported_as_unexpected_state_until_it_recovers() {
    let mut card = default_ready();
    card.bus().inject(
        Fault::BusyAfterWrite { us: 3_000_000 },
        Trigger::block_write(1),
    );
    let err = block_on(card.write_blocks(9, &[blk(1)])).unwrap_err();
    assert_eq!(
        err,
        Error::UnexpectedState {
            cmd: 13,
            lba: Some(9),
            state: Some(CardState::Prg)
        }
    );
    // The abandoned operation is cleaned up at the start of the next call, once the card is free.
    block_on(card.bus().delay_ms(5_000));
    block_on(card.write_blocks(10, &[blk(2)])).unwrap();
    assert_eq!(card.bus().peek_block(10), pattern(2));
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn status_error_bit_raised_by_a_read() {
    let mut card = default_ready();
    card.bus().poke_block(30, &pattern(5));
    card.bus().inject(
        Fault::StatusError {
            mask: status::CARD_ECC_FAILED,
        },
        Trigger::block_read(1),
    );
    let mut b = [Block::new()];
    let err = block_on(card.read_blocks(30, &mut b)).unwrap_err();
    match err {
        Error::CardStatus {
            cmd: 13,
            lba: Some(30),
            status,
        } => assert_eq!(status.errors(), CardStatus::CARD_ECC_FAILED),
        other => panic!("{other}"),
    }
    assert_eq!(card.bus().violations(), &[]);
    block_on(card.read_blocks(30, &mut b)).unwrap();
    assert_eq!(b[0].as_bytes(), &pattern(5)[..]);
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn card_removed_mid_transfer_then_reinserted() {
    let mut card = default_ready();
    for i in 0..4u32 {
        card.bus().poke_block(60 + i, &pattern(i as u8));
    }
    card.bus().inject(Fault::RemoveCard, Trigger::block_read(2));
    let mut out = vec![Block::new(); 4];
    let err = block_on(card.read_blocks(60, &mut out)).unwrap_err();
    assert!(
        matches!(err, Error::UnexpectedState { state: None, .. }),
        "{err}"
    );
    assert_eq!(out[0].as_bytes(), &pattern(0)[..]);
    // Still gone: the next call says so too, quickly.
    assert!(matches!(
        block_on(card.read_blocks(60, &mut out)),
        Err(Error::UnexpectedState { .. })
    ));
    card.bus().reinsert();
    block_on(card.init()).unwrap();
    block_on(card.read_blocks(60, &mut out)).unwrap();
    for (i, b) in out.iter().enumerate() {
        assert_eq!(b.as_bytes(), &pattern(i as u8)[..]);
    }
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn no_card_at_all() {
    let mut card = fresh(pio_sdmmc::sim::CardKind::Sdxc, true);
    card.bus()
        .inject(Fault::RemoveCard, Trigger::new(Match::AnyCommand, 1));
    assert_eq!(block_on(card.init()), Err(Error::NoCard));
    assert_eq!(card.bus().violations(), &[]);
    card.bus().reinsert();
    block_on(card.init()).unwrap();
}
