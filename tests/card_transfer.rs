//! Reads and writes on every card kind, with and without CMD23.

mod card_helpers;

use card_helpers::*;
use embassy_futures::block_on;
use pio_sdmmc::card::{Block, Error};
use pio_sdmmc::sim::CardKind;

const KINDS: [CardKind; 3] = [CardKind::SdscV2, CardKind::Sdhc, CardKind::Sdxc];

fn after_init<T>(
    card: &mut pio_sdmmc::card::SdCard<pio_sdmmc::sim::SimCard>,
    f: impl FnOnce(&mut pio_sdmmc::card::SdCard<pio_sdmmc::sim::SimCard>) -> T,
) -> (T, Vec<u8>, Vec<(u8, u32)>) {
    let (out, trace) = traced(card, f);
    (out, indices(&trace), commands(&trace))
}

#[test]
fn single_block_read_and_write() {
    for kind in KINDS {
        let mut card = ready(kind, true);
        card.bus().poke_block(77, &pattern(9));
        let mut b = [Block::new()];
        let (r, seq, cmds) = after_init(&mut card, |c| block_on(c.read_blocks(77, &mut b)));
        r.unwrap();
        assert_eq!(b[0].as_bytes(), &pattern(9)[..], "{kind:?}");
        assert_eq!(seq, [17, 13], "{kind:?}");
        let addr = if kind == CardKind::SdscV2 {
            77 * 512
        } else {
            77
        };
        assert_eq!(cmds[0], (17, addr), "{kind:?}");

        let (r, seq, cmds) = after_init(&mut card, |c| block_on(c.write_blocks(78, &[blk(4)])));
        r.unwrap();
        assert_eq!(card.bus().peek_block(78), pattern(4), "{kind:?}");
        assert_eq!(seq, [24, 13], "{kind:?}");
        assert_eq!(
            cmds[0].1,
            if kind == CardKind::SdscV2 {
                78 * 512
            } else {
                78
            }
        );
        assert_eq!(card.bus().violations(), &[], "{kind:?}");
        assert_eq!(card.stats().blocks_read, 1);
        assert_eq!(card.stats().blocks_written, 1);
    }
}

#[test]
fn multi_block_read_and_write_with_and_without_cmd23() {
    for kind in KINDS {
        for cmd23 in [true, false] {
            let mut card = ready(kind, cmd23);
            let data = blocks(10, 5);
            let (r, seq, cmds) = after_init(&mut card, |c| block_on(c.write_blocks(200, &data)));
            r.unwrap();
            for (i, b) in data.iter().enumerate() {
                assert_eq!(
                    card.bus().peek_block(200 + i as u32),
                    b.as_bytes(),
                    "{kind:?} cmd23={cmd23} block {i}"
                );
            }
            let addr = |lba: u32| {
                if kind == CardKind::SdscV2 {
                    lba * 512
                } else {
                    lba
                }
            };
            if cmd23 {
                assert_eq!(seq, [23, 25, 13], "{kind:?}");
                assert_eq!(cmds[0], (23, 5));
                assert_eq!(cmds[1], (25, addr(200)));
            } else {
                // CMD55 + ACMD23 (pre-erase count), CMD25, blocks, CMD12, CMD13.
                assert_eq!(seq, [55, 23, 25, 12, 13], "{kind:?}");
                assert_eq!(cmds[1], (23, 5));
                assert_eq!(cmds[2], (25, addr(200)));
            }

            for i in 0..5u32 {
                card.bus().poke_block(300 + i, &pattern(50 + i as u8));
            }
            let mut out = vec![Block::new(); 5];
            let (r, seq, cmds) = after_init(&mut card, |c| block_on(c.read_blocks(300, &mut out)));
            r.unwrap();
            for (i, b) in out.iter().enumerate() {
                assert_eq!(
                    b.as_bytes(),
                    &pattern(50 + i as u8)[..],
                    "{kind:?} cmd23={cmd23} block {i}"
                );
            }
            if cmd23 {
                assert_eq!(seq, [23, 18, 13], "{kind:?}");
                assert_eq!(cmds[1], (18, addr(300)));
            } else {
                assert_eq!(seq, [18, 12, 13], "{kind:?}");
                assert_eq!(cmds[0], (18, addr(300)));
            }
            assert_eq!(card.bus().violations(), &[], "{kind:?} cmd23={cmd23}");
            assert_eq!(card.stats().blocks_written, 5);
            assert_eq!(card.stats().blocks_read, 5);
            assert_eq!(card.stats().recoveries, 0);
        }
    }
}

#[test]
fn out_of_range_is_refused_before_any_bus_traffic() {
    for kind in KINDS {
        let mut card = ready(kind, true);
        let cap = card.info().unwrap().capacity_blocks;
        let len0 = card.bus().trace().len();
        let mut two = [Block::new(), Block::new()];
        let mut one = [Block::new()];
        let err = Error::OutOfRange {
            lba: (cap - 1) as u32,
            blocks: 2,
            capacity_blocks: cap,
        };
        assert_eq!(
            block_on(card.read_blocks((cap - 1) as u32, &mut two)),
            Err(err)
        );
        assert_eq!(
            block_on(card.write_blocks((cap - 1) as u32, &two)),
            Err(err)
        );
        assert!(matches!(
            block_on(card.read_blocks(cap as u32, &mut one)),
            Err(Error::OutOfRange { .. })
        ));
        assert!(matches!(
            block_on(card.write_blocks(u32::MAX, &one)),
            Err(Error::OutOfRange { .. })
        ));
        assert_eq!(card.bus().trace().len(), len0, "{kind:?}");
        // The last block itself is fine.
        block_on(card.write_blocks((cap - 1) as u32, &[blk(3)])).unwrap();
        block_on(card.read_blocks((cap - 1) as u32, &mut one)).unwrap();
        assert_eq!(one[0].as_bytes(), &pattern(3)[..]);
        assert_eq!(card.bus().violations(), &[]);
    }
}

#[test]
fn status_reports_the_transfer_state() {
    let mut card = default_ready();
    let s = block_on(card.status()).unwrap();
    assert_eq!(s.errors(), 0);
    assert_eq!(s.state(), pio_sdmmc::proto::CardState::Tran);
}

#[test]
fn block_conversions() {
    let b = Block::new();
    assert_eq!(b, Block::default());
    assert!(b.as_bytes().iter().all(|&x| x == 0) && b.as_bytes().len() == 512);
    let mut c = Block::from(pattern(1));
    assert_eq!(c.as_bytes(), &pattern(1)[..]);
    c.as_bytes_mut()[511] ^= 0xFF;
    assert_ne!(c, Block::from(pattern(1)));
}

#[test]
fn a_request_longer_than_one_cmd23_is_split() {
    let mut card = default_ready();
    let n = 65_535 + 2;
    card.bus().poke_block(0, &pattern(1));
    card.bus().poke_block(n as u32 - 1, &pattern(2));
    let mut out = vec![Block::new(); n];
    let (r, _, cmds) = after_init(&mut card, |c| block_on(c.read_blocks(0, &mut out)));
    r.unwrap();
    assert_eq!(out[0].as_bytes(), &pattern(1)[..]);
    assert_eq!(out[n - 1].as_bytes(), &pattern(2)[..]);
    let c23: Vec<_> = cmds.iter().filter(|c| c.0 == 23).collect();
    assert_eq!(c23, [&(23, 65_535), &(23, 2)]);
    let c18: Vec<_> = cmds.iter().filter(|c| c.0 == 18).collect();
    assert_eq!(c18, [&(18, 0), &(18, 65_535)]);
    assert_eq!(card.bus().violations(), &[]);
}
