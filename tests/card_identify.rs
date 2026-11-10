//! Identification of each simulated card kind, and the ways it can fail.

mod card_helpers;

use card_helpers::*;
use embassy_futures::block_on;
use pio_sdmmc::card::{CapacityClass, Config, Error, SdCard, UnsupportedReason};
use pio_sdmmc::sim::{CardKind, Fault, SimCard, TraceEvent, Trigger};

/// Clock in force at each event of the trace, checked against the rule "400 kHz or less until
/// CMD3 has been answered, 25 MHz or less always"; returns the clock after the last event.
fn check_clock_profile(trace: &[pio_sdmmc::sim::TraceEntry]) -> u32 {
    let mut clock = 400_000;
    let mut after_cmd3 = false;
    let mut raised = false;
    for e in trace {
        match &e.event {
            TraceEvent::SetClock { actual_hz, .. } => {
                clock = *actual_hz;
                assert!(clock <= 25_000_000);
                if clock > 400_000 {
                    assert!(after_cmd3, "clock raised before CMD3");
                    raised = true;
                }
            }
            TraceEvent::Command { index, .. } => {
                if !after_cmd3 {
                    assert!(clock <= 400_000, "CMD{index} at {clock} Hz");
                }
                if *index == 3 {
                    after_cmd3 = true;
                }
            }
            _ => {}
        }
    }
    assert!(raised);
    clock
}

#[test]
fn default_card_is_identified() {
    let mut card = fresh(CardKind::Sdxc, true);
    let info = block_on(card.init()).unwrap();
    assert_eq!(card.info(), Some(&info));
    assert_eq!(info.capacity_class, CapacityClass::Extended);
    assert_eq!(info.capacity_blocks, 124_735_488);
    assert_eq!(info.rca, RCA);
    assert!(info.ocr.power_up_done() && info.ocr.ccs());
    assert_eq!(&info.cid.product_name, b"SK64G");
    assert_eq!(&info.cid.oem_id, b"SD");
    assert_eq!(info.cid.serial, 0xB183_C2B8);
    assert_eq!(info.csd.capacity_blocks, 124_735_488);
    assert!(info.scr.supports_4bit());
    assert!(info.uses_cmd23);
    assert_eq!(info.clock_hz, 25_000_000);
    assert!(card.bus().is_four_bit());
    assert_eq!(card.bus().clock_hz(), 25_000_000);
    assert_eq!(card.bus().violations(), &[]);
    assert_eq!(check_clock_profile(card.bus().trace()), 25_000_000);
    assert_eq!(
        card.stats().commands_sent as usize,
        indices(card.bus().trace()).len()
    );
    assert_eq!(card.stats().timeouts, 0);
}

#[test]
fn default_card_command_sequence() {
    let mut card = fresh(CardKind::Sdxc, true);
    block_on(card.init()).unwrap();
    let seq = indices(card.bus().trace());
    // CMD0, CMD8, three ACMD41 polls (the card is ready on the third), CMD2, CMD3, CMD9, CMD7,
    // ACMD42, ACMD6, ACMD51.
    assert_eq!(
        seq,
        [
            0, 8, 55, 41, 55, 41, 55, 41, 2, 3, 9, 7, 55, 42, 55, 6, 55, 51
        ]
    );
    let cmds = commands(card.bus().trace());
    assert_eq!(cmds[1], (8, 0x1AA));
    assert_eq!(
        cmds[3],
        (41, 0x40FF_8000),
        "HCS set because CMD8 was answered"
    );
    assert_eq!(cmds[11], (7, u32::from(RCA) << 16));
    assert_eq!(cmds[13], (42, 0));
    assert_eq!(cmds[15], (6, 2));
    // Power-up clocks come before the first command.
    let first_cmd = card
        .bus()
        .trace()
        .iter()
        .position(|e| matches!(e.event, TraceEvent::Command { .. }))
        .unwrap();
    let idle: u32 = card.bus().trace()[..first_cmd]
        .iter()
        .filter_map(|e| match e.event {
            TraceEvent::ClockIdle { cycles } => Some(cycles),
            _ => None,
        })
        .sum();
    assert!(idle >= 74);
}

#[test]
fn every_card_kind_identifies() {
    let cases = [
        (CardKind::SdscV1, CapacityClass::Standard, 2_097_152u64),
        (CardKind::SdscV2, CapacityClass::Standard, 2_097_152),
        (CardKind::Sdhc, CapacityClass::High, 15_523_840),
        (CardKind::Sdxc, CapacityClass::Extended, 124_735_488),
    ];
    for (kind, class, blocks) in cases {
        for cmd23 in [true, false] {
            let mut card = fresh(kind, cmd23);
            let info = block_on(card.init()).unwrap_or_else(|e| panic!("{kind:?}: {e}"));
            assert_eq!(info.capacity_class, class, "{kind:?}");
            assert_eq!(info.capacity_blocks, blocks, "{kind:?}");
            assert_eq!(info.rca, RCA);
            assert_eq!(info.uses_cmd23, cmd23, "{kind:?}");
            assert_eq!(info.clock_hz, 25_000_000);
            assert_eq!(info.ocr.ccs(), class != CapacityClass::Standard);
            assert!(card.bus().is_four_bit());
            assert_eq!(card.bus().violations(), &[], "{kind:?}");
            check_clock_profile(card.bus().trace());

            let seq = indices(card.bus().trace());
            let has_cmd8 = seq.contains(&8);
            assert!(has_cmd8);
            let acmd41_args: Vec<u32> = commands(card.bus().trace())
                .iter()
                .filter(|c| c.0 == 41)
                .map(|c| c.1)
                .collect();
            let hcs = kind != CardKind::SdscV1;
            assert!(
                acmd41_args.iter().all(|a| (a & (1 << 30) != 0) == hcs),
                "{kind:?} HCS"
            );
            assert!(acmd41_args.iter().all(|a| a & 0x00FF_8000 == 0x00FF_8000));
            // CMD16 only for standard capacity, and only after CMD7.
            let cmd16 = commands(card.bus().trace()).into_iter().find(|c| c.0 == 16);
            assert_eq!(
                cmd16,
                if class == CapacityClass::Standard {
                    Some((16, 512))
                } else {
                    None
                },
                "{kind:?}"
            );
            let pos = |i: u8| seq.iter().position(|&x| x == i);
            assert!(pos(7) < pos(42) && pos(42) < pos(6) && pos(6) < pos(51));
            if class == CapacityClass::Standard {
                assert!(pos(7) < pos(16) && pos(16) < pos(42));
            }
        }
    }
}

#[test]
fn cmd8_timeout_means_a_version_1_card() {
    let mut card = fresh(CardKind::SdscV1, true);
    block_on(card.init()).unwrap();
    assert_eq!(card.stats().timeouts, 1);
    // The ILLEGAL_COMMAND the silent CMD8 left behind is not held against the card.
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn init_can_be_repeated() {
    let mut card = default_ready();
    let first = *card.info().unwrap();
    block_on(card.write_blocks(5, &[blk(1)])).unwrap();
    let second = block_on(card.init()).unwrap();
    assert_eq!(first, second);
    assert_eq!(card.bus().violations(), &[]);
    assert_eq!(card.bus().peek_block(5), pattern(1));
    let mut b = [Block::new()];
    block_on(card.read_blocks(5, &mut b)).unwrap();
    assert_eq!(b[0].as_bytes(), &pattern(1)[..]);
}

use pio_sdmmc::card::Block;

#[test]
fn acmd41_polling_stops_at_init_timeout() {
    for timeout in [0u32, 5, 10, 300, 1000] {
        let config = Config {
            init_timeout_ms: timeout,
            ..Config::default()
        };
        let mut card = SdCard::new(SimCard::default(), config);
        card.bus()
            .inject(Fault::Acmd41NeverReady, Trigger::command(41, 1).forever());
        let err = block_on(card.init()).unwrap_err();
        assert_eq!(err, Error::InitTimeout { waited_ms: timeout });
        assert!(card.info().is_none());
        let delays: Vec<u32> = card
            .bus()
            .trace()
            .iter()
            .filter_map(|e| match e.event {
                TraceEvent::DelayMs { ms } => Some(ms),
                _ => None,
            })
            .collect();
        assert_eq!(delays.iter().sum::<u32>(), timeout, "delays {delays:?}");
        assert!(delays.iter().all(|&d| d <= 10));
        // Virtual time: the delays plus the few commands, nothing else.
        let elapsed_us = card.bus().now_us();
        assert!(elapsed_us >= u64::from(timeout) * 1000, "{elapsed_us}");
        // Plus the polls themselves: CMD55 and ACMD41 at 400 kHz take well under a millisecond.
        let polls = count_cmd(card.bus().trace(), 41) as u64;
        assert!(
            elapsed_us < u64::from(timeout) * 1000 + polls * 1000 + 5_000,
            "{elapsed_us}"
        );
        assert_eq!(card.bus().violations(), &[]);
    }
}

#[test]
fn no_card_at_all() {
    let mut card = SdCard::new(SimCard::default(), Config::default());
    card.bus().inject(
        Fault::RemoveCard,
        Trigger::new(pio_sdmmc::sim::Match::AnyCommand, 1),
    );
    assert_eq!(block_on(card.init()), Err(Error::NoCard));
    assert!(card.info().is_none());
    assert_eq!(count_cmd(card.bus().trace(), 55), 3);
    assert_eq!(card.stats().timeouts, 4);
    assert_eq!(card.bus().violations(), &[]);
    assert!(card.bus().now_us() < 100_000);
    assert_eq!(block_on(card.status()), Err(Error::NotInitialised));
}

#[test]
fn removed_card_is_found_again_after_reinsert() {
    let mut card = SdCard::new(SimCard::default(), Config::default());
    card.bus().inject(
        Fault::RemoveCard,
        Trigger::new(pio_sdmmc::sim::Match::AnyCommand, 1),
    );
    assert_eq!(block_on(card.init()), Err(Error::NoCard));
    card.bus().reinsert();
    block_on(card.init()).unwrap();
    assert_eq!(card.bus().violations(), &[]);
}

#[test]
fn nothing_works_before_init() {
    let mut card = fresh(CardKind::Sdxc, true);
    let mut b = [Block::new()];
    assert_eq!(
        block_on(card.read_blocks(0, &mut b)),
        Err(Error::NotInitialised)
    );
    assert_eq!(
        block_on(card.write_blocks(0, &b)),
        Err(Error::NotInitialised)
    );
    assert_eq!(block_on(card.status()), Err(Error::NotInitialised));
    assert!(card.info().is_none());
    assert!(card.bus().trace().is_empty());
    assert_eq!(*card.stats(), Default::default());
}

#[test]
fn cmd8_with_wrong_echo_or_voltage_is_an_unsupported_card() {
    fn wrong_pattern(index: u8, frame: &mut [u8; 6]) {
        if index == 8 {
            *frame = r7(1, 0x55);
        }
    }
    fn wrong_voltage(index: u8, frame: &mut [u8; 6]) {
        if index == 8 {
            *frame = r7(2, 0xAA);
        }
    }
    let bus = Tamper {
        sim: SimCard::default(),
        rewrite: wrong_pattern,
    };
    let mut card = SdCard::new(bus, Config::default());
    assert_eq!(
        block_on(card.init()),
        Err(Error::Unsupported {
            cmd: 8,
            reason: UnsupportedReason::CheckPattern {
                sent: 0xAA,
                got: 0x55
            }
        })
    );
    let bus = Tamper {
        sim: SimCard::default(),
        rewrite: wrong_voltage,
    };
    let mut card = SdCard::new(bus, Config::default());
    assert_eq!(
        block_on(card.init()),
        Err(Error::Unsupported {
            cmd: 8,
            reason: UnsupportedReason::Voltage { accepted: 2 }
        })
    );
    assert!(card.bus().sim.violations().is_empty());
}
