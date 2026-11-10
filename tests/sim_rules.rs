//! Each rule the simulator checks is recorded when the host breaks it, and only then.

mod sim_helpers;

use embassy_futures::block_on;
use pio_sdmmc::sim::{CardState, ResponseClass, SimCard, SimConfig, Violation};
use pio_sdmmc::{Bus, BusError, ResponseBuf};
use sim_helpers::*;

fn rca_arg() -> u32 {
    u32::from(RCA) << 16
}

#[test]
fn command_before_74_idle_clocks() {
    block_on(async {
        let mut card = SimCard::default();
        card.set_clock(400_000);
        card.clock_idle(73).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::CommandBeforePowerUpClocks { idle_cycles: 73 }]
        );
        // Only the first command after power-up is checked.
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(card.violations().len(), 1);

        let mut card = SimCard::default();
        card.set_clock(400_000);
        card.clock_idle(74).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(card.violations(), &[]);

        // A fresh power-up (reinsert) starts the count again.
        let mut card = SimCard::default();
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        card.reinsert();
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::CommandBeforePowerUpClocks { idle_cycles: 0 }]
        );
    });
}

#[test]
fn command_above_400k_before_stby() {
    block_on(async {
        let mut card = SimCard::default();
        card.set_clock(400_000);
        card.clock_idle(80).await;
        card.set_clock(25_000_000);
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::CommandClockAbove400k {
                index: 0,
                state: CardState::Idle,
                hz: 25_000_000
            }]
        );

        // In ready and ident states too; in stand-by it is allowed.
        let mut card = SimCard::default();
        identify(&mut card, true).await;
        assert_eq!(card.violations(), &[]);
        card.set_clock(25_000_000);
        cmd(&mut card, 13, rca_arg()).await.unwrap();
        assert_eq!(card.violations(), &[]);

        let mut card = SimCard::default();
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        let _ = cmd(&mut card, 8, 0x1AA).await;
        for _ in 0..100 {
            cmd(&mut card, 55, 0).await.unwrap();
            let r = cmd(&mut card, 41, 0x40FF_8000).await.unwrap();
            if r[1] & 0x80 != 0 {
                break;
            }
            card.delay_ms(10).await;
        }
        assert_eq!(card.state(), CardState::Ready);
        card.set_clock(25_000_000);
        cmd_long(&mut card, 2, 0).await.unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::CommandClockAbove400k {
                index: 2,
                state: CardState::Ready,
                hz: 25_000_000
            }]
        );
        cmd(&mut card, 3, 0).await.unwrap();
        assert_eq!(
            card.violations()[1],
            Violation::CommandClockAbove400k {
                index: 3,
                state: CardState::Ident,
                hz: 25_000_000
            }
        );
        assert_eq!(card.violations().len(), 2);
    });
}

#[test]
fn clock_above_25_mhz_on_any_operation() {
    block_on(async {
        let mut card = SimCard::default();
        assert_eq!(card.set_clock(50_000_000), 50_000_000);
        card.clock_idle(80).await;
        assert_eq!(
            card.violations(),
            &[Violation::ClockAbove25M { hz: 50_000_000 }]
        );
        card.wait_ready(1).await.unwrap();
        assert_eq!(card.violations().len(), 2);
        card.set_clock(25_000_000);
        card.wait_ready(1).await.unwrap();
        assert_eq!(card.violations().len(), 2);
        // delay_ms runs with the clock stopped.
        card.set_clock(50_000_000);
        card.delay_ms(1).await;
        assert_eq!(card.violations().len(), 2);
    });
}

#[test]
fn command_while_busy() {
    block_on(async {
        let mut card = SimCard::default();
        identify(&mut card, true).await;
        card.set_clock(25_000_000);
        cmd(&mut card, 7, rca_arg()).await.unwrap();
        assert!(card.is_busy());
        // CMD13 and CMD0 are allowed while busy.
        cmd(&mut card, 13, rca_arg()).await.unwrap();
        assert_eq!(card.violations(), &[]);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_eq!(s & READY_FOR_DATA, 0);
        cmd(&mut card, 16, 512).await.unwrap();
        match card.violations() {
            [
                Violation::CommandWhileBusy {
                    index: 16,
                    remaining_us,
                },
            ] => assert!(*remaining_us > 0),
            other => panic!("unexpected {other:?}"),
        }
        card.wait_ready(10).await.unwrap();
        cmd(&mut card, 16, 512).await.unwrap();
        assert_eq!(card.violations().len(), 1);
    });
}

#[test]
fn deselect_while_busy_is_allowed() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        cmd(&mut card, 25, 5).await.unwrap();
        send_block(&mut card, &pattern(1)).await.unwrap();
        cmd(&mut card, 12, 0).await.unwrap();
        assert!(card.is_busy());
        cmd_none(&mut card, 7, 0).await.unwrap();
        assert_eq!(card.violations(), &[]);
        assert_eq!(card.state(), CardState::Dis);
    });
}

#[test]
fn block_length_mismatch() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        cmd(&mut card, 17, 0).await.unwrap();
        let mut w = [0u32; 16];
        assert_eq!(card.read_block(&mut w, 10).await, Err(BusError::Timeout));
        assert_eq!(
            card.violations(),
            &[Violation::BlockLengthMismatch {
                write: false,
                expected_words: 128,
                got_words: 16
            }]
        );
        // The pending read survives and a right-sized read still works.
        let mut w = [0u32; 128];
        assert!(card.read_block(&mut w, 10).await.is_ok());
        assert_eq!(card.violations().len(), 1);

        cmd(&mut card, 24, 0).await.unwrap();
        assert_eq!(
            card.write_block(&[0u32; 2], 0, 10).await,
            Err(BusError::Timeout)
        );
        assert_eq!(
            card.violations()[1],
            Violation::BlockLengthMismatch {
                write: true,
                expected_words: 128,
                got_words: 2
            }
        );
        send_block(&mut card, &pattern(0)).await.unwrap();

        // ACMD22 moves 4 bytes, ACMD51 8, ACMD13 64.
        acmd(&mut card, RCA, 51, 0).await.unwrap();
        let mut w = [0u32; 1];
        assert_eq!(card.read_block(&mut w, 10).await, Err(BusError::Timeout));
        assert_eq!(
            card.violations()[2],
            Violation::BlockLengthMismatch {
                write: false,
                expected_words: 2,
                got_words: 1
            }
        );
    });
}

#[test]
fn data_before_four_bit_mode() {
    block_on(async {
        let mut card = SimCard::default();
        identify(&mut card, true).await;
        card.set_clock(25_000_000);
        cmd(&mut card, 7, rca_arg()).await.unwrap();
        card.wait_ready(10).await.unwrap();
        cmd(&mut card, 17, 0).await.unwrap();
        let mut w = [0u32; 128];
        card.read_block(&mut w, 10).await.unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::DataBeforeFourBit { write: false }]
        );
        cmd(&mut card, 24, 0).await.unwrap();
        send_block(&mut card, &pattern(0)).await.unwrap();
        assert_eq!(
            card.violations()[1],
            Violation::DataBeforeFourBit { write: true }
        );
        // ACMD6 back to one bit is a way back, and four-bit again clears the problem.
        acmd(&mut card, RCA, 6, 0).await.unwrap();
        assert!(!card.is_four_bit());
        acmd(&mut card, RCA, 6, 2).await.unwrap();
        cmd(&mut card, 17, 0).await.unwrap();
        card.read_block(&mut w, 10).await.unwrap();
        assert_eq!(card.violations().len(), 2);
    });
}

#[test]
fn command_while_transfer_pending() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        cmd(&mut card, 18, 0).await.unwrap();
        // CMD13 and CMD12 are allowed.
        cmd(&mut card, 13, rca_arg()).await.unwrap();
        assert_eq!(card.violations(), &[]);
        cmd(&mut card, 55, rca_arg()).await.unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::CommandWhileTransferPending { index: 55 }]
        );
        cmd(&mut card, 12, 0).await.unwrap();
        assert_eq!(card.violations().len(), 1);
        card.wait_ready(10).await.unwrap();

        cmd(&mut card, 24, 0).await.unwrap();
        cmd(&mut card, 55, rca_arg()).await.unwrap();
        assert_eq!(
            card.violations()[1],
            Violation::CommandWhileTransferPending { index: 55 }
        );
    });
}

#[test]
fn other_natural_violations() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;

        // read_block / write_block with nothing pending.
        let mut w = [0u32; 128];
        assert!(card.read_block(&mut w, 1).await.is_err());
        assert!(card.write_block(&w, 0, 1).await.is_err());
        assert_eq!(
            card.take_violations(),
            vec![
                Violation::DataWithoutCommand { write: false },
                Violation::DataWithoutCommand { write: true }
            ]
        );

        // A command the card knows, in the wrong state.
        cmd(&mut card, 7, 0).await.unwrap_err();
        assert_eq!(card.state(), CardState::Stby);
        let _ = card.take_violations();
        assert!(cmd(&mut card, 17, 0).await.is_err());
        assert_eq!(
            card.take_violations(),
            vec![Violation::CommandInWrongState {
                index: 17,
                app: false,
                state: CardState::Stby
            }]
        );
        // An unknown command is not a violation (CMD8 probes on a v1 card are legitimate).
        assert!(cmd(&mut card, 41, 0).await.is_err());
        assert_eq!(card.violations(), &[]);

        // Wrong response buffer for the command.
        let mut r = [0u8; 17];
        card.command(&frame(13, rca_arg()), ResponseBuf::Long(&mut r))
            .await
            .unwrap_err();
        assert_eq!(
            card.take_violations(),
            vec![Violation::ResponseBufferMismatch {
                index: 13,
                expected: ResponseClass::Short,
                given: ResponseClass::Long
            }]
        );

        // A frame whose start bit is wrong.
        let mut bad = frame(13, rca_arg());
        bad[0] |= 0x80;
        let mut r6 = [0u8; 6];
        assert!(
            card.command(&bad, ResponseBuf::Short(&mut r6))
                .await
                .is_err()
        );
        assert_eq!(
            card.take_violations(),
            vec![Violation::MalformedFrame { frame: bad }]
        );
    });
}

#[test]
fn hcs_before_cmd8_and_cmd23_misuse() {
    block_on(async {
        let mut card = SimCard::default();
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        cmd(&mut card, 55, 0).await.unwrap();
        cmd(&mut card, 41, 0x40FF_8000).await.unwrap();
        assert_eq!(card.violations(), &[Violation::HcsWithoutCmd8]);

        let cfg = SimConfig {
            supports_cmd23: false,
            ..SimConfig::default()
        };
        let mut card = SimCard::new(cfg);
        bring_up(&mut card).await;
        acmd(&mut card, RCA, 51, 0).await.unwrap();
        let mut w = [0u32; 2];
        card.read_block(&mut w, 10).await.unwrap();
        assert_eq!(to_bytes(&w)[3] & 0x02, 0);
        assert!(cmd(&mut card, 23, 2).await.is_err());
        assert_eq!(card.violations(), &[Violation::Cmd23NotSupported]);

        let mut card = SimCard::default();
        bring_up(&mut card).await;
        cmd(&mut card, 23, 0).await.unwrap();
        assert_eq!(card.violations(), &[Violation::ZeroBlockCount]);
    });
}

#[test]
fn write_block_while_card_still_busy() {
    block_on(async {
        let cfg = SimConfig {
            write_busy_us: 20_000,
            ..SimConfig::default()
        };
        let mut card = SimCard::new(cfg);
        bring_up(&mut card).await;
        cmd(&mut card, 25, 0).await.unwrap();
        // The first block's busy outlasts the timeout, so the host moves on too early.
        assert_eq!(
            card.write_block(&to_words(&pattern(1)), data_crc(&pattern(1)), 1)
                .await,
            Err(BusError::Timeout)
        );
        assert!(card.is_busy());
        card.write_block(&to_words(&pattern(2)), data_crc(&pattern(2)), 100)
            .await
            .unwrap();
        assert_eq!(
            card.violations(),
            &[Violation::DataWhileBusy { write: true }]
        );
    });
}

#[test]
fn a_correct_session_has_no_violations() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        cmd(&mut card, 23, 3).await.unwrap();
        cmd(&mut card, 25, 77).await.unwrap();
        for i in 0..3 {
            send_block(&mut card, &pattern(i)).await.unwrap();
        }
        cmd(&mut card, 18, 77).await.unwrap();
        for _ in 0..3 {
            recv_block(&mut card).await.unwrap();
        }
        let r = cmd(&mut card, 12, 0).await.unwrap();
        assert_eq!(state_field(status(&r, 12)), 5);
        card.wait_ready(10).await.unwrap();
        assert_eq!(card.violations(), &[]);
    });
}
