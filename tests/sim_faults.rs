//! Each injected fault does what it says at the scripted time, and not otherwise.

mod sim_helpers;

use embassy_futures::block_on;
use pio_sdmmc::sim::{
    CardState, Fault, Match, SimCard, SimConfig, TraceEvent, Trigger, Violation, status,
};
use pio_sdmmc::{Bus, BusError};
use sim_helpers::*;

fn rca_arg() -> u32 {
    u32::from(RCA) << 16
}

async fn ready_card() -> SimCard {
    let mut card = SimCard::default();
    bring_up(&mut card).await;
    card
}

async fn read_raw(card: &mut SimCard, lba: u32) -> ([u8; 512], u64) {
    let r = cmd(card, 17, lba).await.unwrap();
    status(&r, 17);
    recv_block(card).await.unwrap()
}

#[test]
fn no_response_on_the_nth_command_only() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::NoResponse, Trigger::command(13, 2));
        // Commands with other indexes do not count.
        cmd(&mut card, 16, 512).await.unwrap();
        cmd(&mut card, 13, rca_arg()).await.unwrap();
        // Leave an error pending: a lost response must not clear it.
        assert!(cmd(&mut card, 12, 0).await.is_err());
        assert_eq!(cmd(&mut card, 13, rca_arg()).await, Err(BusError::Timeout));
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_ne!(s & ILLEGAL, 0);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_eq!(s & ILLEGAL, 0);
        let lost = card
            .trace()
            .iter()
            .filter(|e| {
                matches!(
                    &e.event,
                    TraceEvent::Command {
                        index: 13,
                        response: None,
                        result: Err(BusError::Timeout),
                        ..
                    }
                )
            })
            .count();
        assert_eq!(lost, 1);
    });
}

#[test]
fn no_response_can_repeat() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::NoResponse, Trigger::command(13, 1).repeat(2));
        assert!(cmd(&mut card, 13, rca_arg()).await.is_err());
        assert!(cmd(&mut card, 13, rca_arg()).await.is_err());
        assert!(cmd(&mut card, 13, rca_arg()).await.is_ok());
        assert!(cmd(&mut card, 13, rca_arg()).await.is_ok());
    });
}

#[test]
fn response_bit_flip() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::FlipResponseBit { bit: 20 }, Trigger::command(13, 2));
        let clean = cmd(&mut card, 13, rca_arg()).await.unwrap();
        let bad = cmd(&mut card, 13, rca_arg()).await.unwrap();
        let again = cmd(&mut card, 13, rca_arg()).await.unwrap();
        assert_eq!(clean, again);
        let diff: Vec<u8> = clean.iter().zip(bad.iter()).map(|(a, b)| a ^ b).collect();
        assert_eq!(diff, [0, 0, 0x08, 0, 0, 0]);
        assert_ne!(bad[5], (crc7(&bad[..5]) << 1) | 1);

        // Bit 0 is the start bit; bit 135 is the end bit of a 136-bit response.
        let mut card = SimCard::default();
        identify(&mut card, true).await;
        let clean = cmd_long(&mut card, 10, rca_arg()).await.unwrap();
        card.inject(Fault::FlipResponseBit { bit: 0 }, Trigger::command(10, 1));
        card.inject(Fault::FlipResponseBit { bit: 135 }, Trigger::command(10, 2));
        let a = cmd_long(&mut card, 10, rca_arg()).await.unwrap();
        let b = cmd_long(&mut card, 10, rca_arg()).await.unwrap();
        assert_eq!(a[0] ^ clean[0], 0x80);
        assert_eq!(&a[1..], &clean[1..]);
        assert_eq!(b[16] ^ clean[16], 0x01);
        assert_eq!(&b[..16], &clean[..16]);
    });
}

#[test]
fn read_data_bit_flip_leaves_the_crc_of_the_true_data() {
    block_on(async {
        let mut card = ready_card().await;
        card.poke_block(7, &pattern(1));
        card.inject(Fault::FlipDataBit { bit: 9 }, Trigger::block_read(2));
        let (a, crc_a) = read_raw(&mut card, 7).await;
        let (b, crc_b) = read_raw(&mut card, 7).await;
        let (c, _) = read_raw(&mut card, 7).await;
        assert_eq!(a, pattern(1));
        assert_eq!(c, pattern(1));
        assert_eq!(crc_a, data_crc(&pattern(1)));
        assert_eq!(crc_b, data_crc(&pattern(1)));
        assert_ne!(data_crc(&b), crc_b);
        let diff: Vec<(usize, u8)> = (0..512)
            .filter(|&i| a[i] != b[i])
            .map(|i| (i, a[i] ^ b[i]))
            .collect();
        assert_eq!(diff, [(1, 0x02)]);
    });
}

#[test]
fn read_crc_bit_flip() {
    block_on(async {
        let mut card = ready_card().await;
        card.poke_block(7, &pattern(1));
        card.inject(
            Fault::FlipCrcBit { bit: 5 },
            Trigger::new(Match::ReadLba(7), 1),
        );
        let (a, crc_a) = read_raw(&mut card, 7).await;
        let (b, crc_b) = read_raw(&mut card, 7).await;
        assert_eq!(a, pattern(1));
        assert_eq!(crc_a ^ data_crc(&a), 1 << 5);
        assert_eq!(b, pattern(1));
        assert_eq!(crc_b, data_crc(&b));
    });
}

#[test]
fn lba_trigger_counts_only_that_block_and_can_repeat() {
    block_on(async {
        let mut card = ready_card().await;
        card.poke_block(5, &pattern(1));
        card.poke_block(6, &pattern(2));
        card.inject(Fault::FlipDataBit { bit: 0 }, Trigger::lba(5).repeat(2));
        let (x, _) = read_raw(&mut card, 6).await;
        assert_eq!(x, pattern(2));
        let (a, _) = read_raw(&mut card, 5).await;
        let (b, _) = read_raw(&mut card, 5).await;
        let (c, _) = read_raw(&mut card, 5).await;
        assert_ne!(a, pattern(1));
        assert_ne!(b, pattern(1));
        assert_eq!(c, pattern(1));
    });
}

#[test]
fn data_out_crc_error_token_even_when_the_host_crc_is_right() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::DataCrcErrorToken, Trigger::block_write(2));
        assert_eq!(write_single(&mut card, 20, &pattern(1)).await, 0b010);
        assert_eq!(write_single(&mut card, 21, &pattern(2)).await, 0b101);
        assert_eq!(card.peek_block(21), [0u8; 512]);
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(write_single(&mut card, 21, &pattern(2)).await, 0b010);
        assert_eq!(card.peek_block(20), pattern(1));
        assert_eq!(card.peek_block(21), pattern(2));
        assert_eq!(card.last_write_ok_blocks(), 1);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn host_crc_mismatch_gets_the_crc_error_token_and_nothing_is_stored() {
    block_on(async {
        let mut card = ready_card().await;
        cmd(&mut card, 24, 30).await.unwrap();
        let tok = card
            .write_block(&to_words(&pattern(1)), data_crc(&pattern(1)) ^ 1, 100)
            .await
            .unwrap();
        assert_eq!(tok, 0b101);
        assert_eq!(card.peek_block(30), [0u8; 512]);
        assert_eq!(card.state(), CardState::Tran);
    });
}

#[test]
fn multi_block_crc_error_leaves_the_card_receiving() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::DataCrcErrorToken, Trigger::block_write(2));
        cmd(&mut card, 25, 40).await.unwrap();
        assert_eq!(send_block(&mut card, &pattern(1)).await.unwrap(), 0b010);
        assert_eq!(send_block(&mut card, &pattern(2)).await.unwrap(), 0b101);
        assert_eq!(card.state(), CardState::Rcv);
        cmd(&mut card, 12, 0).await.unwrap();
        card.wait_ready(100).await.unwrap();
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.last_write_ok_blocks(), 1);
        assert_eq!(card.peek_block(41), [0u8; 512]);
    });
}

#[test]
fn write_error_token_stores_nothing_and_raises_error() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::WriteErrorToken, Trigger::new(Match::WriteLba(9), 1));
        assert_eq!(write_single(&mut card, 8, &pattern(1)).await, 0b010);
        assert_eq!(write_single(&mut card, 9, &pattern(2)).await, 0b110);
        assert_eq!(card.peek_block(9), [0u8; 512]);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_ne!(s & ERROR_BIT, 0);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_eq!(s & ERROR_BIT, 0);
        assert_eq!(write_single(&mut card, 9, &pattern(2)).await, 0b010);
        assert_eq!(card.peek_block(9), pattern(2));
    });
}

#[test]
fn busy_after_write_times_out_the_write_but_stores_the_block() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(
            Fault::BusyAfterWrite { us: 50_000 },
            Trigger::block_write(1),
        );
        cmd(&mut card, 24, 3).await.unwrap();
        let w = to_words(&pattern(4));
        assert_eq!(
            card.write_block(&w, data_crc(&pattern(4)), 10).await,
            Err(BusError::Timeout)
        );
        assert_eq!(card.peek_block(3), pattern(4));
        assert_eq!(card.busy_remaining_us(), 40_000);
        assert_eq!(card.state(), CardState::Prg);
        let t0 = card.now_us();
        assert_eq!(card.wait_ready(10).await, Err(BusError::Timeout));
        assert_eq!(card.now_us(), t0 + 10_000);
        card.wait_ready(100).await.unwrap();
        assert_eq!(card.busy_remaining_us(), 0);
        assert_eq!(card.state(), CardState::Tran);
        // The next write has the configured busy time again.
        let t0 = card.now_us();
        assert_eq!(write_single(&mut card, 4, &pattern(5)).await, 0b010);
        assert!(card.now_us() - t0 < 5_000);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn busy_after_write_within_the_timeout_just_waits() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(
            Fault::BusyAfterWrite { us: 50_000 },
            Trigger::block_write(1),
        );
        cmd(&mut card, 24, 3).await.unwrap();
        let t0 = card.now_us();
        let tok = card
            .write_block(&to_words(&pattern(4)), data_crc(&pattern(4)), 100)
            .await
            .unwrap();
        assert_eq!(tok, 0b010);
        assert!(card.now_us() - t0 >= 50_000);
        assert!(!card.is_busy());
    });
}

#[test]
fn busy_after_cmd12() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(
            Fault::BusyAfterCommand { us: 30_000 },
            Trigger::command(12, 2),
        );
        for lba in [60, 70] {
            cmd(&mut card, 25, lba).await.unwrap();
            send_block(&mut card, &pattern(1)).await.unwrap();
            cmd(&mut card, 12, 0).await.unwrap();
            let busy = card.busy_remaining_us();
            if lba == 60 {
                assert!(busy < 1_000, "{busy}");
            } else {
                assert!(busy >= 30_000, "{busy}");
                assert_eq!(card.wait_ready(10).await, Err(BusError::Timeout));
            }
            card.wait_ready(100).await.unwrap();
        }
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn status_error_bit_in_a_command_response_and_after_a_block() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(
            Fault::StatusError {
                mask: status::ERASE_PARAM,
            },
            Trigger::command(13, 2),
        );
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_eq!(s & status::ERASE_PARAM, 0);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_ne!(s & status::ERASE_PARAM, 0);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_eq!(s & status::ERASE_PARAM, 0);

        card.inject(
            Fault::StatusError {
                mask: status::CARD_ECC_FAILED,
            },
            Trigger::block_read(1),
        );
        read_raw(&mut card, 1).await;
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_ne!(s & status::CARD_ECC_FAILED, 0);
        let s = status(&cmd(&mut card, 13, rca_arg()).await.unwrap(), 13);
        assert_eq!(s & status::CARD_ECC_FAILED, 0);
    });
}

#[test]
fn card_removal_times_out_everything_until_reinsert() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::RemoveCard, Trigger::command(17, 2));
        assert_eq!(write_single(&mut card, 11, &pattern(7)).await, 0b010);
        read_raw(&mut card, 11).await;
        assert!(!card.is_removed());
        assert_eq!(cmd(&mut card, 17, 11).await, Err(BusError::Timeout));
        assert!(card.is_removed());
        let mut w = [0u32; 128];
        let t0 = card.now_us();
        assert_eq!(card.read_block(&mut w, 5).await, Err(BusError::Timeout));
        assert_eq!(card.now_us(), t0 + 5_000);
        assert_eq!(card.write_block(&w, 0, 5).await, Err(BusError::Timeout));
        assert_eq!(card.wait_ready(5).await, Err(BusError::Timeout));
        assert_eq!(cmd(&mut card, 13, rca_arg()).await, Err(BusError::Timeout));
        assert_eq!(cmd_none(&mut card, 0, 0).await, Ok(()));

        card.reinsert();
        assert!(!card.is_removed());
        assert_eq!(card.state(), CardState::Idle);
        assert!(!card.is_four_bit());
        assert_eq!(card.peek_block(11), pattern(7));
        bring_up(&mut card).await;
        assert_eq!(read_single(&mut card, 11).await, pattern(7));
        // The trigger fired once and is spent.
        assert!(!card.is_removed());
    });
}

#[test]
fn card_removal_on_a_block_write() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::RemoveCard, Trigger::block_write(2));
        assert_eq!(write_single(&mut card, 1, &pattern(1)).await, 0b010);
        cmd(&mut card, 24, 2).await.unwrap();
        assert_eq!(
            send_block(&mut card, &pattern(2)).await,
            Err(BusError::Timeout)
        );
        assert_eq!(card.peek_block(2), [0u8; 512]);
        assert!(card.is_removed());
    });
}

#[test]
fn acmd41_never_ready() {
    block_on(async {
        let mut card = SimCard::default();
        card.inject(Fault::Acmd41NeverReady, Trigger::command(41, 1).forever());
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        cmd(&mut card, 8, 0x1AA).await.unwrap();
        for _ in 0..200 {
            cmd(&mut card, 55, 0).await.unwrap();
            let r = cmd(&mut card, 41, 0x40FF_8000).await.unwrap();
            assert_eq!(r[1] & 0x80, 0);
            card.delay_ms(10).await;
        }
        assert_eq!(card.state(), CardState::Idle);
    });
}

#[test]
fn acmd41_never_ready_for_a_set_number_of_polls() {
    block_on(async {
        let cfg = SimConfig {
            acmd41_polls: 0,
            acmd41_ms: 0,
            ..SimConfig::default()
        };
        let mut card = SimCard::new(cfg);
        card.inject(Fault::Acmd41NeverReady, Trigger::command(41, 2).repeat(2));
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        cmd(&mut card, 8, 0x1AA).await.unwrap();
        let mut ready = Vec::new();
        for _ in 0..5 {
            cmd(&mut card, 55, 0).await.unwrap();
            let r = cmd(&mut card, 41, 0x40FF_8000).await.unwrap();
            ready.push(r[1] & 0x80 != 0);
            if ready.last() == Some(&true) {
                break;
            }
        }
        assert_eq!(ready, [true]);

        // Fresh card: poll 1 ready would have ended the dance, so start over without ACMD41 yet.
        let cfg = SimConfig {
            acmd41_polls: 0,
            acmd41_ms: 0,
            ..SimConfig::default()
        };
        let mut card = SimCard::new(cfg);
        card.inject(Fault::Acmd41NeverReady, Trigger::command(41, 1).repeat(2));
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        cmd(&mut card, 8, 0x1AA).await.unwrap();
        let mut ready = Vec::new();
        for _ in 0..3 {
            cmd(&mut card, 55, 0).await.unwrap();
            let r = cmd(&mut card, 41, 0x40FF_8000).await.unwrap();
            ready.push(r[1] & 0x80 != 0);
        }
        assert_eq!(ready, [false, false, true]);
    });
}

#[test]
#[should_panic(expected = "cannot be triggered")]
fn a_fault_with_a_trigger_that_cannot_fire_it_is_refused() {
    let mut card = SimCard::default();
    card.inject(Fault::NoResponse, Trigger::block_read(1));
}

#[test]
fn faulted_scripts_are_deterministic() {
    fn run() -> SimCard {
        let mut card = SimCard::default();
        card.inject(Fault::FlipDataBit { bit: 100 }, Trigger::block_read(2));
        card.inject(Fault::NoResponse, Trigger::command(24, 3));
        card.inject(Fault::BusyAfterWrite { us: 9_999 }, Trigger::block_write(2));
        card.inject(Fault::DataCrcErrorToken, Trigger::block_write(3));
        block_on(async {
            bring_up(&mut card).await;
            for i in 0..4u32 {
                let _ = cmd(&mut card, 24, i).await;
                let _ = send_block(&mut card, &pattern(i as u8)).await;
                let _ = cmd(&mut card, 17, i).await;
                let _ = recv_block(&mut card).await;
                let _ = card.wait_ready(20).await;
            }
        });
        card
    }
    let a = run();
    let b = run();
    assert_eq!(a.trace(), b.trace());
    assert_eq!(a.violations(), b.violations());
    assert_eq!(a.now_us(), b.now_us());
    assert!(a.trace().len() > 30);
}

async fn wr_count(card: &mut SimCard) -> u32 {
    acmd(card, RCA, 22, 0).await.unwrap();
    let mut w = [0u32; 1];
    card.read_block(&mut w, 100).await.unwrap();
    u32::from_be_bytes(to_bytes(&w).try_into().unwrap())
}

/// Blocks 1 and 2 are accepted, block 3 gets `bad_token`, blocks 4 and 5 are ignored.
async fn multi_block_failure(
    card: &mut SimCard,
    preset: Option<u32>,
    bad_token: u8,
    block3_crc_wrong: bool,
) {
    if let Some(n) = preset {
        cmd(card, 23, n).await.unwrap();
    }
    cmd(card, 25, 100).await.unwrap();
    assert_eq!(send_block(card, &pattern(1)).await.unwrap(), 0b010);
    assert_eq!(send_block(card, &pattern(2)).await.unwrap(), 0b010);
    let crc = data_crc(&pattern(3)) ^ u64::from(block3_crc_wrong);
    assert_eq!(
        card.write_block(&to_words(&pattern(3)), crc, 100)
            .await
            .unwrap(),
        bad_token
    );
    assert_eq!(card.violations(), &[]);
    let t0 = card.now_us();
    assert_eq!(send_block(card, &pattern(4)).await, Err(BusError::Timeout));
    assert!(card.now_us() > t0);
    assert_eq!(card.violations(), &[Violation::WriteAfterErrorToken]);
    assert_eq!(send_block(card, &pattern(5)).await, Err(BusError::Timeout));
    assert_eq!(card.violations().len(), 2);
    for lba in 102..105 {
        assert_eq!(card.peek_block(lba), [0u8; 512]);
    }
    assert_eq!(card.peek_block(100), pattern(1));
    assert_eq!(card.peek_block(101), pattern(2));
    assert_eq!(card.last_write_ok_blocks(), 2);
    assert_eq!(card.state(), CardState::Rcv);
    // CMD12 recovers as before.
    let r = cmd(card, 12, 0).await.unwrap();
    assert_eq!(state_field(status(&r, 12)), 6);
    card.wait_ready(100).await.unwrap();
    assert_eq!(card.state(), CardState::Tran);
    assert_eq!(wr_count(card).await, 2);
    card.take_violations();
    // And the card takes writes again.
    assert_eq!(write_single(card, 200, &pattern(9)).await, 0b010);
    assert_eq!(card.peek_block(200), pattern(9));
    assert_eq!(card.violations(), &[]);
}

#[test]
fn multi_block_write_ignores_blocks_after_an_injected_crc_error_token() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::DataCrcErrorToken, Trigger::block_write(3));
        multi_block_failure(&mut card, None, 0b101, false).await;
    });
}

#[test]
fn multi_block_write_ignores_blocks_after_an_injected_write_error_token() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::WriteErrorToken, Trigger::block_write(3));
        multi_block_failure(&mut card, None, 0b110, false).await;
    });
}

#[test]
fn multi_block_write_with_count_ignores_blocks_after_a_real_crc_mismatch() {
    block_on(async {
        let mut card = ready_card().await;
        multi_block_failure(&mut card, Some(5), 0b101, true).await;
    });
}

#[test]
fn cmd0_also_ends_the_ignoring_state() {
    block_on(async {
        let mut card = ready_card().await;
        card.inject(Fault::DataCrcErrorToken, Trigger::block_write(1));
        cmd(&mut card, 25, 100).await.unwrap();
        assert_eq!(send_block(&mut card, &pattern(1)).await.unwrap(), 0b101);
        assert_eq!(
            send_block(&mut card, &pattern(1)).await,
            Err(BusError::Timeout)
        );
        cmd_none(&mut card, 0, 0).await.unwrap();
        bring_up(&mut card).await;
        cmd(&mut card, 25, 100).await.unwrap();
        assert_eq!(send_block(&mut card, &pattern(1)).await.unwrap(), 0b010);
    });
}
