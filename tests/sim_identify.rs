//! Identification, data transfer and card kinds, driven by hand through raw `Bus` calls.

mod sim_helpers;

use embassy_futures::block_on;
use pio_sdmmc::Bus;
use pio_sdmmc::sim::{CardKind, CardState, SimCard, SimConfig, TraceEvent};
use sim_helpers::*;

fn parse_csd_blocks(r: &[u8; 17]) -> u64 {
    let v = u128::from_be_bytes(r[1..17].try_into().unwrap());
    assert_eq!(v >> 126, 1, "CSD version 2");
    let c_size = ((v >> 48) & 0x3F_FFFF) as u64;
    (c_size + 1) * 1024
}

#[test]
fn crc_helpers_match_known_vectors() {
    assert_eq!(frame(0, 0), [0x40, 0, 0, 0, 0, 0x95]);
    assert_eq!(frame(8, 0x1AA), [0x48, 0, 0, 0x01, 0xAA, 0x87]);
    assert_eq!(frame(17, 0), [0x51, 0, 0, 0, 0, 0x55]);
    assert_eq!(frame(55, 0), [0x77, 0, 0, 0, 0, 0x65]);
    assert_eq!(data_crc(&[0u8; 512]), 0);
    // 1024 one-bits per line give CRC16 0xEDA9 (4096 one-bits, as in one-bit mode, give 0x7FA1).
    let mut expect = 0u64;
    for i in 0..16 {
        if (0xEDA9u16 >> (15 - i)) & 1 != 0 {
            expect |= 0xF << (60 - 4 * i);
        }
    }
    assert_eq!(data_crc(&[0xFF; 512]), expect);
}

#[test]
fn default_card_identifies_to_tran_in_four_bit_mode_without_violations() {
    block_on(async {
        let mut card = SimCard::default();
        let ocr = identify(&mut card, true).await;
        assert_eq!(ocr, 0xC0FF_8000);
        assert_eq!(card.state(), CardState::Stby);

        let cid = cmd_long(&mut card, 10, u32::from(RCA) << 16).await.unwrap();
        assert_eq!(cid[1], 0x03);
        assert_eq!(&cid[2..4], b"SD");
        assert_eq!(&cid[4..9], b"SK64G");
        assert_eq!(cid[9], 0x80);
        assert_eq!(
            u32::from_be_bytes([cid[10], cid[11], cid[12], cid[13]]),
            0xB183_C2B8
        );
        assert_eq!(cid[16], (crc7(&cid[1..16]) << 1) | 1);

        let csd = cmd_long(&mut card, 9, u32::from(RCA) << 16).await.unwrap();
        assert_eq!(csd[0], 0x3F);
        assert_eq!(csd[16], (crc7(&csd[1..16]) << 1) | 1);
        assert_eq!(parse_csd_blocks(&csd), 124_735_488);

        let r = cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap();
        let s = status(&r, 13);
        assert_eq!(state_field(s), 3);
        assert_eq!(s & ILLEGAL, 0);

        select_and_widen(&mut card).await;
        assert_eq!(card.state(), CardState::Tran);
        assert!(card.is_four_bit());

        let r = cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap();
        assert_eq!(status(&r, 13), 0x0000_0900);

        // SCR: CMD23 supported, 4-bit and 1-bit widths.
        let r = acmd(&mut card, RCA, 51, 0).await.unwrap();
        assert_eq!(status(&r, 51), 0x0000_0920);
        let mut w = [0u32; 2];
        let crc = card.read_block(&mut w, 100).await.unwrap();
        let scr = to_bytes(&w);
        assert_eq!(crc, data_crc(&scr));
        assert_eq!(scr[1] & 0x0F, 0x05);
        assert_eq!(scr[3] & 0x02, 0x02);
        assert_eq!(card.state(), CardState::Tran);

        // SD status: bus width 4.
        acmd(&mut card, RCA, 13, 0).await.unwrap();
        let mut w = [0u32; 16];
        let crc = card.read_block(&mut w, 100).await.unwrap();
        let st = to_bytes(&w);
        assert_eq!(crc, data_crc(&st));
        assert_eq!(st[0] >> 6, 0b10);

        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn single_block_write_then_read_back() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        let data = pattern(3);
        assert_eq!(write_single(&mut card, 1000, &data).await, 0b010);
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.peek_block(1000), data);
        assert_eq!(read_single(&mut card, 1000).await, data);
        assert_eq!(read_single(&mut card, 1001).await, [0u8; 512]);
        card.poke_block(1002, &[0xFF; 512]);
        cmd(&mut card, 17, 1002).await.unwrap();
        let (_, crc) = recv_block(&mut card).await.unwrap();
        assert_eq!(crc, data_crc(&[0xFF; 512]));
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn multi_block_with_cmd23_ends_by_itself() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        let r = cmd(&mut card, 23, 4).await.unwrap();
        status(&r, 23);
        let r = cmd(&mut card, 25, 200).await.unwrap();
        assert_eq!(state_field(status(&r, 25)), 4);
        for i in 0..4u8 {
            assert_eq!(send_block(&mut card, &pattern(i)).await.unwrap(), 0b010);
        }
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.last_write_ok_blocks(), 4);
        acmd(&mut card, RCA, 22, 0).await.unwrap();
        let mut w = [0u32; 1];
        card.read_block(&mut w, 100).await.unwrap();
        assert_eq!(to_bytes(&w), 4u32.to_be_bytes());

        cmd(&mut card, 23, 4).await.unwrap();
        let r = cmd(&mut card, 18, 200).await.unwrap();
        status(&r, 18);
        for i in 0..4u8 {
            let (b, crc) = recv_block(&mut card).await.unwrap();
            assert_eq!(b, pattern(i));
            assert_eq!(crc, data_crc(&b));
        }
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn multi_block_without_cmd23_runs_until_cmd12() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        cmd(&mut card, 25, 300).await.unwrap();
        for i in 0..3u8 {
            assert_eq!(
                send_block(&mut card, &pattern(10 + i)).await.unwrap(),
                0b010
            );
        }
        assert_eq!(card.state(), CardState::Rcv);
        let r = cmd(&mut card, 12, 0).await.unwrap();
        assert_eq!(state_field(status(&r, 12)), 6);
        assert!(card.is_busy());
        card.wait_ready(100).await.unwrap();
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.last_write_ok_blocks(), 3);

        let r = cmd(&mut card, 18, 300).await.unwrap();
        status(&r, 18);
        for i in 0..3u8 {
            let (b, _) = recv_block(&mut card).await.unwrap();
            assert_eq!(b, pattern(10 + i));
        }
        assert_eq!(card.state(), CardState::Data);
        let r = cmd(&mut card, 12, 0).await.unwrap();
        assert_eq!(state_field(status(&r, 12)), 5);
        card.wait_ready(100).await.unwrap();
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn erase_zeroes_the_range() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        for lba in 10..14 {
            card.poke_block(lba, &pattern(lba as u8));
        }
        cmd(&mut card, 32, 11).await.unwrap();
        cmd(&mut card, 33, 12).await.unwrap();
        let r = cmd(&mut card, 38, 0).await.unwrap();
        status(&r, 38);
        card.wait_ready(100).await.unwrap();
        assert_eq!(card.peek_block(10), pattern(10));
        assert_eq!(card.peek_block(11), [0u8; 512]);
        assert_eq!(card.peek_block(12), [0u8; 512]);
        assert_eq!(card.peek_block(13), pattern(13));
        assert_eq!(read_single(&mut card, 11).await, [0u8; 512]);
        // CMD38 without a range is a sequence error.
        let s = status(&cmd(&mut card, 38, 0).await.unwrap(), 38);
        assert_ne!(s & (1 << 28), 0);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn sdsc_v1_ignores_cmd8_and_takes_byte_addresses() {
    block_on(async {
        let cfg = SimConfig {
            kind: CardKind::SdscV1,
            capacity_blocks: 2_097_152,
            ..SimConfig::default()
        };
        let mut card = SimCard::new(cfg);
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(
            cmd(&mut card, 8, 0x1AA).await,
            Err(pio_sdmmc::BusError::Timeout)
        );
        // The unanswered CMD8 left ILLEGAL_COMMAND for the next R1, once.
        let s = status(&cmd(&mut card, 55, 0).await.unwrap(), 55);
        assert_ne!(s & ILLEGAL, 0);
        let s = status(&cmd(&mut card, 55, 0).await.unwrap(), 55);
        assert_eq!(s & ILLEGAL, 0);
        let mut ocr = 0;
        for _ in 0..100 {
            cmd(&mut card, 55, 0).await.unwrap();
            let r = cmd(&mut card, 41, 0x00FF_8000).await.unwrap();
            ocr = u32::from_be_bytes([r[1], r[2], r[3], r[4]]);
            if ocr & 0x8000_0000 != 0 {
                break;
            }
            card.delay_ms(10).await;
        }
        assert_eq!(ocr, 0x80FF_8000);
        cmd_long(&mut card, 2, 0).await.unwrap();
        cmd(&mut card, 3, 0).await.unwrap();
        let csd = cmd_long(&mut card, 9, u32::from(RCA) << 16).await.unwrap();
        let v = u128::from_be_bytes(csd[1..17].try_into().unwrap());
        assert_eq!(v >> 126, 0, "CSD version 1");
        let c_size = ((v >> 62) & 0xFFF) as u64;
        let mult = ((v >> 47) & 7) as u32;
        assert_eq!((c_size + 1) << (mult + 2), 2_097_152);
        select_and_widen(&mut card).await;

        let data = pattern(9);
        assert_eq!(write_single(&mut card, 7 * 512, &data).await, 0b010);
        assert_eq!(card.peek_block(7), data);
        assert_eq!(read_single(&mut card, 7 * 512).await, data);

        // Misaligned and out-of-range byte addresses.
        let s = status(&cmd(&mut card, 17, 7 * 512 + 4).await.unwrap(), 17);
        assert_ne!(s & ADDRESS_ERROR, 0);
        assert_eq!(card.state(), CardState::Tran);
        let s = status(&cmd(&mut card, 17, 2_097_152 * 512).await.unwrap(), 17);
        assert_ne!(s & OUT_OF_RANGE, 0);
        // Block length: only 512 is accepted.
        let s = status(&cmd(&mut card, 16, 512).await.unwrap(), 16);
        assert_eq!(s & (1 << 29), 0);
        let s = status(&cmd(&mut card, 16, 256).await.unwrap(), 16);
        assert_ne!(s & (1 << 29), 0);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn sdhc_needs_hcs_and_takes_block_addresses() {
    block_on(async {
        let cfg = SimConfig {
            kind: CardKind::Sdhc,
            capacity_blocks: 30_000_000,
            ..SimConfig::default()
        };
        let mut card = SimCard::new(cfg.clone());
        card.set_clock(400_000);
        card.clock_idle(80).await;
        cmd_none(&mut card, 0, 0).await.unwrap();
        cmd(&mut card, 8, 0x1AA).await.unwrap();
        // Without HCS the card stays busy however long it is polled.
        for _ in 0..50 {
            cmd(&mut card, 55, 0).await.unwrap();
            let r = cmd(&mut card, 41, 0x00FF_8000).await.unwrap();
            assert_eq!(r[1] & 0x80, 0);
            card.delay_ms(10).await;
        }
        assert_eq!(card.state(), CardState::Idle);

        let mut card = SimCard::new(cfg);
        identify(&mut card, true).await;
        select_and_widen(&mut card).await;
        let data = pattern(5);
        assert_eq!(write_single(&mut card, 29_999_999, &data).await, 0b010);
        assert_eq!(read_single(&mut card, 29_999_999).await, data);
        let s = status(&cmd(&mut card, 17, 30_000_000).await.unwrap(), 17);
        assert_ne!(s & OUT_OF_RANGE, 0);
        assert_eq!(card.state(), CardState::Tran);
        assert_eq!(card.violations(), &[]);
    });
}

#[test]
fn error_bits_are_reported_once() {
    block_on(async {
        let mut card = SimCard::default();
        bring_up(&mut card).await;
        // Illegal in tran: CMD12. No response, ILLEGAL_COMMAND in the next R1 only.
        assert!(cmd(&mut card, 12, 0).await.is_err());
        let s = status(&cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap(), 13);
        assert_ne!(s & ILLEGAL, 0);
        let s = status(&cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap(), 13);
        assert_eq!(s & ILLEGAL, 0);
        // A frame with a wrong CRC7, and one with a wrong end bit.
        let mut bad = frame(13, u32::from(RCA) << 16);
        bad[5] ^= 0x10;
        let mut r = [0u8; 6];
        assert!(
            card.command(&bad, pio_sdmmc::ResponseBuf::Short(&mut r))
                .await
                .is_err()
        );
        let s = status(&cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap(), 13);
        assert_ne!(s & COM_CRC, 0);
        let mut bad = frame(13, u32::from(RCA) << 16);
        bad[5] &= 0xFE;
        assert!(
            card.command(&bad, pio_sdmmc::ResponseBuf::Short(&mut r))
                .await
                .is_err()
        );
        let s = status(&cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap(), 13);
        assert_ne!(s & COM_CRC, 0);
        let s = status(&cmd(&mut card, 13, u32::from(RCA) << 16).await.unwrap(), 13);
        assert_eq!(s & COM_CRC, 0);
        // CMD0 clears pending errors.
        assert!(cmd(&mut card, 12, 0).await.is_err());
        cmd_none(&mut card, 0, 0).await.unwrap();
        let s = status(&cmd(&mut card, 55, 0).await.unwrap(), 55);
        assert_eq!(s & ILLEGAL, 0);
        assert_eq!(s, 0x0000_0120);
    });
}

#[test]
fn virtual_time_follows_clock_and_cycles() {
    block_on(async {
        let mut card = SimCard::default();
        assert_eq!(card.set_clock(400_000), 400_000);
        assert_eq!(card.now_us(), 0);
        card.clock_idle(400).await;
        assert_eq!(card.now_us(), 1000);
        card.delay_ms(3).await;
        assert_eq!(card.now_us(), 4000);
        cmd_none(&mut card, 0, 0).await.unwrap();
        assert_eq!(card.now_us(), 4000 + (48 + 8) * 1_000_000 / 400_000);
        assert_eq!(card.set_clock(100_000_000), 75_000_000);
        assert_eq!(card.set_clock(30_000_000), 30_000_000);
        assert_eq!(card.set_clock(26_000_000), 25_000_000);
        assert_eq!(card.set_clock(1), 1);
    });
}

#[test]
fn wait_ready_times_out_but_time_still_passes() {
    block_on(async {
        let mut card = SimCard::new(SimConfig {
            cmd12_busy_us: 5000,
            ..SimConfig::default()
        });
        bring_up(&mut card).await;
        cmd(&mut card, 25, 5).await.unwrap();
        send_block(&mut card, &pattern(1)).await.unwrap();
        cmd(&mut card, 12, 0).await.unwrap();
        let busy = card.busy_remaining_us();
        assert!(busy > 100);
        let t0 = card.now_us();
        assert_eq!(card.wait_ready(0).await, Err(pio_sdmmc::BusError::Timeout));
        assert_eq!(card.now_us(), t0);
        let t0 = card.now_us();
        assert!(card.wait_ready(1).await.is_err());
        assert_eq!(card.now_us(), t0 + 1000);
        assert_eq!(card.busy_remaining_us(), busy - 1000.min(busy));
        card.wait_ready(100).await.unwrap();
        assert!(!card.is_busy());
    });
}

async fn script(card: &mut SimCard) {
    bring_up(card).await;
    write_single(card, 42, &pattern(8)).await;
    cmd(card, 23, 2).await.unwrap();
    cmd(card, 25, 50).await.unwrap();
    send_block(card, &pattern(1)).await.unwrap();
    send_block(card, &pattern(2)).await.unwrap();
    read_single(card, 50).await;
    card.wait_ready(5).await.unwrap();
}

#[test]
fn same_script_gives_identical_trace() {
    let mut a = SimCard::default();
    let mut b = SimCard::default();
    block_on(script(&mut a));
    block_on(script(&mut b));
    assert!(a.trace().len() > 25);
    assert_eq!(a.trace(), b.trace());
    assert_eq!(a.violations(), b.violations());
    assert_eq!(a.now_us(), b.now_us());
    assert!(
        a.trace()
            .iter()
            .any(|e| matches!(e.event, TraceEvent::WriteBlock { lba: Some(50), .. }))
    );
    assert!(a.trace().iter().any(|e| matches!(
        e.event,
        TraceEvent::Command {
            index: 25,
            arg: 50,
            ..
        }
    )));
}
