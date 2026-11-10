//! The words exchanged with the state machine, as pure functions. No hardware, no time.
//!
//! Every value here is a 32-bit FIFO word as the state machine sees it: OSR and ISR both shift
//! left, so the first nibble on the wire is bits 31..28 of a word. See the module documentation
//! of [`crate::pio`] for the program that consumes and produces them.

/// SD clock period in state-machine cycles: four low, two high.
pub const SM_CYCLES_PER_CLOCK: u32 = 6;

/// Idle nibbles (CMD high) sent ahead of every command frame.
pub const CMD_IDLE_NIBBLES: usize = 9;

/// Nibbles of a command stream: the idle nibbles, then the first 47 bits of the frame, one bit
/// per nibble. The program sends the frame's end bit itself.
pub const CMD_SEND_NIBBLES: u16 = 56;

/// Words of a command stream.
pub const CMD_STREAM_WORDS: usize = 7;

/// Nibbles received after the start bit of a 48-bit response.
pub const SHORT_RECV_NIBBLES: u16 = 47;

/// Nibbles received after the start bit of a 136-bit response.
pub const LONG_RECV_NIBBLES: u16 = 135;

/// First word of a write stream: seven idle nibbles, then the start nibble.
pub const WRITE_START_WORD: u32 = 0xFFFF_FFF0;

/// Nibbles received after the start bit of the CRC status: three token bits and the end bit.
pub const TOKEN_RECV_NIBBLES: u16 = 4;

/// A word of eight idle nibbles (all lines high).
pub const IDLE_WORD: u32 = 0xFFFF_FFFF;

/// The header word that starts every operation: nibbles to send in bits 31..16, nibbles to
/// receive after the start bit in bits 15..0.
pub const fn header(send: u16, recv: u16) -> u32 {
    ((send as u32) << 16) | recv as u32
}

/// Words the state machine pushes when it receives `recv` nibbles: one per eight nibbles, then
/// the final word with the remaining nibbles right-aligned (zero when none remain).
pub const fn recv_words(recv: u16) -> usize {
    recv as usize / 8 + 1
}

/// The command stream for `frame`: nine nibbles `0xF`, then frame bits 0..=46 as `0xF` (one) or
/// `0x0` (zero). Only bit 0 of each nibble reaches CMD; the other bits keep the stream readable.
pub fn command_stream(frame: &[u8; 6]) -> [u32; CMD_STREAM_WORDS] {
    let mut words = [0u32; CMD_STREAM_WORDS];
    for n in 0..CMD_SEND_NIBBLES as usize {
        let one = match n.checked_sub(CMD_IDLE_NIBBLES) {
            None => true,
            Some(bit) => frame[bit / 8] & (0x80 >> (bit % 8)) != 0,
        };
        if one {
            words[n / 8] |= 0xF << (28 - 4 * (n % 8));
        }
    }
    words
}

/// Nibble `k` of a received stream of `total` nibbles, from the words the state machine pushed:
/// full words first (first nibble in the top bits), then the final word right-aligned.
fn received_nibble(words: &[u32], total: usize, k: usize) -> u32 {
    let full = total / 8;
    if k < full * 8 {
        words[k / 8] >> (28 - 4 * (k % 8)) & 0xF
    } else {
        let rest = total % 8;
        words[full] >> (4 * (rest - 1 - (k - full * 8))) & 0xF
    }
}

/// Rebuilds a response frame from the words received for it.
///
/// `out` is 6 bytes for a short response (47 nibbles, 6 words) or 17 for a long one (135 nibbles,
/// 17 words). Nibble `k` carries response bit `k + 1` in its bit 0; bit 0 of the frame is the
/// start bit, consumed by the program, and is written as 0.
///
/// # Panics
///
/// If `words` does not hold `recv_words` words for `8 * out.len() - 1` nibbles.
pub fn unpack_response(words: &[u32], out: &mut [u8]) {
    let total = 8 * out.len() - 1;
    assert_eq!(words.len(), recv_words(total as u16), "response word count");
    out.fill(0);
    for k in 0..total {
        if received_nibble(words, total, k) & 1 != 0 {
            let bit = k + 1;
            out[bit / 8] |= 0x80 >> (bit % 8);
        }
    }
}

/// Header of a read of `words` data words: the data nibbles and the 16 CRC nibbles.
pub const fn read_header(words: usize) -> u32 {
    header(0, (8 * words + 16) as u16)
}

/// Header of a write of `words` data words: start word, data and CRC out; CRC status in.
pub const fn write_header(words: usize) -> u32 {
    header((8 + 8 * words + 16) as u16, TOKEN_RECV_NIBBLES)
}

/// The data CRC in the [`Bus`](crate::bus::Bus) wire form from its two FIFO words.
pub const fn crc_from_words(hi: u32, lo: u32) -> u64 {
    ((hi as u64) << 32) | lo as u64
}

/// The two FIFO words of a data CRC in the [`Bus`](crate::bus::Bus) wire form.
pub const fn crc_words(crc: u64) -> [u32; 2] {
    [(crc >> 32) as u32, crc as u32]
}

/// The 3-bit CRC status token from the final word of a write: the DAT0 bits of the first three
/// of its four nibbles (bits 12, 8 and 4), the first received most significant.
pub const fn token(final_word: u32) -> u8 {
    (((final_word >> 12) & 1) << 2 | ((final_word >> 8) & 1) << 1 | ((final_word >> 4) & 1)) as u8
}

/// The FIFO word for a data word held in memory: memory byte 0 goes out first. This is what DMA
/// with byte swap produces on a little-endian core.
pub const fn mem_to_fifo(word: u32) -> u32 {
    u32::from_be_bytes(word.to_ne_bytes())
}

/// The data word to store in memory for a received FIFO word: the first received byte at the
/// lowest address. This is what DMA with byte swap produces on a little-endian core.
pub const fn fifo_to_mem(word: u32) -> u32 {
    u32::from_ne_bytes(word.to_be_bytes())
}

/// Fills `stage` with the words of a command (header and stream) for a DMA transfer without
/// byte swap, and returns how many words it used.
pub fn stage_command(stage: &mut [u32], frame: &[u8; 6], recv: u16) -> usize {
    stage[0] = header(CMD_SEND_NIBBLES, recv);
    stage[1..=CMD_STREAM_WORDS].copy_from_slice(&command_stream(frame));
    1 + CMD_STREAM_WORDS
}

/// Fills `stage` with an idle-clock operation of `words` words of idle nibbles (CMD high, nothing
/// received) for a DMA transfer without byte swap, and returns how many words it used.
pub fn stage_idle(stage: &mut [u32], words: usize) -> usize {
    stage[0] = header((8 * words) as u16, 0);
    stage[1..=words].fill(IDLE_WORD);
    1 + words
}

/// Fills `stage` with a write operation for a DMA transfer with byte swap (each word's bytes
/// reversed on the way to the FIFO), and returns how many words it used. Caller data is copied
/// as it is, so it goes out in memory byte order; every other word is stored pre-swapped.
pub fn stage_write(stage: &mut [u32], data: &[u32], crc: u64) -> usize {
    let w = data.len();
    let [hi, lo] = crc_words(crc);
    stage[0] = write_header(w).swap_bytes();
    stage[1] = WRITE_START_WORD.swap_bytes();
    stage[2..2 + w].copy_from_slice(data);
    stage[2 + w] = hi.swap_bytes();
    stage[3 + w] = lo.swap_bytes();
    4 + w
}

/// The integer clock divider for an SD clock of at most `max_hz`, and the rate it gives:
/// `clk_sys / (6 * div)`, with the smallest `div` in `1..=65535` that does not exceed `max_hz`.
/// A `max_hz` below what divider 65535 makes returns divider 65535.
pub const fn clock_divider(clk_sys: u32, max_hz: u32) -> (u16, u32) {
    let div = if max_hz == 0 {
        u16::MAX as u64
    } else {
        let d = (clk_sys as u64).div_ceil(SM_CYCLES_PER_CLOCK as u64 * max_hz as u64);
        if d < 1 {
            1
        } else if d > u16::MAX as u64 {
            u16::MAX as u64
        } else {
            d
        }
    };
    (
        div as u16,
        (clk_sys as u64 / (SM_CYCLES_PER_CLOCK as u64 * div)) as u32,
    )
}

/// Microseconds, rounded up, that 1024 SD clock periods take with divider `div`. Computed once
/// per clock change, so that a bus time is a multiply and a shift.
pub const fn micros_per_1024_clocks(div: u16, clk_sys: u32) -> u64 {
    let sm_cycles = 1024 * SM_CYCLES_PER_CLOCK as u64 * div as u64;
    (sm_cycles * 1_000_000).div_ceil(clk_sys as u64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{Command, crc7, crc16_4bit, decode_r1, decode_r2};

    struct Xorshift(u64);

    impl Xorshift {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
    }

    /// The ISR as the program drives it: shift left by one nibble per `in pins, 4`, autopush at
    /// 32 bits, and one explicit `push` at the end.
    #[derive(Default)]
    struct Isr {
        isr: u32,
        bits: u32,
        pushed: Vec<u32>,
    }

    impl Isr {
        fn shift_in(&mut self, nibble: u32) {
            self.isr = (self.isr << 4) | (nibble & 0xF);
            self.bits += 4;
            if self.bits == 32 {
                self.pushed.push(self.isr);
                self.isr = 0;
                self.bits = 0;
            }
        }

        fn finish(mut self) -> Vec<u32> {
            self.pushed.push(self.isr);
            self.pushed
        }
    }

    /// The OSR as the program drives it: autopull at 32 bits, one nibble per `out pins, 4`.
    fn osr_nibbles(fifo: &[u32]) -> impl Iterator<Item = u32> + '_ {
        fifo.iter()
            .flat_map(|w| (0..8).map(move |i| w >> (28 - 4 * i) & 0xF))
    }

    /// The words the state machine pushes for a response frame: the card sends the frame, the
    /// program eats the start bit and reads `8 * len - 1` nibbles of one bit each (IN count 1).
    fn receive_frame(frame: &[u8]) -> Vec<u32> {
        let mut isr = Isr::default();
        for bit in 1..8 * frame.len() {
            isr.shift_in(u32::from(frame[bit / 8] >> (7 - bit % 8) & 1));
        }
        isr.finish()
    }

    #[test]
    fn header_layout() {
        assert_eq!(header(56, 47), 0x0038_002F);
        assert_eq!(read_header(128), header(0, 1040));
        assert_eq!(read_header(2), header(0, 32));
        assert_eq!(write_header(128), header(1048, 4));
        assert_eq!(recv_words(SHORT_RECV_NIBBLES), 6);
        assert_eq!(recv_words(LONG_RECV_NIBBLES), 17);
        assert_eq!(recv_words(1040), 131);
        assert_eq!(recv_words(0), 1);
    }

    #[test]
    fn cmd0_stream_words() {
        // CMD0 = 40 00 00 00 00 95: nine idle nibbles, then 0100_0000, 32 zeros, 1001_010 (the
        // final 1 is the end bit, sent by the program).
        let words = command_stream(&Command::go_idle_state().frame());
        assert_eq!(words, [0xFFFF_FFFF, 0xF0F0_0000, 0, 0, 0, 0, 0x0F00_F0F0]);
    }

    #[test]
    fn command_stream_carries_frame_bits() {
        let mut rng = Xorshift(0x0123_4567_89AB_CDEF);
        for _ in 0..200 {
            let cmd = Command {
                index: (rng.next() % 64) as u8,
                arg: rng.next() as u32,
                response: crate::proto::ResponseKind::R1,
            };
            let frame = cmd.frame();
            let words = command_stream(&frame);
            let nibbles: Vec<u32> = osr_nibbles(&words).collect();
            assert_eq!(nibbles.len(), CMD_SEND_NIBBLES as usize);
            assert!(nibbles[..CMD_IDLE_NIBBLES].iter().all(|&n| n == 0xF));
            for (bit, &n) in nibbles[CMD_IDLE_NIBBLES..].iter().enumerate() {
                let want = frame[bit / 8] >> (7 - bit % 8) & 1;
                assert_eq!(n, if want == 1 { 0xF } else { 0 }, "bit {bit}");
            }
            // The 48th bit, sent by `mov pins, !null`, is the end bit.
            assert_eq!(frame[5] & 1, 1);
        }
    }

    #[test]
    fn short_response_round_trip() {
        let mut rng = Xorshift(0xA5A5_5A5A_1234_5678);
        for _ in 0..200 {
            let mut frame = [0u8; 6];
            frame[0] = (rng.next() % 64) as u8;
            frame[1..5].copy_from_slice(&(rng.next() as u32).to_be_bytes());
            frame[5] = (crc7(&frame[..5]) << 1) | 1;
            let words = receive_frame(&frame);
            assert_eq!(words.len(), 6);
            let mut out = [0xEEu8; 6];
            unpack_response(&words, &mut out);
            assert_eq!(out, frame);
            assert!(decode_r1(frame[0], &out).is_ok());
        }
    }

    #[test]
    fn long_response_round_trip() {
        let mut rng = Xorshift(0x1357_9BDF_2468_ACE0);
        for _ in 0..100 {
            let mut frame = [0u8; 17];
            frame[0] = 0x3F;
            for b in &mut frame[1..16] {
                *b = rng.next() as u8;
            }
            frame[16] = (crc7(&frame[1..16]) << 1) | 1;
            let words = receive_frame(&frame);
            assert_eq!(words.len(), 17);
            let mut out = [0u8; 17];
            unpack_response(&words, &mut out);
            assert_eq!(out, frame);
            assert!(decode_r2(&out).is_ok());
        }
    }

    #[test]
    fn start_bit_written_as_zero() {
        // Whatever the words say, bit 0 of the frame is the start bit, which the program consumed.
        let mut out = [0u8; 6];
        unpack_response(&[u32::MAX; 6], &mut out);
        assert_eq!(out[0], 0x7F);
        assert_eq!(out[1..], [0xFF; 5]);
    }

    #[test]
    fn token_extraction() {
        // DAT0 carries the token and end bit; DAT1-3 float high.
        for (bits, want) in [
            ([0u32, 1, 0, 1], 0b010u8),
            ([1, 0, 1, 1], 0b101),
            ([1, 1, 0, 1], 0b110),
            ([1, 1, 1, 1], 0b111),
        ] {
            let mut isr = Isr::default();
            for b in bits {
                isr.shift_in(0xE | b);
            }
            let words = isr.finish();
            assert_eq!(words.len(), recv_words(TOKEN_RECV_NIBBLES));
            assert_eq!(token(words[0]), want, "{bits:?}");
        }
    }

    #[test]
    fn read_block_words() {
        let mut rng = Xorshift(0xDEAD_BEEF_0BAD_F00D);
        for w in [1usize, 2, 16, 128] {
            let bytes: Vec<u8> = (0..4 * w).map(|_| rng.next() as u8).collect();
            let crc = crc16_4bit(&bytes);
            // The card sends the data high nibble first, then sixteen CRC nibbles.
            let mut isr = Isr::default();
            for b in &bytes {
                isr.shift_in(u32::from(b >> 4));
                isr.shift_in(u32::from(b & 0xF));
            }
            for i in 0..16 {
                isr.shift_in((crc >> (60 - 4 * i)) as u32 & 0xF);
            }
            let words = isr.finish();
            assert_eq!(words.len(), recv_words((read_header(w) & 0xFFFF) as u16));
            assert_eq!(words.len(), w + 3);
            let data: Vec<u32> = words[..w].iter().map(|&f| fifo_to_mem(f)).collect();
            assert_eq!(crate::bus::as_bytes(&data), &bytes[..]);
            assert_eq!(crc_from_words(words[w], words[w + 1]), crc);
            assert_eq!(words[w + 2], 0, "final word");
        }
    }

    #[test]
    fn write_staging_produces_wire_order() {
        let mut rng = Xorshift(0x0F0F_F0F0_3C3C_C3C3);
        for w in [1usize, 2, 16, 128] {
            let data: Vec<u32> = (0..w).map(|_| rng.next() as u32).collect();
            let bytes = crate::bus::as_bytes(&data).to_vec();
            let crc = crc16_4bit(&bytes);
            let mut stage = [0u32; 132];
            let n = stage_write(&mut stage, &data, crc);
            assert_eq!(n, w + 4);
            // DMA BSWAP reverses the bytes of every word on the way to the FIFO.
            let fifo: Vec<u32> = stage[..n].iter().map(|v| v.swap_bytes()).collect();
            assert_eq!(fifo[0], write_header(w));
            for (k, d) in data.iter().enumerate() {
                assert_eq!(fifo[2 + k], mem_to_fifo(*d));
            }
            let send = (fifo[0] >> 16) as usize;
            let nibbles: Vec<u32> = osr_nibbles(&fifo[1..]).collect();
            assert_eq!(nibbles.len(), send);
            assert!(nibbles[..7].iter().all(|&x| x == 0xF), "idle nibbles");
            assert_eq!(nibbles[7], 0, "start nibble");
            for (i, b) in bytes.iter().enumerate() {
                assert_eq!(nibbles[8 + 2 * i], u32::from(b >> 4));
                assert_eq!(nibbles[9 + 2 * i], u32::from(b & 0xF));
            }
            for i in 0..16 {
                assert_eq!(
                    nibbles[8 + 8 * w + i],
                    (crc >> (60 - 4 * i)) as u32 & 0xF,
                    "crc nibble {i}"
                );
            }
        }
    }

    #[test]
    fn command_and_idle_staging() {
        let frame = Command::send_if_cond(0xAA).frame();
        let mut stage = [0u32; 132];
        assert_eq!(stage_command(&mut stage, &frame, SHORT_RECV_NIBBLES), 8);
        assert_eq!(stage[0], header(56, 47));
        assert_eq!(stage[1..8], command_stream(&frame));
        assert_eq!(stage_idle(&mut stage, 10), 11);
        assert_eq!(stage[0], header(80, 0));
        assert!(stage[1..11].iter().all(|&w| w == IDLE_WORD));
    }

    #[test]
    fn crc_word_split() {
        let crc = 0x0123_4567_89AB_CDEFu64;
        assert_eq!(crc_words(crc), [0x0123_4567, 0x89AB_CDEF]);
        let [hi, lo] = crc_words(crc);
        assert_eq!(crc_from_words(hi, lo), crc);
    }

    #[test]
    fn memory_and_fifo_order() {
        let mem = u32::from_ne_bytes([0x12, 0x34, 0x56, 0x78]);
        assert_eq!(mem_to_fifo(mem), 0x1234_5678);
        assert_eq!(fifo_to_mem(0x1234_5678), mem);
    }

    #[test]
    fn dividers() {
        assert_eq!(clock_divider(150_000_000, 400_000), (63, 396_825));
        assert_eq!(clock_divider(150_000_000, 25_000_000), (1, 25_000_000));
        assert_eq!(clock_divider(150_000_000, 50_000_000), (1, 25_000_000));
        assert_eq!(clock_divider(150_000_000, 24_999_999), (2, 12_500_000));
        assert_eq!(clock_divider(150_000_000, 12_500_000), (2, 12_500_000));
        assert_eq!(clock_divider(125_000_000, 400_000), (53, 393_081));
        assert_eq!(clock_divider(150_000_000, 0), (65535, 381));
        assert_eq!(clock_divider(150_000_000, 1), (65535, 381));
        for max in [100_000u32, 400_000, 1_000_000, 10_000_000, 20_000_000] {
            let (div, rate) = clock_divider(150_000_000, max);
            assert!(rate <= max);
            assert!(
                150_000_000 / (6 * (u32::from(div) - 1)) > max,
                "{max}: divider {div} not the smallest"
            );
        }
    }

    #[test]
    fn clock_time() {
        // 1024 clocks at 396.8 kHz = 2580.48 us; at 25 MHz = 40.96 us.
        assert_eq!(micros_per_1024_clocks(63, 150_000_000), 2581);
        assert_eq!(micros_per_1024_clocks(1, 150_000_000), 41);
        assert_eq!(micros_per_1024_clocks(65535, 150_000_000), 2_684_314);
        // The bus time (clocks * rate >> 10, plus 1) is never shorter than the clocks take.
        for div in [1u16, 2, 63, 1000, 65535] {
            let k = micros_per_1024_clocks(div, 150_000_000);
            for clocks in [1u64, 8, 64, 199, 1049, 65535] {
                let exact_ns = clocks * 6 * u64::from(div) * 1000 / 150;
                assert!(
                    (((clocks * k) >> 10) + 1) * 1000 >= exact_ns,
                    "div {div}, {clocks} clocks"
                );
            }
        }
    }
}
