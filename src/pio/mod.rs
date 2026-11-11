//! The [`Bus`](crate::bus::Bus) on an RP235x PIO state machine and one DMA channel
//! (feature `embassy-rp-0-10`).
//!
//! All bus timing comes from the state machine. The CPU and the DMA only fill and drain FIFOs,
//! and the program is written so that a FIFO serviced late stops the clock instead of corrupting
//! a bit: late servicing can slow a transfer, never damage it.
//!
//! # The program
//!
//! One state machine runs one 21-instruction program for every operation. CLK is its side-set
//! pin and nothing else drives it, so whenever an instruction stalls the clock stops with it.
//! Every SD clock is six state-machine cycles: four with CLK low (`L`), then two high (`H`).
//! OSR and ISR both shift left (the first nibble on the wire is bits 31..28 of a word), with
//! autopull and autopush at 32 bits.
//!
//! ```text
//! .side_set 1                                ; CLK
//! .define H 2
//! .define L 4
//! .wrap_target
//! hdr:    out x, 16          side 0          ; nibbles to send (0 = none). Parks here, CLK low.
//!         out y, 16          side 0          ; nibbles to receive (0 = none)
//!         jmp x-- drive      side 0          ; x = send count - 1
//!         jmp rx             side 0
//! drive:  set pindirs, 15    side 0          ; drive the mapped line(s); their latched level is high
//! tx:     out pins, 4        side 0 [L-1]    ; launch on the falling edge
//!         jmp x-- tx         side 1 [H-1]
//!         mov pins, !null    side 0 [L-1]    ; one more nibble, all ones (the end bit)
//!         set pindirs, 0     side 1 [H-1]    ; release at the rising edge
//! rx:     mov x, y           side 0
//!         jmp x-- wait0      side 0          ; x = receive count - 1
//!         jmp done           side 0          ; nothing to receive
//! wait0:  nop                side 0 [L-1]
//!         jmp pin wait0      side 1 [H-1]    ; until the line is low (start bit)
//! rd:     nop                side 0 [L-1]
//!         in pins, 4         side 1 [H-2]    ; sample on the rising-edge instruction
//!         jmp x-- rd         side 1
//! public wait1:
//!         nop                side 0 [L-1]
//!         jmp pin done       side 1 [H-1]    ; until the line is high (end bit / not busy)
//!         jmp wait1          side 1
//! done:   push               side 0          ; the final word
//! .wrap
//! ```
//!
//! (In the source each label sits on its own line, as the `pio_asm!` grammar requires.)
//!
//! An operation is: select the pin mapping if it is not already selected, push one header word
//! `(send_nibbles << 16) | recv_nibbles` and, when sending, the stream of nibbles. The program
//! drives the lines for the send nibbles plus one all-ones end nibble, releases them on the
//! rising edge of that end nibble, clocks until the start bit (a low on the JMP pin), reads the
//! receive nibbles, clocks until the JMP pin is high, pushes exactly one final word (the ISR,
//! remaining nibbles right-aligned) and parks at `hdr` with CLK low. A final word that does not
//! arrive in the allowed time is a timeout.
//!
//! ## Why late servicing cannot corrupt a bit
//!
//! - Outputs change only on instructions with side 0, that is on or after a falling edge, and
//!   are held at least four cycles before the rising edge that the card samples on. An `out`
//!   that stalls on an empty TX FIFO stalls with CLK low: the low phase just gets longer.
//!   (Side-set takes effect on the first cycle of an instruction even if it stalls; its delay
//!   cycles start after the stall clears.)
//! - Inputs are sampled (`in pins`, `jmp pin`) only on instructions with side 1. A Default Speed
//!   card changes its outputs after the falling edge, so an `in` that stalls on a full RX FIFO
//!   stalls with CLK high and sees the same level at the start and at the end of the stall.
//! - Every receive ends at `done` with a blocking `push` (side 0) and every send ends with the
//!   OSR empty, so the next header always starts on a word boundary.
//!
//! At 150 MHz `clk_sys` and divider 1 the clock is 25 MHz: data is set up 26.7 ns before the
//! rising edge and held 13.3 ns after it. The divider is an integer (no fractional jitter):
//! `rate = clk_sys / (6 * div)`, so 396.8 kHz at divider 63 for identification. Default Speed
//! only: no CMD6, no high-speed timing, 3.3 V signalling.
//!
//! ## Two pin mappings
//!
//! - **CMD**: OUT, SET and IN base = CMD with count 1; JMP pin = CMD. `out pins, 4` drives only
//!   bit 0 of each nibble, and IN count 1 reads the other three bits of each nibble as 0.
//! - **DAT**: OUT, SET and IN base = DAT0 with count 4; JMP pin = DAT0.
//!
//! CLK is the side-set pin in both. The bus switches between the two prepared configurations
//! (`set_config`, which also jumps to `hdr`) only while parked, and only when the operation needs
//! the other one.
//!
//! ## Operations
//!
//! | Operation | Mapping | Header (send, recv) | Stream | Words received |
//! |---|---|---|---|---|
//! | `command`, short | CMD | 56, 47 | 7 words: nine `0xF` nibbles, frame bits 0..46 one per nibble | 6 |
//! | `command`, long | CMD | 56, 135 | as above | 17 |
//! | `command`, none | CMD | 56, 0 | as above, then an 8-clock idle operation | 1 + 1 |
//! | `read_block` (W words) | DAT | 0, 8W + 16 | none | W data (DMA, byte swap) + CRC hi + CRC lo + final |
//! | `write_block` (W words) | DAT | 8 + 8W + 16, 4 | `0xFFFF_FFF0`, data, CRC hi, CRC lo (one DMA, byte swap) | 1: the CRC status |
//! | `wait_ready` | DAT | none: exec `jmp wait1` | none | 1 |
//! | `clock_idle(n)` | CMD | 8 * ceil(n / 8), 0 | `0xFFFF_FFFF` words | 1 |
//!
//! The frame's 48th bit (the end bit) is the program's all-ones nibble. A response nibble `k`
//! carries response bit `k + 1` in bit 0 (the start bit is consumed by `wait0`). After a
//! response's end bit the program clocks exactly once more (the `wait1` pass), so a data block
//! that follows on DAT loses nothing. The CRC status token is bits 12, 8 and 4 of the final word
//! of a write; `wait1` then clocks until the card releases DAT0. See [`wire`] for the exact
//! packing, tested on the host.
//!
//! ## Timeouts and errors
//!
//! Each wait is bounded with `embassy_time::with_timeout` and is a phase that needs nothing from
//! the CPU: the timer starts after the CPU has done its part (pushed the header, armed the DMA),
//! so a late CPU cannot turn into a false timeout. A command allows the rest of the frame,
//! N_CR = 64 clocks, the response and 1 ms; a data operation allows the caller's `timeout_ms`,
//! the transfer and 1 ms. On any error, timeout or dropped future the bus aborts to idle: the DMA
//! transfer is dropped (aborted), the state machine disabled, `set pindirs, 0` (side 0)
//! executed, the state machine restarted, its FIFOs cleared, its OSR emptied with
//! `out null, 32` (a restart zeroes the output shift count, which means a full OSR), and it is
//! parked at `hdr` and re-enabled.
//!
//! # Constraints on the PIO block and the pins
//!
//! - CLK may be GPIO32 or above (as on MARV V2: DAT0-3 = GPIO28-31, CLK = GPIO32, CMD = GPIO33).
//!   The block's GPIOBASE must then be 16, and `set_config` sets it from the pin ranges each time
//!   the bus switches mapping. **Another state machine in the same PIO block configured with all
//!   its pins below 32 writes GPIOBASE back to 0 and breaks this bus**; give such a state machine
//!   another PIO block, or include a pin of 16 or above in its configuration.
//! - The JMP pin field is counted from GPIOBASE, like the other pin selections, so the bus
//!   writes it itself instead of using `Config::set_jmp_pin` (which stores the absolute GPIO
//!   number). Measured on an RP2354B with GPIOBASE = 16: field 17 follows GPIO33 and field 12
//!   follows GPIO28; fields 1 and 28 (the absolute numbers) do not.
//! - The input synchronisers of CMD and DAT0-3 are bypassed (the interface is synchronous to our
//!   own clock). `INPUT_SYNC_BYPASS` is shared by the block; the bus sets only its own bits,
//!   counted from GPIOBASE. Measured on an RP2354B with GPIOBASE = 16: bit 17 removes a cycle of
//!   input delay on GPIO33 and bit 12 on GPIO28; bits 1 and 28 have no effect on them.
//! - The pins get no pulls (the board has external pull-ups on CMD and DAT0-3), Schmitt
//!   triggers on, and the drive strength and slew rate from [`PioBusConfig`].

#[cfg(feature = "embassy-rp-0-10")]
mod driver;
#[cfg(feature = "embassy-rp-0-10")]
#[doc(hidden)]
#[cfg(feature = "_test-hooks")]
pub mod hooks;
pub mod wire;

#[cfg(feature = "embassy-rp-0-10")]
pub use driver::{IDENTIFICATION_HZ, PioBus, PioBusConfig};
