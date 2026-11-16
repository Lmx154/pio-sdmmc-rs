# pio-sdmmc

Native four-bit SD card driver (async, `no_std`) for the MARV V2 Rev 1 flight controller (RP2354B). The protocol and the card driver (`SdCard`: identification, block reads and writes, error recovery) are hardware-independent and reach the card only through the `Bus` trait. With no feature the crate has no dependencies. Feature `embassy-rp-0-10` adds the bus for the RP2350, one PIO state machine and one DMA channel completing on `DMA_IRQ_1` (depends on `embassy-rp` 0.10 with `unstable-pac`, `embassy-time` and `atomic-waker` 1.1). Feature `sim` adds a simulated card for host tests. Feature `block-device-driver-0-2` adds an interface for filesystem libraries (depends on `block-device-driver` 0.2 and `aligned`). Feature `defmt` adds `defmt::Format`.

**Status: version 0.1.0, not published (`publish = false`), the API may still change.** One card was tested, on one board; see "Not covered".

## Works with

The protocol core and the public `Bus` trait are hardware-independent: any transport can implement `Bus`. The PIO + DMA bus is for the Raspberry Pi RP2040/RP2350, is written for `embassy-rp` 0.10 and uses `embassy-time`; it is behind the `embassy-rp-0-10` feature. No back-end for `rp2040-hal` or `rp235x-hal` exists for the PIO transport; the core and the `Bus` trait are the place to add one.

## Quick start

Against the simulated card. This block is compiled and run as a doctest (see "Tests"). It needs `embassy-futures` for `block_on`, and this crate with feature `sim`.

```rust
use embassy_futures::block_on;
use pio_sdmmc::sim::{SimCard, SimConfig};
use pio_sdmmc::{Block, Config, Error, SdCard};

# fn main() -> Result<(), Error> {
let mut card = SdCard::new(SimCard::new(SimConfig::default()), Config::default());
block_on(async {
    card.init().await?;
    let mut out = [Block::new(); 4];
    out[0].as_bytes_mut()[..5].copy_from_slice(b"hello");
    card.write_blocks(100, &out).await?;
    let mut back = [Block::new(); 4];
    card.read_blocks(100, &mut back).await?;
    assert_eq!(&back[0].as_bytes()[..5], b"hello");
    Ok::<(), Error>(())
})?;
let info = card.info().expect("initialised");
assert_eq!(info.capacity_blocks, 124_735_488);
# Ok(())
# }
```

On the board, with the PIO bus (does not compile on a host). `hil/src/bin/sd_smoke.rs` is the compiled, tested original of this set-up:

```text
use atomic_waker::AtomicWaker;
use pio_sdmmc::pio::{DmaIrqHandler, DmaWaker, PioBus, PioBusConfig};

// The waker of the bus's DMA completion: the application's static (the driver has none).
static SD_WAKER: AtomicWaker = AtomicWaker::new();
struct SdWaker;
impl DmaWaker for SdWaker {
    fn waker() -> &'static AtomicWaker {
        &SD_WAKER
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_1 => DmaIrqHandler<SdWaker>;
});
let p = embassy_rp::init(Default::default());
let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
// Built on the core that is to take DMA_IRQ_1.
let bus = PioBus::new(
    &mut common, sm0, p.DMA_CH4, Irqs,
    p.PIN_32, p.PIN_33,                      // CLK, CMD
    p.PIN_28, p.PIN_29, p.PIN_30, p.PIN_31,  // DAT0..DAT3
    PioBusConfig::default(),
);
let mut card = SdCard::new(bus, Config::default());
card.init().await?;
```

`PioBus::new` signature: `new<W: DmaWaker>(common: &mut Common<'d, PIO>, sm: StateMachine<'d, PIO, SM>, dma: Peri<'d, impl dma::ChannelInstance>, irq: impl Binding<DMA_IRQ_1, DmaIrqHandler<W>>, clk, cmd, dat0, dat1, dat2, dat3: Peri<'d, impl PioPin>, config: PioBusConfig) -> PioBus<'d, PIO, SM>`.

## What the application provides

- For the PIO bus: one PIO state machine, one DMA channel, the `DMA_IRQ_1` binding to `DmaIrqHandler<W>` and the `static` waker behind `W: DmaWaker` (above; one bus per `DMA_IRQ_1`), six pins (CLK, CMD, DAT0-3), and the chip feature of embassy-rp (`rp235xa` or `rp235xb`; the on-board programs use `rp235xb`). DAT0-3 must be consecutive GPIOs. Either all six pins are below 32 or all are 16 or above; a mix panics.
- With CLK at GPIO32 or above (MARV V2: DAT0-3 = GPIO28-31, CLK = GPIO32, CMD = GPIO33) the PIO block's GPIOBASE must be 16. Another state machine in the same PIO block configured with all its pins below 32 writes GPIOBASE back to 0 and breaks this bus: give it another PIO block, or include a pin of 16 or above in its configuration.
- The clock: the on-board programs run with `clk_sys` = 150 MHz (they print it). The PIO program makes one SD clock from six state-machine cycles with an integer divider, so 150 MHz gives the 25 MHz data clock (396.8 kHz for identification). Default Speed only.
- A card that the driver accepts: 3.3 V, native four-bit mode (see "Not covered"). The board's socket has no card-detect switch; the application learns whether a card is present from `init` (`Error::NoCard`).
- Or another bus back-end: implement `Bus` (`src/bus.rs` states the rules).

## DMA and interrupts (PIO bus)

The bus programs its one DMA channel itself, through embassy-rp's PAC, and completes on `DMA_IRQ_1`; it never builds an `embassy_rp::dma::Channel`. embassy-rp 0.10 puts every channel on `DMA_IRQ_0`, and its `Transfer::drop`, which runs after every transfer, aborts with a read-modify-write of the `CHAN_ABORT` register shared by all channels. That is unsafe when the other core runs embassy DMA transfers, as the flight core does on MARV V2.

- **Construction** (`PioBus::new`, on the core that is to take the interrupt): the channel's bit is cleared in INTE0, INTE2 and INTE3 and set in INTE1, each through the register's atomic CLR or SET alias (RP2350 datasheet 2.1.3); `DMA_IRQ_1` gets priority 3 and is enabled in the calling core's NVIC. The bus never touches `DMA_IRQ_0` or INTE0 beyond clearing its own channel's bit.
- **The handler** `DmaIrqHandler<W>`: reads INTS1, writes the same value back (write 1 to clear; only the bus channel's bit can be set there) and wakes `W::waker()`. No lock, no critical section, no panic.
- **A transfer**: READ_ADDR, WRITE_ADDR, TRANS_COUNT (word count), then CTRL_TRIG: word size, increment on the memory side only, the state machine's TX or RX DREQ, CHAIN_TO = its own channel (no chaining), byte swap as before, interrupt on completion. The task's waker is registered before the trigger. The three uses keep their order and their time-outs: the staged stream to the TX FIFO, the data block from the RX FIFO (armed before the header goes in), the response words from the RX FIFO.
- **Completion**: the channel is idle (CTRL.BUSY 0) and its raw interrupt bit has been acknowledged by the handler; then the transfer counts only with no READ_ERROR or WRITE_ERROR and the incrementing address at start + 4 x count, otherwise `BusError::Fault`. The transfer count is not used: an abort holds it at 0 (erratum RP2350-E5). Because the handler must have run, a missing interrupt shows as `BusError::Timeout`, not as a slow success. For an application that polls the bus from an interrupt executor at priority 3 or above (as urgent as `DMA_IRQ_1` or more), the handler cannot run while that executor's interrupt is active: each completion is seen only after the executor's interrupt has returned and been re-entered, and a transfer that finishes just before its deadline is reported as `BusError::Timeout` when the deadline's poll comes first. Run the bus in thread mode or in an interrupt executor below priority 3.
- **Cancel** (a transfer dropped before it finished: a time-out, an error or a dropped future), the datasheet's RP2350-E5 sequence plus pico-sdk's BUSY poll: (1) clear CTRL.EN through the CLR alias of AL1_CTRL, the control alias that does not trigger; (2) write the channel's bit to CHAN_ABORT (a plain write, never a read-modify-write); (3) spin until that CHAN_ABORT bit and CTRL.BUSY are both 0, at most 100 us; past that the bus is dead and every later call returns `BusError::Fault` until a new `PioBus` is built; (4) write the channel's bit to INTS1. This runs before the state machine is reset and before the buffer is released. A finished transfer's drop writes nothing. The 100 us bound is a worst case; in the firmware's probe of this sequence (2026-11-15, 2 000 cancels of a paced copy) the channel was at rest by the second poll every time.
- Registers shared by all channels (INTR, INTE0-3, INTF0-3, INTS0-3, CHAN_ABORT, MULTI_CHAN_TRIGGER) are written only through SET/CLR aliases or as plain writes of the bus channel's own bit: `grep -rnE 'DMA\.(intr|inte|intf|ints|chan_abort|multi_chan_trigger)\([^)]*\)\.modify' src` prints nothing.
- A late write-back by another core's embassy `Transfer::drop` (its read-modify-write of CHAN_ABORT) can still cut a transfer of this bus short; the end-address check turns that into `BusError::Fault`.

## Rules for correct use

- After a failed read or write the driver makes one clean-up attempt (CMD13; CMD12 only if the card still says it is in a data phase; wait for idle; CMD13). If the card is back in the transfer state, the original error is returned and the next call works. Otherwise the call returns `Error::UnexpectedState`; `init` is the documented remedy. The clean-up stays pending when it fails, so the next call also retries it first; when `cmd` is 55 or 23 (the two commands that close an interrupted application command failed), that retry is all that is needed.
- In a failed read, the blocks before the failed one hold valid data. `stats().blocks_written` counts only blocks whose write was confirmed.
- A dropped (cancelled) operation is recovered by the next call: it first makes the same single clean-up attempt. The host tests (`tests/card_sweeps.rs`, simulated card) drop single and multi-block reads and writes, with and without CMD23, at every await point. After an interrupted write, the test asserts that every block counted in `stats().blocks_written` holds the new data and that every other block is wholly old or wholly new.
- If an application command (CMD55 and the command after it) was cut short, the clean-up first sends CMD55 and ACMD23 for one block, so the card does not take CMD13 for an application command. That is two extra commands on the next call, and only in that case; an uninterrupted operation sends none (tested).
- A request that reaches past the card fails with `Error::OutOfRange` before any bus traffic. A request longer than one CMD23 can announce (65535 blocks) is split.
- There is no retry policy: after the one clean-up the error is returned to the caller, who decides whether to retry, call `init` or give up.
- `init` may be called again at any time to start over. It clears `info()` first, so after a failed `init` block calls return `Error::NotInitialised`.

## Logging with this driver

A filesystem is not required. There are two ways.

- Write records straight to consecutive blocks with `write_blocks`, and read them back with `read_blocks`. This is the most predictable: no feature, no layer between the application and the card, and the cost is that of the calls (see "Measured"). A host tool must then read the raw card, not a mounted volume.
- Or put a filesystem library on the optional feature `block-device-driver-0-2`. `SdCard` then implements the async `block_device_driver::BlockDevice<512>` trait (`read`, `write`, `size`), which `embedded-fatfs` reaches through `block-device-adapters`. The driver itself contains no filesystem. Checked 2026-11-11: `embedded-fatfs` is only available from git; crates.io holds a 0.0.0 placeholder. Choosing a filesystem and testing it (power loss included) is the application's job; no filesystem was tested with this driver.

Through the trait, every request goes through a bounce buffer of 8 blocks (4 KiB): N blocks become `ceil(N / 8)` driver calls and one 512-byte copy per block. A multi-chunk write that fails leaves the earlier 8-block chunks on the card; a failed read leaves the caller's buffer partly filled. Measured at 256 blocks per call (see "Measured"): 2.8 MB/s write and 4.4 MB/s read through the trait, against 6.8 and 7.9 MB/s direct. Only that case was measured through the trait. For either way, the card pauses now and then (see the latencies in "Measured"); the writer has to buffer across the pause.

## What the driver does

- Identification: `init` returns `CardInfo`, `info()` returns it afterwards. It holds the capacity class (`CapacityClass`: SDSC, SDHC, SDXC), capacity in blocks, RCA, OCR, CID, CSD, SCR, the data clock the bus granted, and whether multi-block transfers use CMD23 (from the SCR). `init` also switches the card to four-bit mode.
- `read_blocks` and `write_blocks` on 512-byte `Block`s: single and multi-block, with CMD23 when the card supports it, otherwise open-ended with CMD12 (a write then sends ACMD23 with the block count first). CRC16 is checked on reads, the CRC status token on writes, and the card status after each chunk.
- `status()` (CMD13, error bits included; it also runs the pending clean-up first, so after a dropped call it costs extra commands), `stats()` (`Stats`: commands sent, blocks read and written, read CRC errors, write rejections, timeouts, bad responses, recoveries), `bus()`, `release()`.
- `Config`: clock caps (identification at most 400 kHz, data at most 25 MHz) and timeouts: ACMD41 polling 1000 ms, start of a read block 100 ms, each write wait 500 ms (the defaults).
- `Error` (also `Display`, `core::error::Error`, and `defmt::Format` with `defmt`): `NotInitialised`, `NoCard`, `Unsupported { cmd, reason }`, `InitTimeout { waited_ms }`, `Bus { cmd, lba, error }`, `BadResponse`, `CardStatus { status }`, `WrongState { state }`, `DataCrc { expected, got }`, `WriteRejected { token }`, `OutOfRange`, `UnexpectedState`. Most carry the command index `cmd` and the block address `lba`.
- The simulated card (feature `sim`, host only): `SimCard` is a `Bus` that models the card in virtual time, records a trace, and lists protocol violations. `inject(Fault, Trigger)` adds faults: no response, flipped response or data or CRC bit, CRC-error and write-error tokens, long busy, status error bits, card removal, ACMD41 never ready. `tests/card_faults.rs` has a driver-level test for every fault kind.
- The block-device interface (feature `block-device-driver-0-2`): see "Logging".

## Not covered

- High Speed mode (CMD6): the bus runs at Default Speed, 25 MHz at most; the PIO program samples on Default Speed timing.
- Erase and discard, SD Status (ACMD13), and counting the blocks written by a failed multi-block write (ACMD22). Command builders exist in `proto`; the driver does not use them.
- A retry policy (see "Rules"). Timeouts are the fixed `Config` values, not taken from the card class.
- Card detect (the board has none) and a probe call: a missing card shows only as an error from `init` or from an operation.
- Cards that `init` refuses with `Error::Unsupported`: CMD8 voltage range not 2.7-3.6 V, a wrong CMD8 check pattern, SDUC, a CSD that cannot be read, OCR capacity bit and CSD version that disagree, an SCR without four-bit width, RCA 0 on every request. Not handled either: SPI mode, 1.8 V and UHS-I, card lock, the write-protect switch.
- The no-panic lint block the sensor drivers have is in place, but its lints do not cover `assert!`: the PIO back-end still asserts two caller preconditions (a data block of 4, 8, 64 or 512 bytes in `src/pio/driver.rs`; the response word count in `pio::wire::unpack_response`). Neither depends on what the card sends.

### Not verified on real hardware

- Recovery after an interrupted application command has run on ONE real card (`sd_faults`, 2026-11-11, the SanDisk 64 GB card): the pair was cut between CMD55 and ACMD23 52 times per run, the card then answered a raw CMD13 with its APP_CMD bit set (6 of 6), the clean-up's CMD55 + ACMD23 pair was accepted, and the next operation succeeded with correct data every time. That card supports CMD23, so the driver takes the ACMD23 route on it only because the test hides the SCR's CMD23 bit from the driver; a card that really lacks CMD23 has not been tested. The rule the clean-up relies on (a command after CMD55 is an application command only when one with that index exists) was not re-read in the specification.
- Cancellation on the real bus is covered in part. `sd_faults` drops driver futures on the real card: about 400 reads and 390 writes per run, cut by time and at every suspension point, with the first operation after each cancel being a status call, a read or a write in turn. The driver cleaned up by itself every time: 0 failed next operations, 0 wrong bytes, 0 bytes damaged outside the written range (three runs). Every one of those cancels landed in the data phase or between two operations. None landed inside a command frame or in a busy wait on the ordinary route (0 of 70 attempts: on this back-end the command future was never pending when polled), so those cases, and read CRC errors, rejected writes, timeouts, card removal and power loss, are proven on the simulated card only. At bus level, `sd_bus` does show that a command the card ignores times out and leaves the bus usable.
- Cards other than the one tested: a SanDisk 64 GB SDXC (CID product "SK64G"). No SDSC or SDHC card, no other manufacturer.
- Any other board, pins or `clk_sys`.

## Measured

**The figures below were measured with the earlier back-end (embassy-rp's `dma::Channel` on `DMA_IRQ_0`, 2026-11-11).** The `DMA_IRQ_1` back-end ran against the same card on 2026-11-15: `sd_bus`, `sd_smoke`, `sd_blockdev` and `sd_faults` (two runs) all end `RESULT: PASS`, and the negative control (`sd_bus` with `negative-no-inte1`) fails as intended, CMD0 with `Err(Timeout)` before any write (`hil/logs/*-20261115-22*.log`). Its throughput differs from the figures below by more than 10 %, in both directions and not the same way in `sd_smoke` and `sd_blockdev` (for example `sd_smoke` 256-block calls: write 8.97 MB/s, read 7.56 MB/s; `sd_blockdev` direct: write 5.86 MB/s, read 9.67 MB/s). The figures below stay until that difference is understood. Its register sequence (set-up, trigger, completion test, cancel) was run on the board in the firmware's probe P2 (2026-11-15, memory-to-memory copies on channel 4 while the flight core ran its IMUs on channels 0-3: 100 000 completions, 2 000 cancels, 0 errors).

MARV V2 Rev 1 board, RP2354B, embassy-rp 0.10, one microSD card (the SanDisk above), `clk_sys` 150 MHz, 25 MHz data clock. Single runs, not statistics.

- `sd_blockdev`, 2026-11-11 (`hil/logs/sd_blockdev-20261111-135754.log`), 256 blocks per call, 1024 blocks in all (4 x 256), so a different transfer size and run from the `sd_smoke` rows below, which explains the different direct-path figures: direct `write_blocks` 6.8 MB/s and `read_blocks` 7.9 MB/s; through the trait 2.8 MB/s write and 4.4 MB/s read.
- `sd_smoke`, 2026-11-11 (`hil/logs/sd_smoke-20261111-135811.log`), 1 MiB per row:

| Blocks per call | Write | Read |
|---|---|---|
| 1 | 0.39 MB/s (longest call 32.2 ms) | 1.29 MB/s |
| 8 | 2.93 MB/s | 4.93 MB/s |
| 64 | 6.54 MB/s | 8.42 MB/s |
| 256 | 7.53 MB/s | 9.11 MB/s |

- Same run: 256 consecutive 4 KiB (8-block) writes, mean 1347 us per call, worst 1465 us.
- `sd_bus`, 2026-11-11 (`hil/logs/sd_bus-20261111-135820.log`), raw bus calls, no card driver: 2048-block write 9.20 MB/s and read 11.09 MB/s (bus calls only); wire time per block at 25 MHz, computed (1042 clocks per read block, printed by `sd_bus`, not measured): 41.7 us, 12.29 MB/s. Single-block read: mean 215 us, worst 514 us. Single-block write including busy: mean 1333 us, worst 3940 us.
- Identification facts of the tested card (OCR 0xC0FF8000, 124735488 blocks, ACMD41 ready after about 14 ms) are in `docs/device.md`.

## Tests

On the host (everything except the PIO bus itself): `cargo test --no-fail-fast`; plain `cargo test` stops at the first failing test binary. No flag is needed for `sim`, because the dev-dependencies turn it on. Observed 2026-11-11: with the default features cargo prints 148 passed and 1 ignored (a timing test) for the unit and integration tests; with `--features block-device-driver-0-2`, 155 passed. The README doctest (`cargo test --doc --features sim`) is counted separately.

- Fault tests (`tests/card_faults.rs`, `tests/card_failures.rs`): the fault kinds of the simulated card and the `Unsupported` reasons, through the driver. Each asserts the exact error and that the next call works. Sweeps (`tests/card_sweeps.rs`): a bus failure at every phase and a dropped future at every await, for single and multi-block reads and writes, with and without CMD23. `tests/blockdev.rs` covers the trait.
- The Rust example in this file is compiled and run by `cargo test --doc --features sim`; a hook in `src/lib.rs` includes this file as a doctest when `doctest` and `sim` are set.

On the board, through a debug probe: `tools/run.sh <program>` (it takes a lock, so runs never overlap). Used here: a CMSIS-DAP debug probe on the board's SWD header. Exit status 0 means every check passed; logs go to `hil/logs/`.

| Program | What it proves |
|---|---|
| `alive` | tool chain, probe and board |
| `sd_bus` | the PIO bus alone: identification at 400 kHz, reads, writes, a deliberately late DMA transfer, speed at 25 MHz. With hil feature `negative-no-inte1` (NEGATIVE CONTROL) the channel is taken out of INTE1 after `PioBus::new`: every DMA transfer must time out, so identification fails |
| `sd_smoke` | the card driver on the real bus: identification, 1 MiB write and read-back compare, throughput by call size, 4 KiB write latency, `OutOfRange` then a normal read |
| `sd_blockdev` | the trait on the real bus: round trips against the direct path, a request across the 8-block chunk boundary, trait against direct throughput |
| `sd_faults` | recovery on the real card: reads and writes cancelled by time and at every suspension point, the application-command pair cut between CMD55 and ACMD23, positive controls that find the card not ready when the clean-up is left out, card identity and data intact at the end |

`sd_smoke`, `sd_blockdev` and `sd_faults` write only in a fixed scratch area: 1 MiB from block 50000000 (blocks 50000000 to 50002047). They overwrite it. `sd_bus` also overwrites blocks, at random addresses between block 1000000 and 1000000 blocks before the end of the card, and from block 8000000. Use an expendable card.

## Cargo features

All are off by default.

| Feature | What it enables | When to use |
|---|---|---|
| `std` | `std` in the crate | turned on by `sim` |
| `sim` | the simulated card `sim`, with fault injection (host only) | tests of this crate and of code built on it |
| `defmt` | `defmt::Format` on the public types, and defmt in embassy-rp and embassy-time when those are on | on target, if the application uses defmt |
| `embassy-rp-0-10` | the PIO + DMA bus `pio::PioBus` with `DmaIrqHandler` and `DmaWaker` (embassy-rp 0.10 with `unstable-pac`, embassy-time, atomic-waker 1.1) | on the RP2350; the application picks the chip feature |
| `block-device-driver-0-2` | `BlockDevice<512>` for `SdCard` (block-device-driver 0.2, aligned) | to put a filesystem library on the driver |
| `_test-hooks` | entry points only the on-board back-pressure test uses | test-only, outside any promise |

## Layout

```text
src/            proto (SD protocol), card (driver), bus.rs (Bus trait), pio (embassy-rp-0-10),
                sim (sim), blockdev.rs (block-device-driver-0-2)
tests/          host tests against the simulated card
hil/            on-board programs alive, sd_bus, sd_smoke, sd_blockdev (own package and workspace)
tools/run.sh    builds, flashes and runs one on-board program
docs/device.md  socket, pins, measured card facts
```

## License

MIT, see `LICENSE`. The protocol follows the SD Physical Layer Simplified Specification, cited in the doc comments of `proto` and `card`.
