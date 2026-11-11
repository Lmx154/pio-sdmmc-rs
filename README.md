# pio-sdmmc

Driver for a microSD card on a native four-bit SD bus, for the MARV V2 Rev 1 flight controller (RP2354B). It has three layers. `proto` is the SD protocol as pure functions. `card` is the driver (`SdCard`: identification, block reads and writes). Both are hardware-independent and reach the card only through the `Bus` trait. The `pio` module (feature `embassy-rp-0-10`) is the bus for the RP2350: one PIO state machine and one DMA channel. The state machine that moves the bits also makes the clock, so a late DMA transfer or a busy CPU pauses the clock instead of corrupting data.

## Usage

```rust
let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
let bus = PioBus::new(
    &mut common, sm0, dma::Channel::new(p.DMA_CH0, Irqs),
    p.PIN_32, p.PIN_33,                          // CLK, CMD
    p.PIN_28, p.PIN_29, p.PIN_30, p.PIN_31,      // DAT0-3
    PioBusConfig::default(),
);
let mut card = SdCard::new(bus, Config::default());
let info = card.init().await?;
card.write_blocks(lba, &blocks).await?;          // blocks: &[pio_sdmmc::Block]
card.read_blocks(lba, &mut blocks).await?;
```

## What the application provides

- A `Bus`. The PIO + DMA bus for embassy-rp is included (feature `embassy-rp-0-10`).
- For the included bus: a PIO block whose pins all lie in GPIO16-47, one DMA channel, and the chip feature of embassy-rp (`rp235xa` or `rp235xb`).
- The driver reads and writes blocks. Files need a filesystem on top; none is included.

Dependencies: none with the default features. `defmt`, `embassy-rp` and `embassy-time` are optional.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |
| `embassy-rp-0-10` | the PIO transport for embassy-rp; the application picks the chip feature (`rp235xa` or `rp235xb`) |
| `sim` | a simulated SD card with fault injection, for host tests of code built on the driver (needs `std`) |
| `_test-hooks` | entry points that only the on-board back-pressure test uses |

## SD card

Measured on the board at 25 MHz, 1 MiB per run, SanDisk 64 GB card:

| Blocks per call | Write | Read |
|---|---|---|
| 1 | 0.4 MB/s | 1.3 MB/s |
| 8 | 3.0 MB/s | 4.8 MB/s |
| 64 | 6.8 MB/s | 8.4 MB/s |
| 256 | 5.8 to 7.9 MB/s | 9.1 MB/s |

- Write in large calls. A call costs a few commands on top of the data, and a single-block write costs about 1.2 ms of card programming time.
- The card sometimes pauses for tens of milliseconds while it reorganises itself. A 4 KiB write takes about 1.4 ms typically; the worst seen across runs is 33 ms. Whatever writes logs has to buffer across such a pause.
- The bus needs a PIO block whose pins all lie in GPIO16-47. embassy-rp rewrites that setting each time any state machine in the block is configured, so every other user of the same PIO block must have a pin at GPIO32 or above, or SD should have a PIO block to itself.
- The driver reads and writes blocks. Files need a filesystem on top; none is included.
- On a failed transfer the driver brings the card back to its idle state once and returns the error. It does not retry.

## Tests

On the host, everything except the PIO bus itself:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh sd_bus
    tools/run.sh sd_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `sd_bus` | the PIO bus alone: identification, data in both directions, a deliberately late DMA transfer, timing of each part |
| `sd_smoke` | the card driver on the real bus: 1 MiB write and read-back, throughput by call size, write latency |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`. `sd_bus` and `sd_smoke` overwrite blocks on the card.

Pins the test uses: DAT0-DAT3 on GPIO28-31, CLK on GPIO32 (through 22 ohm R58) and CMD on GPIO33, on microSD socket J11 (10k pull-ups R36-R40 on CMD and DAT0-3, none on CLK).

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `alive`, `sd_bus` and `sd_smoke` exited 0 when last run.
- On-board test program: `cargo clippy` in `hil/` reports style findings in `sd_bus`. They do not change any result.

Not done for this device:

- A filesystem, automatic retries, High Speed mode (the bus runs at Default Speed, 25 MHz).

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver: proto, card, pio (feature embassy-rp-0-10), sim (feature sim)
tests/           host tests against the simulated card
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs, sd_bus.rs and sd_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   socket, pins, measured card facts, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The protocol follows the SD Physical Layer Simplified Specification, cited in the doc comments of `proto` and `card`. The socket datasheet is listed in `docs/device.md`.
