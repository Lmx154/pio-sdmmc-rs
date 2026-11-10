//! The seam between the SD protocol and the hardware that moves the bits.
//!
//! A [`Bus`] knows nothing about SD commands. It sends a 48-bit frame and returns the bits that
//! came back; it moves one data block and hands over the CRC bits it saw. Building frames,
//! checking CRCs, interpreting responses and deciding what to do next all happen in the card
//! layer above. That keeps a back-end small, and it means the logic that matters is the logic
//! the host tests exercise.
//!
//! # Rules every implementation follows
//!
//! - The data bus is always four bits wide. The card layer switches the card to four-bit mode
//!   (ACMD6) before the first data transfer and never moves data in one-bit mode.
//! - The clock only runs while an operation is in progress. Between operations it is low.
//! - An operation that returns an error leaves the bus idle: clock low, CMD and DAT released.
//! - A wait is never open-ended. Each one ends in success or in [`BusError::Timeout`].
//! - Dropping a future before it completes is memory-safe and leaves the bus idle, but the card
//!   may be part-way through a transfer. The card layer treats that as needing recovery.
#![allow(async_fn_in_trait)]

/// Why a single bus operation failed.
///
/// A response or a data block that arrives damaged is not a bus error: the bus returns the bits
/// as received and the card layer checks them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusError {
    /// The card did not act in time: no response start bit, no data start bit, no CRC status,
    /// or DAT0 still low (busy) when the time allowed ran out.
    Timeout,
    /// The hardware reported a fault of its own, for example a DMA bus error.
    Fault,
}

/// Where a command's response goes, which also says how long a response to expect.
#[derive(Debug)]
pub enum ResponseBuf<'a> {
    /// The command has no response. The bus clocks at least eight cycles after the frame.
    None,
    /// A 48-bit response (R1, R1b, R3, R6, R7).
    Short(&'a mut [u8; 6]),
    /// A 136-bit response (R2: CID or CSD).
    Long(&'a mut [u8; 17]),
}

/// Four-bit SD bus, one operation at a time.
///
/// # Wire formats
///
/// **Frames** are bytes in wire order, most significant bit first. A command frame is
/// `[0x40 | index, arg[31:24], arg[23:16], arg[15:8], arg[7:0], (crc7 << 1) | 1]`. A response
/// buffer is filled the same way, starting with the byte that holds the start bit and the
/// transmission bit (both 0 in a valid response).
///
/// **Data blocks** are typed `[u32]` only to guarantee the alignment and length that DMA needs.
/// Byte `i` of the block is the byte at offset `i` of the slice's memory, whatever the target's
/// endianness; view it with [`as_bytes`] and [`as_bytes_mut`]. On the wire each byte goes high
/// nibble first, and a nibble puts its bit 3 on DAT3 down to bit 0 on DAT0. Implementations
/// accept blocks of 1, 2, 16 and 128 words (4, 8, 64 and 512 bytes) and may panic on any other
/// length.
///
/// **Data CRCs** are the 64 bits that follow a block on the wire, as one `u64`: sixteen nibbles,
/// the first one sent in bits 63..60 and the last in bits 3..0. Bit `k` of every nibble belongs
/// to the CRC16 of line DAT`k`, most significant CRC bit first.
pub trait Bus {
    /// Sets the clock to the fastest rate the hardware can make that does not exceed `max_hz`,
    /// and returns that rate in hertz. Takes effect from the next operation.
    fn set_clock(&mut self, max_hz: u32) -> u32;

    /// Runs `cycles` clock cycles with CMD and DAT0-3 high. Used for the power-up clocks.
    async fn clock_idle(&mut self, cycles: u32);

    /// Sends one command frame and receives its response.
    ///
    /// Returns [`BusError::Timeout`] if a response was expected and no start bit appeared within
    /// 64 clock cycles (N_CR) of the end of the frame. The implementation may allow more than 64
    /// cycles, never fewer. The clock stops within one cycle of the response's end bit, so the
    /// first cycle of a data block that follows is not lost.
    async fn command(&mut self, frame: &[u8; 6], response: ResponseBuf<'_>)
    -> Result<(), BusError>;

    /// Receives one data block into `data` and returns the CRC bits that followed it.
    ///
    /// Clocks until the start bit appears on DAT0, for at most `timeout_ms`. The CRC is returned
    /// as received and is not checked here.
    async fn read_block(&mut self, data: &mut [u32], timeout_ms: u32) -> Result<u64, BusError>;

    /// Sends one data block followed by `crc`, then returns the card's 3-bit CRC status token
    /// once the card has released DAT0.
    ///
    /// `0b010` means accepted, `0b101` CRC error, `0b110` write error; the token is returned as
    /// received and is not interpreted here. Returns [`BusError::Timeout`] if no CRC status
    /// starts, or if DAT0 is still low `timeout_ms` after the token.
    async fn write_block(
        &mut self,
        data: &[u32],
        crc: u64,
        timeout_ms: u32,
    ) -> Result<u8, BusError>;

    /// Clocks until DAT0 is high (the card is not busy), for at most `timeout_ms`.
    async fn wait_ready(&mut self, timeout_ms: u32) -> Result<(), BusError>;

    /// Waits `ms` milliseconds with the clock stopped.
    async fn delay_ms(&mut self, ms: u32);
}

/// Views a data block as bytes in wire order.
pub fn as_bytes(words: &[u32]) -> &[u8] {
    // SAFETY: `u8` has alignment 1 and no invalid bit patterns, the length covers exactly the
    // memory of `words`, and the returned borrow holds `words` for its whole lifetime.
    unsafe {
        core::slice::from_raw_parts(words.as_ptr().cast::<u8>(), core::mem::size_of_val(words))
    }
}

/// Views a data block as mutable bytes in wire order.
pub fn as_bytes_mut(words: &mut [u32]) -> &mut [u8] {
    // SAFETY: as for `as_bytes`; every bit pattern is also a valid `u32`, so writes through the
    // byte view cannot create an invalid value, and the exclusive borrow is carried over.
    unsafe {
        core::slice::from_raw_parts_mut(
            words.as_mut_ptr().cast::<u8>(),
            core::mem::size_of_val(words),
        )
    }
}
