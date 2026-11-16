//! What every on-target test shares: reporting the verdict to the host, and the SD bus's DMA
//! waker.
#![no_std]

use atomic_waker::AtomicWaker;
use pio_sdmmc::pio::DmaWaker;

/// The waker of the SD bus's DMA completion (the driver keeps no static state).
static SD_WAKER: AtomicWaker = AtomicWaker::new();

/// Every program binds `DMA_IRQ_1 => DmaIrqHandler<SdWaker>`.
pub struct SdWaker;

impl DmaWaker for SdWaker {
    fn waker() -> &'static AtomicWaker {
        &SD_WAKER
    }
}

/// NEGATIVE CONTROL (feature `negative-no-inte1`): takes `channel` out of INTE1 again, through the
/// atomic CLR alias (RP2350 datasheet 2.1.3), so the bus's completion interrupt never comes.
#[cfg(feature = "negative-no-inte1")]
pub fn negative_no_inte1(channel: u8) {
    use embassy_rp::pac;
    use embassy_rp::pac::common::{RW, Reg};
    let inte1 = pac::DMA.inte(1);
    // SAFETY: the CLR alias of INTE1 (+0x3000) is a register of the DMA block.
    unsafe { Reg::<u32, RW>::from_ptr(inte1.as_ptr().cast::<u8>().wrapping_add(0x3000).cast()) }
        .write_value(1 << channel);
    defmt::warn!(
        "[INFO] NEGATIVE CONTROL: channel {} taken out of INTE1 (INTE1 now {:#010x}); every DMA transfer must time out",
        channel,
        inte1.read()
    );
}

/// Logs the verdict and ends the `probe-rs run` session through semihosting SYS_EXIT.
/// The host exit status is 0 when `failures == 0`, otherwise 1.
pub fn exit(failures: u32) -> ! {
    if failures == 0 {
        defmt::info!("RESULT: PASS");
    } else {
        defmt::error!("RESULT: FAIL ({} failed checks)", failures);
    }
    semihosting::process::exit(if failures == 0 { 0 } else { 1 })
}
