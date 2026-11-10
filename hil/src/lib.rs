//! The one thing every on-target test shares: reporting the verdict to the host.
#![no_std]

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
