//! Proves the tool chain, the probe and the board: boots, reports the system clock, exits 0.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    defmt::info!(
        "[INFO] alive: clk_sys = {} Hz",
        embassy_rp::clocks::clk_sys_freq()
    );
    hil_common::exit(0)
}
