//! Native four-bit SD card host.
//!
//! Three layers, one seam:
//!
//! - [`proto`]: the SD protocol as pure functions. Command frames, CRC7, the four-line CRC16,
//!   responses and card registers. No hardware, no time, no allocation.
//! - [`card`]: the card driver. Identification, block reads and writes, error recovery. It talks
//!   to hardware only through [`Bus`].
//! - A bus back-end that implements [`Bus`]: the PIO + DMA bus for embassy-rp, or the simulated
//!   card in the `sim` module (feature `sim`) for host tests.
//!
//! [`Bus`] is a deliberately dumb pipe. Every protocol decision and every integrity check lives
//! above it, so all of it runs and is tested on the host.
#![cfg_attr(not(any(test, feature = "std")), no_std)]

#[cfg(feature = "block-device-driver-0-2")]
pub mod blockdev;
pub mod bus;
pub mod card;
#[cfg(any(test, feature = "embassy-rp-0-10"))]
pub mod pio;
pub mod proto;
#[cfg(feature = "sim")]
pub mod sim;

pub use bus::{Bus, BusError, ResponseBuf};
pub use card::{Block, CardInfo, Config, Error, SdCard};

#[cfg(all(doctest, feature = "sim"))]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
