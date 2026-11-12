//! [`BlockDevice<512>`](block_device_driver::BlockDevice) for [`SdCard`] (feature
//! `block-device-driver-0-2`).
//!
//! The trait is the async block interface that filesystem libraries build on; for example
//! `embedded-fatfs` reaches it through `block-device-adapters`. With the feature on, a card that
//! has been initialised can be handed to such a library directly. The driver itself still
//! contains no filesystem: it stays a block driver, and with the feature off nothing in this
//! crate changes.
//!
//! An application that writes its own records straight to blocks needs no feature at all: it
//! calls [`SdCard::read_blocks`] and [`SdCard::write_blocks`] with [`Block`]s, which avoids the
//! copy described below.
//!
//! # Copying and chunking
//!
//! The trait hands over `&[Aligned<A4, [u8; 512]>]`, the driver works on [`Block`]. They are
//! different types, and this module has no `unsafe`, so every request goes through a bounce
//! buffer of 8 [`Block`]s (4 KiB, inside the future) in chunks of up to 8 blocks. A read calls
//! [`SdCard::read_blocks`] for a chunk and then copies it out; a write copies a chunk in and
//! then calls [`SdCard::write_blocks`]. A request of N blocks therefore becomes `ceil(N / 8)`
//! driver calls, each of them one multi-block command on the card, and costs one 512-byte copy
//! per block.
//!
//! # Failure
//!
//! The range of the whole request is checked against the card's capacity before the first
//! chunk, so a request that reaches past the end fails with [`Error::OutOfRange`] and moves no
//! block. After that, a failed or cancelled call follows the rules of [`SdCard::read_blocks`]
//! and [`SdCard::write_blocks`], applied to the chunk that was running. A write of several
//! chunks that fails part-way leaves the chunks before the failed one on the card (and, within
//! the failed chunk, the blocks the driver documents as written); the chunks after it were
//! never sent. A read that fails part-way leaves the buffer partly filled.

use aligned::{A4, Aligned};
use block_device_driver::BlockDevice;

use crate::bus::Bus;
use crate::card::{Block, Error, SdCard};

/// Blocks per driver call.
const CHUNK_BLOCKS: usize = 8;

impl<B: Bus> SdCard<B> {
    /// Fails with [`Error::NotInitialised`] before `init`, and with [`Error::OutOfRange`] if
    /// `blocks` blocks from `lba` reach past the card.
    fn check_range(&self, lba: u32, blocks: usize) -> Result<(), Error> {
        let capacity_blocks = self.info().ok_or(Error::NotInitialised)?.capacity_blocks;
        let end = u64::from(lba).saturating_add(u64::try_from(blocks).unwrap_or(u64::MAX));
        if end > capacity_blocks {
            return Err(Error::OutOfRange {
                lba,
                blocks,
                capacity_blocks,
            });
        }
        Ok(())
    }
}

impl<B: Bus> BlockDevice<512> for SdCard<B> {
    type Error = Error;
    type Align = A4;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<A4, [u8; 512]>],
    ) -> Result<(), Error> {
        self.check_range(block_address, data.len())?;
        let mut bounce = [Block::new(); CHUNK_BLOCKS];
        let mut lba = block_address;
        for chunk in data.chunks_mut(CHUNK_BLOCKS) {
            #[allow(clippy::indexing_slicing)]
            // bound: `chunks_mut(CHUNK_BLOCKS)` gives len <= CHUNK_BLOCKS = bounce.len()
            let staged = &mut bounce[..chunk.len()];
            self.read_blocks(lba, staged).await?;
            for (dst, src) in chunk.iter_mut().zip(staged.iter()) {
                dst.copy_from_slice(src.as_bytes());
            }
            lba = lba.saturating_add(CHUNK_BLOCKS as u32);
        }
        Ok(())
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<A4, [u8; 512]>],
    ) -> Result<(), Error> {
        self.check_range(block_address, data.len())?;
        let mut bounce = [Block::new(); CHUNK_BLOCKS];
        let mut lba = block_address;
        for chunk in data.chunks(CHUNK_BLOCKS) {
            #[allow(clippy::indexing_slicing)]
            // bound: `chunks(CHUNK_BLOCKS)` gives len <= CHUNK_BLOCKS = bounce.len()
            let staged = &mut bounce[..chunk.len()];
            for (dst, src) in staged.iter_mut().zip(chunk.iter()) {
                dst.as_bytes_mut().copy_from_slice(&src[..]);
            }
            self.write_blocks(lba, staged).await?;
            lba = lba.saturating_add(CHUNK_BLOCKS as u32);
        }
        Ok(())
    }

    async fn size(&mut self) -> Result<u64, Error> {
        let info = self.info().ok_or(Error::NotInitialised)?;
        Ok(info.capacity_blocks * Block::SIZE as u64)
    }
}
