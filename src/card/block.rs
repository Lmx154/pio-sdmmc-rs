use crate::bus::{as_bytes, as_bytes_mut};

/// One 512-byte data block, stored as 128 words so it is aligned for DMA.
///
/// Byte `i` of the block is the byte at offset `i` of its memory, which is the order the bytes
/// go on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Block([u32; 128]);

impl Block {
    /// Bytes in a block.
    pub const SIZE: usize = 512;

    /// A block of zeros.
    pub const fn new() -> Self {
        Self([0; 128])
    }

    /// The block's 512 bytes.
    pub fn as_bytes(&self) -> &[u8] {
        as_bytes(&self.0)
    }

    /// The block's 512 bytes, writable.
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        as_bytes_mut(&mut self.0)
    }

    pub(crate) fn words(&self) -> &[u32] {
        &self.0
    }

    pub(crate) fn words_mut(&mut self) -> &mut [u32] {
        &mut self.0
    }
}

impl Default for Block {
    fn default() -> Self {
        Self::new()
    }
}

impl From<[u8; 512]> for Block {
    fn from(bytes: [u8; 512]) -> Self {
        let mut block = Self::new();
        for (dst, src) in block.as_bytes_mut().iter_mut().zip(bytes) {
            *dst = src;
        }
        block
    }
}
