//! The bus's one DMA channel, programmed through the PAC and completing on `DMA_IRQ_1`.
//!
//! embassy-rp 0.10 routes every channel to `DMA_IRQ_0` and aborts with a read-modify-write of the
//! shared `CHAN_ABORT`, which is unsafe when the other core runs embassy transfers. This channel
//! never touches `DMA_IRQ_0`, and of INTE0 only clears its own bit. It writes the registers shared
//! across channels only through the atomic SET/CLR aliases (INTE0-3) or as plain writes of its own
//! bit (INTS1, CHAN_ABORT).

use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::pin::Pin;
use core::sync::atomic::{Ordering, fence};
use core::task::{Context, Poll};

use atomic_waker::AtomicWaker;
use embassy_rp::Peri;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::interrupt::typelevel::{Binding, DMA_IRQ_1, Handler};
use embassy_rp::interrupt::{self, InterruptExt, Priority};
use embassy_rp::pac;
use embassy_rp::pac::common::{RW, Reg};
use embassy_rp::pac::dma::regs::{ChanAbort, CtrlTrig};
use embassy_rp::pac::dma::vals::{DataSize, TransCountMode, TreqSel};
use embassy_time::{Duration, Instant};

use crate::bus::BusError;

/// The longest a cancelled transfer may take to come to rest before the bus is marked dead.
const QUIESCE_LIMIT: Duration = Duration::from_micros(100);
/// CTRL.EN.
const CTRL_EN: u32 = 1;

/// Where the waker of the bus's DMA completion lives. The application provides the `static` and
/// this impl (the driver keeps no static state):
///
/// ```text
/// static SD_WAKER: AtomicWaker = AtomicWaker::new();
/// struct SdWaker;
/// impl DmaWaker for SdWaker {
///     fn waker() -> &'static AtomicWaker {
///         &SD_WAKER
///     }
/// }
/// ```
pub trait DmaWaker: 'static {
    /// The waker the interrupt handler wakes. One per bus.
    fn waker() -> &'static AtomicWaker;
}

/// The `DMA_IRQ_1` handler: acknowledges what INTS1 shows (only the bus channel's bit is enabled
/// there) and wakes [`DmaWaker::waker`]. Bind it with
/// `bind_interrupts!(struct Irqs { DMA_IRQ_1 => DmaIrqHandler<SdWaker>; })`.
pub struct DmaIrqHandler<W: DmaWaker> {
    _waker: PhantomData<W>,
}

impl<W: DmaWaker> Handler<DMA_IRQ_1> for DmaIrqHandler<W> {
    unsafe fn on_interrupt() {
        let ints = pac::DMA.ints(1).read();
        pac::DMA.ints(1).write_value(ints);
        W::waker().wake();
    }
}

/// Writes `bits` to the atomic SET alias of `reg`: the peripheral sets exactly those bits in place.
fn write_set(reg: Reg<u32, RW>, bits: u32) {
    // SAFETY: RP2350 datasheet 2.1.3 "Atomic register access": every peripheral register block has
    // an atomic bitmask-set alias at +0x2000 and a clear alias at +0x3000; the DMA block is not
    // among the exceptions (SIO, the CoreSight window, the PPB, OTP's SBPI). `reg` comes from the
    // PAC, so its alias is a register too. embassy-rp 0.10.0 does the same in its private `RegExt`
    // (`lib.rs:722-761`, used at `dma.rs:55`).
    unsafe { Reg::<u32, RW>::from_ptr(reg.as_ptr().cast::<u8>().wrapping_add(0x2000).cast()) }
        .write_value(bits);
}

/// Writes `bits` to the atomic CLR alias of `reg`: the peripheral clears exactly those bits in place.
fn write_clear(reg: Reg<u32, RW>, bits: u32) {
    // SAFETY: as `write_set`, the clear alias at +0x3000.
    unsafe { Reg::<u32, RW>::from_ptr(reg.as_ptr().cast::<u8>().wrapping_add(0x3000).cast()) }
        .write_value(bits);
}

/// The bus's DMA channel. Built on the core that is to take its interrupt.
pub(super) struct DmaChannel {
    number: u8,
    waker: &'static AtomicWaker,
    /// A cancelled transfer did not come to rest within [`QUIESCE_LIMIT`]: no transfer starts
    /// again on this channel.
    dead: bool,
}

/// Which side of a transfer is memory, and so increments.
#[derive(Clone, Copy)]
enum Memory {
    /// Memory to a FIFO: READ_ADDR increments.
    Read,
    /// A FIFO to memory: WRITE_ADDR increments.
    Write,
}

impl DmaChannel {
    /// Takes `ch` out of INTE0, INTE2 and INTE3 (CLR alias), puts it in INTE1 (SET alias), and
    /// enables `DMA_IRQ_1` at priority 3 in the calling core's NVIC.
    pub(super) fn new<'d, T: ChannelInstance, W: DmaWaker>(
        _ch: Peri<'d, T>,
        _irq: impl Binding<DMA_IRQ_1, DmaIrqHandler<W>>,
    ) -> Self {
        let number = T::number();
        let bit = 1u32 << number;
        for n in [0, 2, 3] {
            write_clear(pac::DMA.inte(n), bit);
        }
        write_set(pac::DMA.inte(1), bit);
        interrupt::DMA_IRQ_1.set_priority(Priority::P3);
        // SAFETY: `DmaIrqHandler` is bound to `DMA_IRQ_1` (the `Binding`); it takes no lock and
        // touches only INTS1 and the waker.
        unsafe { interrupt::DMA_IRQ_1.enable() };
        Self {
            number,
            waker: W::waker(),
            dead: false,
        }
    }

    /// Whether a cancel failed to bring the channel to rest; every operation then fails.
    pub(super) fn dead(&self) -> bool {
        self.dead
    }

    /// Registers the calling task's waker, so that it is in place before the transfer is
    /// triggered.
    pub(super) async fn register(&self) {
        poll_fn(|cx| {
            self.waker.register(cx.waker());
            Poll::Ready(())
        })
        .await
    }

    /// Starts sending `words` to the FIFO at `fifo`, paced by `treq`.
    pub(super) fn push<'a>(
        &'a mut self,
        words: &'a [u32],
        fifo: *mut u32,
        treq: TreqSel,
        bswap: bool,
    ) -> Result<Transfer<'a>, BusError> {
        let start = words.as_ptr() as u32;
        self.start(start, fifo as u32, words.len(), Memory::Read, treq, bswap)
    }

    /// Starts receiving `words.len()` words from the FIFO at `fifo` into `words`, paced by `treq`.
    pub(super) fn pull<'a>(
        &'a mut self,
        fifo: *const u32,
        words: &'a mut [u32],
        treq: TreqSel,
        bswap: bool,
    ) -> Result<Transfer<'a>, BusError> {
        let start = words.as_mut_ptr() as u32;
        self.start(fifo as u32, start, words.len(), Memory::Write, treq, bswap)
    }

    fn start<'a>(
        &'a mut self,
        read: u32,
        write: u32,
        len: usize,
        memory: Memory,
        treq: TreqSel,
        bswap: bool,
    ) -> Result<Transfer<'a>, BusError> {
        if self.dead {
            return Err(BusError::Fault);
        }
        let count = len as u32;
        let ch = pac::DMA.ch(usize::from(self.number));
        ch.read_addr().write_value(read);
        ch.write_addr().write_value(write);
        ch.trans_count().write(|w| {
            w.set_mode(TransCountMode::NORMAL);
            w.set_count(count);
        });
        let mut c = CtrlTrig(0);
        c.set_treq_sel(treq);
        c.set_data_size(DataSize::SIZE_WORD);
        c.set_incr_read(matches!(memory, Memory::Read));
        c.set_incr_write(matches!(memory, Memory::Write));
        c.set_chain_to(self.number);
        c.set_bswap(bswap);
        c.set_irq_quiet(false);
        c.set_en(true);
        let end = match memory {
            Memory::Read => read,
            Memory::Write => write,
        }
        .wrapping_add(4 * count);
        // The buffer's last CPU writes land before the channel reads it.
        fence(Ordering::SeqCst);
        ch.ctrl_trig().write_value(c);
        Ok(Transfer {
            channel: self,
            memory,
            end,
            finished: false,
            _buffer: PhantomData,
        })
    }
}

/// One transfer in flight. Ready when the channel is idle and its interrupt has been taken;
/// dropped before that, it cancels the transfer (RP2350-E5 sequence).
pub(super) struct Transfer<'a> {
    channel: &'a mut DmaChannel,
    memory: Memory,
    /// Where the incrementing address must stand when every word has moved.
    end: u32,
    /// The channel was seen idle: dropping writes nothing.
    finished: bool,
    _buffer: PhantomData<&'a mut [u32]>,
}

impl Future for Transfer<'_> {
    type Output = Result<(), BusError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        this.channel.waker.register(cx.waker());
        let bit = 1u32 << this.channel.number;
        let ch = pac::DMA.ch(usize::from(this.channel.number));
        let c = ch.ctrl_trig().read();
        // The raw interrupt stays set until `DmaIrqHandler` acknowledges it.
        if c.busy() || pac::DMA.intr(0).read() & bit != 0 {
            return Poll::Pending;
        }
        this.finished = true;
        // The channel's last writes are seen before the CPU reads the buffer.
        fence(Ordering::SeqCst);
        let reached = match this.memory {
            Memory::Read => ch.read_addr().read(),
            Memory::Write => ch.write_addr().read(),
        };
        if c.read_error() || c.write_error() || reached != this.end {
            Poll::Ready(Err(BusError::Fault))
        } else {
            Poll::Ready(Ok(()))
        }
    }
}

impl Drop for Transfer<'_> {
    fn drop(&mut self) {
        if self.finished {
            return;
        }
        let number = self.channel.number;
        let bit = 1u32 << number;
        let ch = pac::DMA.ch(usize::from(number));
        write_clear(ch.al1_ctrl(), CTRL_EN);
        pac::DMA.chan_abort().write_value(ChanAbort(bit));
        let began = Instant::now();
        loop {
            let aborting = u32::from(pac::DMA.chan_abort().read().chan_abort()) & bit != 0;
            if !aborting && !ch.ctrl_trig().read().busy() {
                break;
            }
            if began.elapsed() > QUIESCE_LIMIT {
                self.channel.dead = true;
                break;
            }
        }
        pac::DMA.ints(1).write_value(bit);
    }
}
