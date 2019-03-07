//! Time units

use cortex_m::peripheral::DCB;
use cortex_m::peripheral::DWT;

pub use bitrate::*;
use rcc::Clocks;

/// A monotonic nondecreasing timer
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz<u32>,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, mut dcb: DCB, clocks: Clocks) -> Self {
        unsafe {
            dwt.lar.write(0xC5ACCE55);
            dwt.cyccnt.write(0);
        }
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or resetted
        drop(dwt);
        drop(dcb);
        MonoTimer { frequency: clocks.sysclk() }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(&self) -> Hertz<u32> {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(&self) -> Instant {
        Instant { now: DWT::get_cycle_count() }
    }
}

/// A measurement of a monotonically nondecreasing clock
#[derive(Clone, Copy, Debug)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(&self) -> u32 {
        DWT::get_cycle_count().wrapping_sub(self.now)
    }
}
