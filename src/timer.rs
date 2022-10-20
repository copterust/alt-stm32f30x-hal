//! Timers

use crate::pac::{TIM2, TIM3, TIM4};
use bobbin_bits::*;
use cast::{u16, u32};
use core::intrinsics::transmute;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use hal::timer::{CountDown, Periodic};
use nb;
use void::Void;

use crate::rcc::Clocks;
use crate::time::Hertz;

#[doc(hidden)]
mod private {
    #[doc(hidden)]
    pub trait Sealed {}
    impl Sealed for super::PwmFree {}
    impl Sealed for super::PwmTaken {}
    impl Sealed for super::CH1 {}
    impl Sealed for super::CH2 {}
    impl Sealed for super::CH3 {}
    impl Sealed for super::CH4 {}
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

/// Timer channel
pub trait TimerChannel {
    /// Enable channel
    fn enable(&mut self);
    /// Disable channel
    fn disable(&mut self);
    /// Read value of capture/compare register
    fn read_ccr(&self) -> u32;
    /// Read value of Timer's auto-reload register
    fn read_arr(&self) -> u32;
    /// Write value to capture/compare register
    fn write_ccr(&mut self, value: u32);
}

/// System timer
pub mod syst {
    use super::*;
    /// System timer
    pub struct Timer {
        clocks: Clocks,
        tim: SYST,
        timeout: Hertz<u32>,
    }

    impl Timer {
        /// System timer
        pub fn new<T>(mut syst: SYST, timeout: T, clocks: Clocks) -> Self
            where T: Into<Hertz<u32>>
        {
            syst.set_clock_source(SystClkSource::Core);
            let mut timer = Timer { tim: syst,
                                    clocks,
                                    timeout: Hertz(0) };
            timer.reset(timeout);
            timer
        }

        /// Resets timeout
        pub fn reset<T: Into<Hertz<u32>>>(&mut self, timeout: T) {
            self.timeout = timeout.into();
            let rvr = self.clocks.sysclk().0 / self.timeout.0 - 1;

            assert!(rvr < (1 << 24));

            self.tim.set_reload(rvr);
            self.tim.clear_current();
        }

        /// Starts listening for an `event`
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::TimeOut => self.tim.enable_interrupt(),
            }
        }

        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::TimeOut => self.tim.disable_interrupt(),
            }
        }
    }

    impl CountDown for Timer {
        type Time = Hertz<u32>;

        fn start<T>(&mut self, timeout: T)
            where T: Into<Self::Time>
        {
            self.reset(timeout);
            self.tim.enable_counter();
        }

        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.has_wrapped() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    impl Periodic for Timer {}
}

/// Trait for channel state
pub trait PwmState: private::Sealed {}
/// PwmChannels are free and not taken
pub struct PwmFree;
impl PwmState for PwmFree {}
/// PwmChannels are taken
pub struct PwmTaken;
impl PwmState for PwmTaken {}

/// Channel mode
pub trait ChMode {
    /// get bits
    fn channel_mode(&self) -> U4;
}

/// Frozen ChMode
pub struct Frozen;
impl ChMode for Frozen {
    fn channel_mode(&self) -> U4 {
        U4::B0000
    }
}

/// Active ChMode
pub struct Active;
impl ChMode for Active {
    fn channel_mode(&self) -> U4 {
        U4::B0001
    }
}

/// Inactive ChMode
pub struct Inactive;
impl ChMode for Inactive {
    fn channel_mode(&self) -> U4 {
        U4::B0010
    }
}

/// Toggle ChMode
pub struct Toggle;
impl ChMode for Toggle {
    fn channel_mode(&self) -> U4 {
        U4::B0011
    }
}

/// ForceInactive ChMode
pub struct ForceInactive;
impl ChMode for ForceInactive {
    fn channel_mode(&self) -> U4 {
        U4::B0100
    }
}

/// ForceActive ChMode
pub struct ForceActive;
impl ChMode for ForceActive {
    fn channel_mode(&self) -> U4 {
        U4::B0101
    }
}

/// Pwm1 ChMode
pub struct Pwm1;
impl ChMode for Pwm1 {
    fn channel_mode(&self) -> U4 {
        U4::B0110
    }
}

/// Pwm2 ChMode
pub struct Pwm2;
impl ChMode for Pwm2 {
    fn channel_mode(&self) -> U4 {
        U4::B0111
    }
}

/// CombinedPwm1 ChMode
pub struct CombinedPwm1;
impl ChMode for CombinedPwm1 {
    fn channel_mode(&self) -> U4 {
        U4::B1100
    }
}

/// CombinedPwm2 ChMode
pub struct CombinedPwm2;
impl ChMode for CombinedPwm2 {
    fn channel_mode(&self) -> U4 {
        U4::B1101
    }
}

/// AsymPwm1 ChMode
pub struct AsymPwm1;
impl ChMode for AsymPwm1 {
    fn channel_mode(&self) -> U4 {
        U4::B1110
    }
}

/// AsymPwm2 ChMode
pub struct AsymPwm2;
impl ChMode for AsymPwm2 {
    fn channel_mode(&self) -> U4 {
        U4::B1111
    }
}

/// Trait for channel number
pub trait ChNum: private::Sealed {
    /// get channel number
    fn channel_number() -> U2;
}
/// CH1
pub struct CH1;
impl ChNum for CH1 {
    fn channel_number() -> U2 {
        U2::B00
    }
}
/// CH2
pub struct CH2;
impl ChNum for CH2 {
    fn channel_number() -> U2 {
        U2::B01
    }
}
/// CH3
pub struct CH3;
impl ChNum for CH3 {
    fn channel_number() -> U2 {
        U2::B10
    }
}
/// CH4
pub struct CH4;
impl ChNum for CH4 {
    fn channel_number() -> U2 {
        U2::B11
    }
}

macro_rules! tim {
    ($TIMSRC:ident, $apb:ident, $timmod:ident, $timXen:ident, $timXrst:ident) => {
        /// $TIMSRC impl
        pub mod $timmod {
            use super::*;
            use crate::rcc;
            use crate::rcc::Clocks;
            use core::marker::PhantomData;

            /// Timer channel
            pub struct Channel<CN: ChNum, M: ChMode> {
                _index: PhantomData<CN>,
                _mode: PhantomData<M>,
            }

            impl<CN: ChNum, M: ChMode> TimerChannel for Channel<CN, M> {
                fn enable(&mut self) {
                    self.ccer(true)
                }

                fn disable(&mut self) {
                    self.ccer(false)
                }

                fn write_ccr(&mut self, value: u32) {
                    let index = CN::channel_number();
                    let tim = unsafe { &(*$TIMSRC::ptr()) };
                    match index {
                        U2::B00 => tim.ccr1().write(|w| unsafe {
                            w.bits(value)
                        }),
                        U2::B01 => tim.ccr2().write(|w| unsafe {
                            w.bits(value)
                        }),
                        U2::B10 => tim.ccr3().write(|w| unsafe {
                            w.bits(value)
                        }),
                        U2::B11 => tim.ccr4().write(|w| unsafe {
                            w.bits(value)
                        }),
                    }
                }

                fn read_ccr(&self) -> u32 {
                    let index = CN::channel_number();
                    let tim = unsafe { &(*$TIMSRC::ptr()) };
                    match index {
                        U2::B00 => tim.ccr1().read().bits(),
                        U2::B01 => tim.ccr2().read().bits(),
                        U2::B10 => tim.ccr3().read().bits(),
                        U2::B11 => tim.ccr4().read().bits(),
                    }
                }

                fn read_arr(&self) -> u32 {
                    let tim = unsafe { &(*$TIMSRC::ptr()) };
                    tim.arr.read().bits()
                }
            }

            impl<M: ChMode, CN: ChNum> Channel<CN, M> {
                fn ccer(&mut self, value: bool) {
                    let index: u32 = CN::channel_number().into();
                    let ccer_offset = index * 4;
                    let ccer_mask: u32 = 1;
                    let ccer_value: u32 = if value { 1 } else { 0 };
                    let tim = unsafe { &(*$TIMSRC::ptr()) };
                    tim.ccer.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(ccer_mask << ccer_offset))
                                       | ((ccer_value & ccer_mask)
                                          << ccer_offset))
                            });
                }

                /// Change channel output mode
                pub fn mode<NM: ChMode>(self, nm: NM) -> Channel<CN, NM> {
                    let tim = unsafe { &(*$TIMSRC::ptr()) };
                    let mode_bits: u8 = nm.channel_mode().into();
                    match CN::channel_number() {
                        U2::B00 => tim.ccmr1_output()
                                      .modify(|_, w| w.oc1m().bits(mode_bits)),
                        U2::B01 => tim.ccmr1_output()
                                      .modify(|_, w| w.oc2m().bits(mode_bits)),
                        U2::B10 => tim.ccmr2_output()
                                      .modify(|_, w| w.oc3m().bits(mode_bits)),
                        U2::B11 => tim.ccmr2_output()
                                      .modify(|_, w| w.oc4m().bits(mode_bits)),
                    }

                    unsafe { transmute(self) }
                }

                /// Set preload
                pub fn preload(&mut self, value: bool) {
                    let index: u32 = CN::channel_number().into();
                    let tim = unsafe { &(*$TIMSRC::ptr()) };
                    let mask = true;
                    if index < 2 {
                        let offset: u32 = 3 + (index * 4);
                        tim.ccmr1_output().modify(|r, w| unsafe {
                                              w.bits((r.bits()
                                                      & !((mask as u32)
                                                          << offset))
                                                     | (((value & mask)
                                                         as u32)
                                                        << offset))
                                          })
                    } else {
                        let offset: u32 = 3 + (index - 2) * 4;
                        tim.ccmr2_output().modify(|r, w| unsafe {
                                              w.bits((r.bits()
                                                      & !((mask as u32)
                                                          << offset))
                                                     | (((value & mask)
                                                         as u32)
                                                        << offset))
                                          })
                    };
                }
            }

            /// Timer impl
            pub struct Timer<PS: PwmState> {
                clocks: Clocks,
                tim: $TIMSRC,
                timeout: Hertz<u32>,
                _ps: PhantomData<PS>,
            }

            impl Timer<PwmFree> {
                /// Creates new timer
                pub fn new<T>(tim: $TIMSRC,
                              timeout: T,
                              clocks: Clocks)
                              -> Timer<PwmFree>
                    where T: Into<Hertz<u32>>
                {
                    let mut apb = rcc::$apb::internal_get();
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().enabled());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut t = Timer { clocks,
                                        tim,
                                        timeout: Hertz(0),
                                        _ps: PhantomData };
                    t.reset(timeout);

                    t
                }

                /// Releases the TIM peripheral
                pub fn free(self, apb: &mut rcc::$apb) -> $TIMSRC {
                    // pause counter and disable peripheral
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());
                    apb.enr().modify(|_, w| w.$timXen().disabled());

                    self.tim
                }
            }

            impl<PS: PwmState> Timer<PS> {
                /// Stop timer and reset frequency (doesn't start/enable)
                pub fn reset<T>(&mut self, timeout: T)
                    where T: Into<Hertz<u32>>
                {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // restart counter
                    self.tim.cnt.reset();
                    let timeout = timeout.into();
                    let frequency = timeout.0;
                    let mult = if self.clocks.ppre1() == 1 { 1 } else { 2 };
                    let ticks = self.clocks.pclk1().0 * mult / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| unsafe { w.bits(u32(psc)) });
                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
                    self.timeout = timeout;
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Enable timer
                pub fn enable(&mut self) {
                    // enable counter
                    self.tim.cr1.modify(|_, w| w.cen().bit(true));
                }
            }

            impl<PS: PwmState> Periodic for Timer<PS> {}

            impl<PS: PwmState> CountDown for Timer<PS> {
                type Time = Hertz<u32>;

                fn start<T>(&mut self, timeout: T)
                    where T: Into<Hertz<u32>>
                {
                    self.reset(timeout);
                    self.enable();
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Timer<PwmFree> {
                /// Consumes timer and returns pwm channels and timer without
                /// them.
                pub fn use_pwm(
                    self)
                    -> ((Channel<CH1, Inactive>,
                         Channel<CH2, Inactive>,
                         Channel<CH3, Inactive>,
                         Channel<CH4, Inactive>),
                        Timer<PwmTaken>) {
                    let ch1: Channel<CH1, Inactive> =
                        Channel { _index: PhantomData,
                                  _mode: PhantomData };
                    let ch2: Channel<CH2, Inactive> =
                        Channel { _index: PhantomData,
                                  _mode: PhantomData };
                    let ch3: Channel<CH3, Inactive> =
                        Channel { _index: PhantomData,
                                  _mode: PhantomData };
                    let ch4: Channel<CH4, Inactive> =
                        Channel { _index: PhantomData,
                                  _mode: PhantomData };

                    ((ch1, ch2, ch3, ch4), unsafe { transmute(self) })
                }
            }

            impl Timer<PwmTaken> {
                /// Returns pwm channels back.
                pub fn return_pwm<M1, M2, M3, M4>(self,
                                                  _channels: (Channel<CH1,
                                                           M1>,
                                                   Channel<CH2,
                                                           M2>,
                                                   Channel<CH3,
                                                           M3>,
                                                   Channel<CH4,
                                                           M4>))
                                                  -> Timer<PwmFree>
                    where M1: ChMode,
                          M2: ChMode,
                          M3: ChMode,
                          M4: ChMode
                {
                    unsafe { transmute(self) }
                }
            }
        }
    };
}

tim!(TIM2, APB1, tim2, tim2en, tim2rst);
tim!(TIM3, APB1, tim3, tim3en, tim3rst);
tim!(TIM4, APB1, tim4, tim4en, tim4rst);
// TODO: add other timers with different number of channels
// tim!(TIM6, APB1, tim6, tim6en, tim6rst);
// tim!(TIM7, APB1, tim7, tim7en, tim7rst);
// tim!(TIM8, APB2, tim8, tim8en, tim8rst);
