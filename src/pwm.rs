//! Pwm
use core::marker::PhantomData;
use core::mem;

use cast::u16;
use hal;
use stm32f30x::{TIM2, TIM3, TIM4};

use bb;
use gpio::gpioa::{PA0, PA1, PA2, PA3, PA6, PA7};
use gpio::gpiob::{PB0, PB1, PB6, PB7, PB8, PB9};
use gpio::{AF1, AF2};
use rcc::{APB1, Clocks};
use time::Hertz;

/// Pins
pub trait Pins<TIM> {
    /// C1
    const C1: bool;
    /// C2
    const C2: bool;
    /// C3
    const C3: bool;
    /// C4
    const C4: bool;
    /// channels
    type Channels;
}

impl Pins<TIM2> for (PA0<AF1>, PA1<AF1>, PA2<AF1>, PA3<AF1>) {
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM2, C1>, Pwm<TIM2, C2>, Pwm<TIM2, C3>, Pwm<TIM2, C4>);
}

impl Pins<TIM2> for PA0<AF2> {
    const C1: bool = true;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C1>;
}

impl Pins<TIM3> for (PA6<AF2>, PA7<AF2>, PB0<AF2>, PB1<AF2>) {
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM3, C1>, Pwm<TIM3, C2>, Pwm<TIM3, C3>, Pwm<TIM3, C4>);
}

impl Pins<TIM3> for (PB0<AF2>, PB1<AF2>) {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM3, C3>, Pwm<TIM3, C4>);
}

impl Pins<TIM4> for (PB6<AF2>, PB7<AF2>, PB8<AF2>, PB9<AF2>) {
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM4, C1>, Pwm<TIM4, C2>, Pwm<TIM4, C3>, Pwm<TIM4, C4>);
}

/// PwmExt
pub trait PwmExt: Sized {
    /// pwm
    fn pwm<PINS, T>(self, PINS, frequency: T, clocks: Clocks, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;
}

impl PwmExt for TIM2 {
    fn pwm<PINS, T>(self, _pins: PINS, freq: T, clocks: Clocks, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        tim2(self, _pins, freq.into(), clocks, apb)
    }
}

impl PwmExt for TIM3 {
    fn pwm<PINS, T>(self, _pins: PINS, freq: T, clocks: Clocks, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        tim3(self, _pins, freq.into(), clocks, apb)
    }
}

impl PwmExt for TIM4 {
    fn pwm<PINS, T>(self, _pins: PINS, freq: T, clocks: Clocks, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        tim4(self, _pins, freq.into(), clocks, apb)
    }
}

/// Pwm<Tim, Channel>
pub struct Pwm<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

/// C1
pub struct C1;
/// C2
pub struct C2;
/// C3
pub struct C3;
/// C4
pub struct C4;

const PWM1: u8 = 6;
// const PWM2: u8 = 7;

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clocks: Clocks,
                apb: &mut APB1,
            ) -> PINS::Channels
            where
                PINS: Pins<$TIMX>,
            {
                apb.enr().modify(|_, w| w.$timXen().enabled());
                apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                if PINS::C1 {
                    tim.ccmr1_output
                        .modify(|_, w| unsafe { w.oc1pe().set_bit().oc1m().bits(PWM1) });
                }

                if PINS::C2 {
                    tim.ccmr1_output
                        .modify(|_, w| unsafe { w.oc2pe().set_bit().oc2m().bits(PWM1) });
                }

                if PINS::C3 {
                    tim.ccmr2_output
                        .modify(|_, w| unsafe { w.oc3pe().set_bit().oc3m().bits(PWM1) });
                }

                if PINS::C4 {
                    tim.ccmr2_output
                        .modify(|_, w| unsafe { w.oc4pe().set_bit().oc4m().bits(PWM1) });
                }

                let clk = clocks.pclk1().0 * if clocks.ppre1() == 1 { 1 } else { 2 };
                let freq = freq.0;
                let ticks = clk / freq;
                let psc = ticks / (1 << 16);
                tim.psc.write(|w| unsafe { w.psc().bits(u16(psc).unwrap()) });
                let arr = ticks / (psc + 1);
                tim.arr.write(|w| unsafe { w.bits(arr) });

                tim.cr1.write(|w| unsafe {
                    w.cms()
                        .bits(0b00)
                        .dir()
                        .bit(false)
                        .opm()
                        .bit(false)
                        .cen()
                        .bit(true)
                });

                unsafe { mem::uninitialized() }
            }

            impl hal::PwmPin for Pwm<$TIMX, C1> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 0) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 0) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr1.read().ccr1h().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arrh().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr1.write(|w| w.ccr1h().bits(duty)) }
                }
            }

            impl hal::PwmPin for Pwm<$TIMX, C2> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 4) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 4) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr2.read().ccr2h().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arrh().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr2.write(|w| w.ccr2h().bits(duty)) }
                }
            }

            impl hal::PwmPin for Pwm<$TIMX, C3> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 8) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 8) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr3.read().ccr3h().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arrh().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr3.write(|w| w.ccr3h().bits(duty)) }
                }
            }

            impl hal::PwmPin for Pwm<$TIMX, C4> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 12) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 12) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr4.read().ccr4h().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arrh().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr4.write(|w| w.ccr4h().bits(duty)) }
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, tim2en, tim2rst),
    TIM3: (tim3, tim3en, tim3rst),
    TIM4: (tim4, tim4en, tim4rst),
}
