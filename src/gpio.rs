//! General Purpose Input / Output

// TODO the pins here currently correspond to the LQFP-100 package. There
// should be Cargo features that let you select different microcontroller
// packages

use bobbin_bits::*;
use core::intrinsics::transmute;
use core::marker::PhantomData;
use rcc::AHB;

/// Market trait for any pin
pub trait GPIOPin {}

/// Trait for pin mode
pub trait PinMode {
    /// Convert type state into actual bits
    fn pin_mode() -> U2;
}

/// Input
pub struct Input;
impl PinMode for Input {
    /// bits
    fn pin_mode() -> U2 {
        U2::B00
    }
}

/// Output
pub struct Output<OT: OutputType, OS: OutputSpeed> {
    _output_mode: PhantomData<OT>,
    _output_speed: PhantomData<OS>,
}
impl<OT: OutputType, OS: OutputSpeed> PinMode for Output<OT, OS> {
    fn pin_mode() -> U2 {
        U2::B01
    }
}

/// Alternating function
pub struct AltFn<AN: AltFnNum, OT: OutputType, OS: OutputSpeed> {
    _afnum: PhantomData<AN>,
    _output_mode: PhantomData<OT>,
    _output_speed: PhantomData<OS>,
}
impl<AN: AltFnNum, OT: OutputType, OS: OutputSpeed> PinMode
    for AltFn<AN, OT, OS>
{
    fn pin_mode() -> U2 {
        U2::B10
    }
}

/// Analog
pub struct Analog;
impl PinMode for Analog {
    fn pin_mode() -> U2 {
        U2::B11
    }
}

/// Pull (pin resistor state)
pub trait PullType {
    /// pull
    fn pull_type(&self) -> U2;
}

/// No pull; floating
pub struct PullNone;
impl PullType for PullNone {
    fn pull_type(&self) -> U2 {
        U2::B00
    }
}

/// Pull up
pub struct PullUp;
impl PullType for PullUp {
    fn pull_type(&self) -> U2 {
        U2::B01
    }
}

/// Pull down
pub struct PullDown;
impl PullType for PullDown {
    fn pull_type(&self) -> U2 {
        U2::B10
    }
}

/// Reserved
pub struct PullReserved;
impl PullType for PullReserved {
    fn pull_type(&self) -> U2 {
        U2::B11
    }
}

/// Configures output speed
pub trait OutputSpeed {
    /// Converts type state to actual bits
    fn output_speed(&self) -> U2;
}

/// Low speed
pub struct LowSpeed;
impl OutputSpeed for LowSpeed {
    fn output_speed(&self) -> U2 {
        U2::B00
    }
}

/// Medium  speed
pub struct MediumSpeed;
impl OutputSpeed for MediumSpeed {
    fn output_speed(&self) -> U2 {
        U2::B01
    }
}

/// Fast speed
pub struct FastSpeed;
impl OutputSpeed for FastSpeed {
    fn output_speed(&self) -> U2 {
        U2::B10
    }
}

/// High speed
pub struct HighSpeed;
impl OutputSpeed for HighSpeed {
    fn output_speed(&self) -> U2 {
        U2::B11
    }
}

/// Output type
pub trait OutputType {
    /// converts type state to actual type
    fn output_type(&self) -> U1;
}

/// Push pull
pub struct PushPull;
impl OutputType for PushPull {
    fn output_type(&self) -> U1 {
        U1::B0
    }
}

/// Open drain
pub struct OpenDrain;
impl OutputType for OpenDrain {
    fn output_type(&self) -> U1 {
        U1::B1
    }
}

/// AltFn number
pub trait AltFnNum {
    /// converts type state
    fn alt_fn_num(&self) -> U4;
}

/// AF0
pub struct AF0;
impl AltFnNum for AF0 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0000
    }
}

/// AF1
pub struct AF1;
impl AltFnNum for AF1 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0001
    }
}

/// AF2
pub struct AF2;
impl AltFnNum for AF2 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0010
    }
}

/// AF3
pub struct AF3;
impl AltFnNum for AF3 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0011
    }
}

/// AF4
pub struct AF4;
impl AltFnNum for AF4 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0100
    }
}

/// AF5
pub struct AF5;
impl AltFnNum for AF5 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0101
    }
}

/// AF6
pub struct AF6;
impl AltFnNum for AF6 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0110
    }
}

/// AF7
pub struct AF7;
impl AltFnNum for AF7 {
    fn alt_fn_num(&self) -> U4 {
        U4::B0111
    }
}

/// AF8
pub struct AF8;
impl AltFnNum for AF8 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1000
    }
}

/// AF9
pub struct AF9;
impl AltFnNum for AF9 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1001
    }
}

/// AF10
pub struct AF10;
impl AltFnNum for AF10 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1010
    }
}

/// AF11
pub struct AF11;
impl AltFnNum for AF11 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1011
    }
}

/// AF12
pub struct AF12;
impl AltFnNum for AF12 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1100
    }
}

/// AF13
pub struct AF13;
impl AltFnNum for AF13 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1101
    }
}

/// AF14
pub struct AF14;
impl AltFnNum for AF14 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1110
    }
}

/// AF15
pub struct AF15;
impl AltFnNum for AF15 {
    fn alt_fn_num(&self) -> U4 {
        U4::B1111
    }
}

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Ports;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB) -> Self::Ports;
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $iopxenr:ident, $iopxrst:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $AFR:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::marker::PhantomData;
            use hal::digital::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use stm32f30x::$GPIOX;

            use rcc::AHB;
            use super::*;

            /// GPIO parts
            pub struct Ports {
                $(
                    /// Pin $PXi
                    pub $pxi: $PXi<PullNone, Input>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Ports = Ports;

                fn split(self, ahb: &mut AHB) -> Ports {
                    ahb.enr().modify(|_, w| w.$iopxenr().enabled());
                    ahb.rstr().modify(|_, w| w.$iopxrst().set_bit());
                    ahb.rstr().modify(|_, w| w.$iopxrst().clear_bit());

                    Ports {
                        $(
                            $pxi: $PXi {
                                _pullup_state: PhantomData,
                                _pin_mode: PhantomData
                            },
                        )+
                    }
                }
            }

            fn set_pin_mode<PM: PinMode>(index: u32) {
                let offset = 2 * index;
                let mode_bits:u32 = PM::pin_mode().into();
                let moder = unsafe { &(*$GPIOX::ptr()).moder };
                // set io mode
                moder.modify(|r, w| unsafe {
                    w.bits((r.bits()
                            & !(0b11 << offset))
                           | (mode_bits << offset))
                });
            }

            fn set_output_speed<OS: OutputSpeed>(index: u32, os: OS) {
                let offset = 2 * index;
                let ospeedr = unsafe { &(*$GPIOX::ptr()).ospeedr };
                let speed_bits:u32 = os.output_speed().into();
                ospeedr.modify(|r, w| unsafe {
                    w.bits((r.bits()
                            & !(0b11 << offset))
                           | (speed_bits << offset))
                });
            }

            fn set_output_type<OT: OutputType>(index: u32, ot: OT) {
                let otyper = unsafe { &(*$GPIOX::ptr()).otyper };
                let type_bits:u32 = ot.output_type().into();
                otyper.modify(|r, w| unsafe {
                    w.bits(r.bits() | (type_bits << index))
                });
            }

            /// Partially erased pin
            pub struct $PXx<PT: PullType, PM: PinMode> {
                i: u8,
                _pullup_state: PhantomData<PT>,
                _pin_mode: PhantomData<PM>
            }

            impl<PT: PullType, OT: OutputType, OS: OutputSpeed>
                OutputPin for $PXx<PT, Output<OT, OS>> {
                    fn set_high(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
                    }

                    fn set_low(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) }
                    }
                }

            impl<PT: PullType,
                 AN: AltFnNum,
                 OT: OutputType,
                 OS: OutputSpeed> OutputPin for $PXx<PT, AltFn<AN, OT, OS>> {
                    fn set_high(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
                    }

                    fn set_low(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) }
                    }
                }

            impl<PT: PullType> InputPin for $PXx<PT, Input> {
                fn is_high(&self) -> bool {
                    !self.is_low()
                }

                fn is_low(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 }
                }
            }

            $(
                /// Pin
                pub struct $PXi<PT: PullType, PM: PinMode> {
                    _pullup_state: PhantomData<PT>,
                    _pin_mode: PhantomData<PM>
                }

                impl <PT: PullType, PM: PinMode> GPIOPin for  $PXi<PT, PM> {}

                impl<PT: PullType, PM: PinMode> $PXi<PT, PM> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<PT, PM> {
                        $PXx {
                            i: $i,
                            _pullup_state: PhantomData,
                            _pin_mode: PhantomData
                        }
                    }

                    /// Sets pull type: Floaing, PullUp, PullDown
                    pub fn pull_type<NPT: PullType>(self, pt: NPT)
                                                    -> $PXi<NPT, PM>
                    {
                        let shift = 0 + ($i << 1);
                        let pupdr = unsafe { &(*$GPIOX::ptr()).pupdr };
                        let pd_bits:u32 = pt.pull_type().into();
                        pupdr.modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0b11 << shift))
                                   | (pd_bits << shift))
                        });

                        unsafe { transmute(self) }
                    }

                    // XXX: it maybe makes sense to disallow this
                    //      when Pin is input already;
                    //      need to think about that
                    /// Sets io_mode to input
                    pub fn input(self) -> $PXi<PT, Input> {
                        set_pin_mode::<Input>($i);
                        unsafe { transmute(self) }
                    }

                    /// Sets io_mode to analog
                    pub fn analog(self) -> $PXi<PT, Analog> {
                        set_pin_mode::<Analog>($i);
                        unsafe { transmute(self) }
                    }

                    /// Set io_mode to output
                    pub fn output(self) -> $PXi<PT, Output<PushPull, LowSpeed>> {
                        let result: $PXi<PT, Output<PushPull, LowSpeed>> =
                            unsafe { transmute(self) };
                        // ensure output type and speed are set
                        let result2 = result
                            .output_type(PushPull)
                            .output_speed(LowSpeed);
                        set_pin_mode::<Output<PushPull, LowSpeed>>($i);
                        result2
                    }

                    /// Set io_mode to altfn and set alternating function
                    pub fn alternating<AFN: AltFnNum>(self, af: AFN) -> $PXi<PT, AltFn<AFN, PushPull, LowSpeed>> {
                        let result: $PXi<PT, AltFn<AFN, PushPull, LowSpeed>> =
                            unsafe { transmute(self) };
                        // ensure output type, speed, and afnum are set
                        let result2 = result
                            .alt_fn(af)
                            .output_type(PushPull)
                            .output_speed(LowSpeed);
                        set_pin_mode::<AltFn<AFN, PushPull, LowSpeed>>($i);
                        result2
                    }
                }

                impl<PT: PullType, OT: OutputType, OS: OutputSpeed> $PXi<PT, Output<OT, OS>> {
                    /// Set output type
                    pub fn output_type<NOT: OutputType>(self, ot: NOT) -> $PXi<PT, Output<NOT, OS>> {
                        set_output_type($i, ot);
                        unsafe { transmute(self) }
                    }

                    /// Set output speed
                    pub fn output_speed<NOS: OutputSpeed>(self, os: NOS) -> $PXi<PT, Output<OT, NOS>> {
                        set_output_speed($i, os);
                        unsafe { transmute(self) }
                    }
                }

                impl<PT: PullType, AFN: AltFnNum, OT: OutputType, OS: OutputSpeed> $PXi<PT, AltFn<AFN, OT, OS>> {
                    /// Set output type
                    pub fn output_type<NOT: OutputType>(self, ot: NOT) -> $PXi<PT, AltFn<AFN, NOT, OS>> {
                        set_output_type($i, ot);
                        unsafe { transmute(self) }
                    }

                    /// Set output speed
                    pub fn output_speed<NOS: OutputSpeed>(self, os: NOS) -> $PXi<PT, AltFn<AFN, OT, NOS>> {
                        set_output_speed($i, os);
                        unsafe { transmute(self) }
                    }

                    /// Set altfn
                    pub fn alt_fn<NAFN: AltFnNum>(self, af: NAFN) -> $PXi<PT, AltFn<NAFN, OT, OS>> {
                        let index = $i & (8 - 1);
                        let shift: usize = 0 + (index << 2);
                        let af_bits: u32 = af.alt_fn_num().into();
                        let afr = unsafe { &(*$GPIOX::ptr()).$AFR };
                        afr.modify(|r, w| unsafe {
                            w.bits((r.bits() & !(0b1111 << shift))
                                   | (af_bits << shift))
                        });

                        unsafe { transmute(self) }
                    }
                }

                impl<PT: PullType, OT:OutputType, OS:OutputSpeed> OutputPin
                    for $PXi<PT, Output<OT, OS>> {
                        fn set_high(&mut self) {
                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) }
                        }

                        fn set_low(&mut self) {
                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) }
                        }
                    }

                impl<PT: PullType, AN: AltFnNum, OT:OutputType, OS:OutputSpeed> OutputPin
                    for $PXi<PT, AltFn<AN, OT, OS>> {
                        fn set_high(&mut self) {
                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) }
                        }

                        fn set_low(&mut self) {
                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) }
                        }
                    }

                impl<PT: PullType, OT:OutputType, OS:OutputSpeed> StatefulOutputPin
                    for $PXi<PT, Output<OT, OS>> {
                        fn is_set_high(&self) -> bool {
                            !self.is_set_low()
                        }

                        fn is_set_low(&self) -> bool {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 }
                        }
                    }

                impl<PT: PullType, OT:OutputType, OS:OutputSpeed> toggleable::Default
                    for $PXi<PT, Output<OT, OS>> {}

                impl<PT: PullType> InputPin for $PXi<PT, Input> {
                    fn is_high(&self) -> bool {
                        !self.is_low()
                    }

                    fn is_low(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 }
                    }
                }

            )+

        }
    }
}

gpio!(GPIOA, gpioa, iopaen, ioparst, PAx, [
    PA0: (pa0, 0, afrl),
    PA1: (pa1, 1, afrl),
    PA2: (pa2, 2, afrl),
    PA3: (pa3, 3, afrl),
    PA4: (pa4, 4, afrl),
    PA5: (pa5, 5, afrl),
    PA6: (pa6, 6, afrl),
    PA7: (pa7, 7, afrl),
    PA8: (pa8, 8, afrh),
    PA9: (pa9, 9, afrh),
    PA10: (pa10, 10, afrh),
    PA11: (pa11, 11, afrh),
    PA12: (pa12, 12, afrh),
    PA13: (pa13, 13, afrh),
    PA14: (pa14, 14, afrh),
    PA15: (pa15, 15, afrh),
]);

gpio!(GPIOB, gpiob, iopben, iopbrst, PBx, [
    PB0: (pb0, 0, afrl),
    PB1: (pb1, 1, afrl),
    PB2: (pb2, 2, afrl),
    PB3: (pb3, 3, afrl),
    PB4: (pb4, 4, afrl),
    PB5: (pb5, 5, afrl),
    PB6: (pb6, 6, afrl),
    PB7: (pb7, 7, afrl),
    PB8: (pb8, 8, afrh),
    PB9: (pb9, 9, afrh),
    PB10: (pb10, 10, afrh),
    PB11: (pb11, 11, afrh),
    PB12: (pb12, 12, afrh),
    PB13: (pb13, 13, afrh),
    PB14: (pb14, 14, afrh),
    PB15: (pb15, 15, afrh),
]);

gpio!(GPIOC, gpioc, iopcen, iopcrst, PCx, [
    PC0: (pc0, 0, afrl),
    PC1: (pc1, 1, afrl),
    PC2: (pc2, 2, afrl),
    PC3: (pc3, 3, afrl),
    PC4: (pc4, 4, afrl),
    PC5: (pc5, 5, afrl),
    PC6: (pc6, 6, afrl),
    PC7: (pc7, 7, afrl),
    PC8: (pc8, 8, afrh),
    PC9: (pc9, 9, afrh),
    PC10: (pc10, 10, afrh),
    PC11: (pc11, 11, afrh),
    PC12: (pc12, 12, afrh),
    PC13: (pc13, 13, afrh),
    PC14: (pc14, 14, afrh),
    PC15: (pc15, 15, afrh),
]);

gpio!(GPIOD, gpiod, iopden, iopdrst, PDx, [
    PD0: (pd0, 0, afrl),
    PD1: (pd1, 1, afrl),
    PD2: (pd2, 2, afrl),
    PD3: (pd3, 3, afrl),
    PD4: (pd4, 4, afrl),
    PD5: (pd5, 5, afrl),
    PD6: (pd6, 6, afrl),
    PD7: (pd7, 7, afrl),
    PD8: (pd8, 8, afrh),
    PD9: (pd9, 9, afrh),
    PD10: (pd10, 10, afrh),
    PD11: (pd11, 11, afrh),
    PD12: (pd12, 12, afrh),
    PD13: (pd13, 13, afrh),
    PD14: (pd14, 14, afrh),
    PD15: (pd15, 15, afrh),
]);

gpio!(GPIOE, gpioe, iopeen, ioperst, PEx, [
    PE0: (pe0, 0, afrl),
    PE1: (pe1, 1, afrl),
    PE2: (pe2, 2, afrl),
    PE3: (pe3, 3, afrl),
    PE4: (pe4, 4, afrl),
    PE5: (pe5, 5, afrl),
    PE6: (pe6, 6, afrl),
    PE7: (pe7, 7, afrl),
    PE8: (pe8, 8, afrh),
    PE9: (pe9, 9, afrh),
    PE10: (pe10, 10, afrh),
    PE11: (pe11, 11, afrh),
    PE12: (pe12, 12, afrh),
    PE13: (pe13, 13, afrh),
    PE14: (pe14, 14, afrh),
    PE15: (pe15, 15, afrh),
]);

gpio!(GPIOF, gpiof, iopfen, iopfrst, PFx, [
    PF0: (pf0, 0, afrl),
    PF1: (pf1, 1, afrl),
    PF2: (pf2, 2, afrl),
    PF4: (pf3, 4, afrl),
    PF6: (pf6, 6, afrl),
    PF9: (pf9, 9, afrh),
    PF10: (pf10, 10, afrh),
]);
