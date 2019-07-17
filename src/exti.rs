//! External interrupts

use crate::gpio;
use crate::syscfg::Syscfg;

use crate::pac::{self, EXTI};

/// Extension trait that contsrins the `EXTI` peripheral
pub trait ExtiExt {
    /// Constrains the `EXTI` peripheral so it plays nicely with the other
    /// abstractions
    fn constrain(self) -> ExternalInterrupts;
}

/// External interrupts configuration
pub struct Exti<E: ExternalInterrupt> {
    ei: E,
}

/// Interrupt bound to pin
pub struct BoundInterrupt<GP, E> {
    pin: GP,
    ei: E,
}

impl<GP, E> BoundInterrupt<GP, E>
    where GP: gpio::GPIOPin,
          E: ExternalInterrupt
{
    /// Clears pending status on external interrupt
    pub fn unpend(&mut self) {
        let mask: bool = true;
        let exti = unsafe { &(*EXTI::ptr()) };
        let offset: u8 = self.ei.index();

        exti.pr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !((mask as u32) << offset))
                           | (((true & mask) as u32) << offset))
                });
    }

    /// Disconnect pin from external interrupt
    pub fn free(self, syscfg: &mut Syscfg) -> (GP, E) {
        let exti = unsafe { &(*EXTI::ptr()) };

        // Disable external interrupt on rise
        let offset = self.pin.index() as u32;
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b1 << offset)) });
        exti.emr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b1 << offset)) });
        exti.rtsr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b1 << offset)) });

        // Clear pin group
        let group_bits_to_clear = !bits_of_gpio_group(&self.pin.group());
        match self.ei.enumeration() {
            ExtIn::EXTI0 => {
                syscfg.exticr1().modify(|_, w| unsafe {
                                    w.exti0().bits(group_bits_to_clear)
                                });
            }
            ExtIn::EXTI1 => {
                syscfg.exticr1().modify(|_, w| unsafe {
                                    w.exti1().bits(group_bits_to_clear)
                                });
            }
            ExtIn::EXTI2 => {
                syscfg.exticr1().modify(|_, w| unsafe {
                                    w.exti2().bits(group_bits_to_clear)
                                });
            }
            ExtIn::EXTI3 => {
                syscfg.exticr1().modify(|_, w| unsafe {
                                    w.exti3().bits(group_bits_to_clear)
                                });
            }
            ExtIn::EXTI4 => {
                syscfg.exticr2().modify(|_, w| unsafe {
                                    w.exti4().bits(group_bits_to_clear)
                                });
            }
            ExtIn::EXTI13 => {
                syscfg.exticr4().modify(|_, w| unsafe {
                                    w.exti13().bits(group_bits_to_clear)
                                });
            }
        }

        (self.pin, self.ei)
    }
}

impl<E: ExternalInterrupt> Exti<E> {
    /// Bind interrupt to input pin. Returns bound interrput that can be used to
    /// unpend.
    pub fn bind<GP>(self, pin: GP, syscfg: &mut Syscfg) -> BoundInterrupt<GP, E>
        where GP: gpio::GPIOPin + hal::digital::v2::InputPin
    {
        let exti = unsafe { &(*EXTI::ptr()) };

        let group_bits = bits_of_gpio_group(&pin.group());

        // Setup pin group
        match self.ei.enumeration() {
            ExtIn::EXTI0 => {
                syscfg.exticr1()
                      .modify(|_, w| unsafe { w.exti0().bits(group_bits) });
            }
            ExtIn::EXTI1 => {
                syscfg.exticr1()
                      .modify(|_, w| unsafe { w.exti1().bits(group_bits) });
            }
            ExtIn::EXTI2 => {
                syscfg.exticr1()
                      .modify(|_, w| unsafe { w.exti2().bits(group_bits) });
            }
            ExtIn::EXTI3 => {
                syscfg.exticr1()
                      .modify(|_, w| unsafe { w.exti3().bits(group_bits) });
            }
            ExtIn::EXTI4 => {
                syscfg.exticr2()
                      .modify(|_, w| unsafe { w.exti4().bits(group_bits) });
            }
            ExtIn::EXTI13 => {
                syscfg.exticr4()
                      .modify(|_, w| unsafe { w.exti13().bits(group_bits) });
            }
        }

        let offset = pin.index() as u32;
        // Enable external interrupt on rise
        exti.imr1.modify(|r, w| unsafe {
                     w.bits((r.bits() & !(0b1 << offset)) | (1 << offset))
                 });
        exti.emr1.modify(|r, w| unsafe {
                     w.bits((r.bits() & !(0b1 << offset)) | (1 << offset))
                 });
        exti.rtsr1.modify(|r, w| unsafe {
                      w.bits((r.bits() & !(0b1 << offset)) | (1 << offset))
                  });

        BoundInterrupt { pin: pin,
                         ei: self.ei }
    }
}

#[doc(hidden)]
pub trait ExternalInterrupt: private::Sealed {
    #[doc(hidden)]
    fn interrupt(&self) -> pac::Interrupt;
    #[doc(hidden)]
    fn enumeration(&self) -> ExtIn;
    #[doc(hidden)]
    fn index(&self) -> u8;
}

// XXX: re do this, without enum
#[doc(hidden)]
pub enum ExtIn {
    /// 0,
    EXTI0,
    /// 1,
    EXTI1,
    /// 2,
    EXTI2,
    /// 3,
    EXTI3,
    /// 4,
    EXTI4,
    /// 13 (15_10),
    EXTI13,
}

macro_rules! gen_exti {
    ([$(($name:ident, $exti:ident, $i: expr),)+]) => {
        #[doc(hidden)]
        mod private {
            #[doc(hidden)]
            pub trait Sealed {}
            $(
                impl Sealed for super::$name {}
            )+
        }


        $(
            /// $name external interrupt
            pub struct $name {
                _0: (),
            }
            impl ExternalInterrupt for $name {
                fn interrupt(&self) -> pac::Interrupt {
                    pac::Interrupt::$exti
                }

                fn enumeration(&self) -> ExtIn {
                    ExtIn::$name
                }

                fn index(&self) -> u8 {
                    $i
                }
            }
        )+

        /// All external interrupts
        #[allow(non_snake_case)]
        pub struct ExternalInterrupts {
            $(
                /// $name interrupt
                pub $name: Exti<$name>,
            )+
        }

        impl ExtiExt for EXTI {
            fn constrain(self) -> ExternalInterrupts {
                (
                    ExternalInterrupts {
                        $(
                            $name: Exti { ei: $name { _0: () }} ,
                        )+
                    }
                )
            }
        }
    }
}

// TODO: fix this, EXTIx should be only connectable to PAx, PBx, etc.
gen_exti!([(EXTI0, EXTI0, 0),
           (EXTI1, EXTI1, 1),
           (EXTI2, EXTI2_TSC, 2),
           (EXTI3, EXTI3, 2),
           (EXTI4, EXTI4, 4),
           (EXTI13, EXTI15_10, 13),]);

fn bits_of_gpio_group(group: &gpio::Group) -> u8 {
    match group {
        gpio::Group::A => 0b0000,
        gpio::Group::B => 0b0001,
        gpio::Group::C => 0b0010,
        gpio::Group::D => 0b0011,
        gpio::Group::E => 0b0100,
        gpio::Group::F => 0b0101,
        gpio::Group::G => 0b0110,
    }
}
