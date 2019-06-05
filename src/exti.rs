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

impl<E: ExternalInterrupt> Exti<E> {
    /// Bind interrupt to input pin
    pub fn bind<GP>(&mut self, pin: GP, syscfg: &mut Syscfg)
        where GP: gpio::GPIOPin + hal::digital::v2::InputPin
    {
        let exti = unsafe { &(*EXTI::ptr()) };

        let group_bits = bits_of_gpio_group(&pin.group());

        // XXX: modify, instead of write
        // Setup pin group
        match self.ei.enumeration() {
            ExtIn::EXTI0 => {
                syscfg.exticr1().modify(|r, w| unsafe {
                                    w.bits(r.bits()
                                           | ((r.exti0().bits() | group_bits)
                                              as u32))
                                });
            }
            ExtIn::EXTI1 => {
                syscfg.exticr1().modify(|r, w| unsafe {
                                    w.bits(r.bits()
                                           | ((r.exti1().bits() | group_bits)
                                              as u32))
                                });
            }
            ExtIn::EXTI2 => {
                syscfg.exticr1().modify(|r, w| unsafe {
                                    w.bits(r.bits()
                                           | ((r.exti2().bits() | group_bits)
                                              as u32))
                                });
            }
            ExtIn::EXTI3 => {
                syscfg.exticr1().modify(|r, w| unsafe {
                                    w.bits(r.bits()
                                           | ((r.exti3().bits() | group_bits)
                                              as u32))
                                });
            }
            ExtIn::EXTI4 => {
                syscfg.exticr2().modify(|r, w| unsafe {
                                    w.bits(r.bits()
                                           | ((r.exti4().bits() | group_bits)
                                              as u32))
                                });
            }
            ExtIn::EXTI13 => {
                syscfg.exticr4().modify(|r, w| unsafe {
                                    w.bits(r.bits()
                                           | ((r.exti13().bits() | group_bits)
                                              as u32))
                                });
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
    }

    /// Clears pending status on external interrupt
    pub fn unpend(&mut self) {
        let mask: u32 = 15;
        let offset: u32 = self.ei.index() as u32;
        let exti = unsafe { &(*EXTI::ptr()) };
        exti.pr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(mask << offset)) | (1 << offset))
                });
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
