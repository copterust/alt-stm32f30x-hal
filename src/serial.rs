//! Serial

use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use hal::serial;
use nb;
use stm32f30x::{Interrupt, RCC, USART1, USART2, USART3};
use void::Void;

// use crate::afio::MAPR;
use crate::dma::{dma1, CircBuffer, Static, Transfer, R, W};
use gpio::{AltFn, HighSpeed, PinMode, PullType, PushPull, AF7};
use gpio::{PA10, PA14, PA15, PA2, PA3, PA9};
use gpio::{PB10, PB11, PB3, PB4, PB6, PB7};
use gpio::{PC10, PC11, PC4, PC5};
use gpio::{PD5, PD6, PD8, PD9};
use gpio::{PE0, PE1, PE15};
use rcc::Clocks;
use time::Bps;

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
}

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial extension for USART
pub trait SerialExt<USART, ITX, IRX, TX, RX> {
    /// Configures USART and consumes pair of (tx, rx) pins
    /// to act as serial port.
    /// Configures pins accordingly.
    /// Returns [`Serial`].
    ///
    /// [`Serial`]: ./struct.Serial.html
    fn serial(self,
              pins: (ITX, IRX),
              baud_rate: Bps<u32>,
              clocks: Clocks)
              -> Serial<USART, (TX, RX)>;
}

macro_rules! serial {
    ($USARTX:ident,
     $INTNAME:ident,
     $apbenr:ident,
     $apbrstr:ident,
     $usartXen:ident,
     $usartXrst:ident,
     $pclkX:ident,
     $afn:ident,
     $speed:ident,
     [$($txpin: ident, )+],
     $restrx: tt
    ) => {
        serial!{$USARTX,
                $INTNAME,
                $apbenr,
                $apbrstr,
                $usartXen,
                $usartXrst,
                $pclkX,
                $afn,
                $speed,
                [$(
                    ($txpin, $restrx),
                )+]}
    };
    ($USARTX:ident,
     $INTNAME:ident,
     $apbenr:ident,
     $apbrstr:ident,
     $usartXen:ident,
     $usartXrst:ident,
     $pclkX:ident,
     $afn:ident,
     $speed:ident,
     [$(($txpin: ident, [$($rxpin: ident,)+]), )+]
    ) => {
        $(
            $(
                impl <PT: PullType,
                PM: PinMode>
                    SerialExt<$USARTX,
                $txpin<PT, PM>,
                $rxpin<PT, PM>,
                $txpin<PT, AltFn<$afn, PushPull, $speed>>,
                $rxpin<PT, AltFn<$afn, PushPull, $speed>>>
                    for $USARTX {
                        fn serial(self,
                                  pins: ($txpin<PT, PM>, $rxpin<PT, PM>),
                                  baud_rate: Bps<u32>,
                                  clocks: Clocks)
                                  -> Serial<$USARTX, ($txpin<PT, AltFn<$afn, PushPull, $speed>>,
                                                      $rxpin<PT, AltFn<$afn, PushPull, $speed>>)>
                        {
                            let outpins = (
                                pins.0
                                    .alternating($afn)
                                    .output_speed($speed),
                                pins.1
                                    .alternating($afn)
                                    .output_speed($speed),
                            );

                            // enable or reset $USARTX
                            let apbenr = unsafe { &(*RCC::ptr()).$apbenr };
                            let apbrstr = unsafe { &(*RCC::ptr()).$apbrstr };
                            apbenr.modify(|_, w| w.$usartXen().enabled());
                            apbrstr.modify(|_, w| w.$usartXrst().set_bit());
                            apbrstr.modify(|_, w| w.$usartXrst().clear_bit());
                            self.cr3.write(|w| w.dmat().set_bit().dmar().set_bit());

                            let brr = clocks.$pclkX().0 / baud_rate.0;
                            assert!(brr >= 16, "impossible baud rate");
                            self.brr.write(|w| unsafe { w.bits(brr) });

                            // UE: enable USART
                            // RE: enable receiver
                            // TE: enable transceiver
                            self.cr1.write(|w| {
                                w.ue()
                                    .set_bit()
                                    .re()
                                    .set_bit()
                                    .te()
                                    .set_bit()
                            });

                            Serial { usart: self,
                                     pins: outpins, }
                        }
                    }
            )+
        )+

        impl<TX, RX> Serial<$USARTX, (TX, RX)> {
            /// Returns associated interrupt
            pub fn get_interrupt(&self) -> Interrupt {
                Interrupt::$INTNAME
            }

            /// Starts listening for an interrupt event
            pub fn listen(&mut self, event: Event) {
                match event {
                    Event::Rxne => {
                        self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                    }
                    Event::Txe => {
                        self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                    }
                }
            }

            /// Starts listening for an interrupt event
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    Event::Rxne => {
                        self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                    }
                    Event::Txe => {
                        self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                    }
                }
            }

            /// Splits the `Serial` abstraction into a transmitter and a
            /// receiver half
            pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                (Tx { _usart: PhantomData, }, Rx { _usart: PhantomData, })
            }

            /// Releases the USART peripheral and associated pins
            pub fn free(self) -> ($USARTX, (TX, RX)) {
                (self.usart, self.pins)
            }
        }

        impl Rx<$USARTX> {
            /// clear overrun
            pub fn clear_overrun_error(&mut self) -> u8 {
                unsafe { (*$USARTX::ptr()).icr.write(|w| w.orecf().set_bit()) };
                let rdr = unsafe { (*$USARTX::ptr()).rdr.read() };
                (rdr.bits() & 0xFF) as u8
            }

            /// clear framing error
            pub fn clear_framing_error(&mut self) -> u8 {
                unsafe { (*$USARTX::ptr()).icr.write(|w| w.fecf().set_bit()) };
                let rdr = unsafe { (*$USARTX::ptr()).rdr.read() };
                (rdr.bits() & 0xFF) as u8
            }

            /// clear noise error
            pub fn clear_noise_error(&mut self) -> u8 {
                unsafe { (*$USARTX::ptr()).icr.write(|w| w.ncf().set_bit()) };
                let rdr = unsafe { (*$USARTX::ptr()).rdr.read() };
                (rdr.bits() & 0xFF) as u8
            }
        }

        impl serial::Read<u8> for Rx<$USARTX> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                // NOTE(unsafe) atomic read with no side effects
                let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                Err(if isr.pe().bit_is_set() {
                    nb::Error::Other(Error::Parity)
                } else if isr.fe().bit_is_set() {
                    self.clear_framing_error();
                    nb::Error::Other(Error::Framing)
                } else if isr.nf().bit_is_set() {
                    self.clear_noise_error();
                    nb::Error::Other(Error::Noise)
                } else if isr.ore().bit_is_set() {
                    self.clear_overrun_error();
                    nb::Error::Other(Error::Overrun)
                } else if isr.rxne().bit_is_set() {
                    // NOTE(read_volatile) see `write_volatile` below
                    return Ok(unsafe {
                        ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _
                                           as *const _)
                    });
                } else {
                    nb::Error::WouldBlock
                })
            }
        }

        impl serial::Write<u8> for Tx<$USARTX> {
            // NOTE(Void) See section "29.7 USART interrupts"; the only
            // possible errors during transmission are: clear to send
            // (which is disabled in this case)
            // errors and framing errors (which only occur in SmartCard
            // mode); neither of these apply to our hardware configuration
            type Error = Void;

            fn flush(&mut self) -> nb::Result<(), Void> {
                // NOTE(unsafe) atomic read with no side effects
                let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                if isr.tc().bit_is_set() {
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Void> {
                // NOTE(unsafe) atomic read with no side effects
                let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                if isr.txe().bit_is_set() {
                    // NOTE(unsafe) atomic write to stateless register
                    // NOTE(write_volatile) 8-bit write that's not possible
                    // through the svd2rust API
                    unsafe {
                        ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _
                                            as *mut _,
                                            byte)
                    }
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

            impl<B> ReadDma<B> for Rx<$USARTX> where B: AsMut<[u8]> {
                fn circ_read(self, mut chan: Self::Dma, buffer: &'static mut [B; 2],
                ) -> CircBuffer<B, Self::Dma>
                {
                    {
                        let buffer = buffer[0].as_mut();
                        chan.cmar().write(|w| unsafe {
                            w.ma().bits(buffer.as_ptr() as usize as u32)
                        });
                        chan.cndtr().write(|w| unsafe{
                            w.ndt().bits((buffer.len() * 2) as u16)
                        });
                        chan.cpar().write(|w| unsafe {
                            w.pa().bits(&(*$USARTX::ptr()).rdr as *const _ as usize as u32)
                        });

                        // TODO can we weaken this compiler barrier?
                        // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                        // the next statement, which starts the DMA transfer
                        atomic::compiler_fence(Ordering::SeqCst);

                        unsafe {
                            chan.ccr().modify(|_, w| {
                                w.mem2mem()
                                    .clear_bit()
                                    .pl()
                                    .bits(0b01)
                                    .msize()
                                    .bits(0b0) // 8 bit
                                    .psize()
                                    .bits(0b0)
                                    .minc()
                                    .set_bit()
                                    .pinc()
                                    .clear_bit()
                                    .circ()
                                    .set_bit()
                                    .dir()
                                    .clear_bit()
                            });
                            chan.ccr().modify(|_, w| w.en().set_bit() )
                        }
                    }

                    CircBuffer::new(buffer, chan)
                }

                fn read_exact(self, mut chan: Self::Dma, buffer: &'static mut B,
                ) -> Transfer<W, &'static mut B, Self::Dma, Self>
                {
                    {
                        let buffer = buffer.as_mut();
                        chan.cmar().write(|w| unsafe {
                            w.ma().bits(buffer.as_ptr() as usize as u32)
                        });
                        chan.cndtr().write(|w| unsafe{
                            w.ndt().bits(buffer.len() as u16)
                        });
                        chan.cpar().write(|w| unsafe {
                            w.pa().bits(&(*$USARTX::ptr()).rdr as *const _ as usize as u32)
                        });

                        // TODO can we weaken this compiler barrier?
                        // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                        // the next statement, which starts the DMA transfer
                        atomic::compiler_fence(Ordering::SeqCst);

                        unsafe {
                            chan.ccr().modify(|_, w| {
                                w.mem2mem()
                                    .clear_bit()
                                    .pl()
                                    .bits(0b01)
                                    .msize()
                                    .bits(0b0)
                                    .psize()
                                    .bits(0b0)
                                    .minc()
                                    .set_bit()
                                    .pinc()
                                    .clear_bit()
                                    .circ()
                                    .clear_bit()
                                    .dir()
                                    .clear_bit()
                                    .en()
                                    .set_bit()
                            });
                        }
                    }

                    Transfer::w(buffer, chan, self)
                }
            }

            impl<A, B> WriteDma<A, B> for Tx<$USARTX> where A: AsRef<[u8]>, B: Static<A> {
                fn write_all(self, mut chan: Self::Dma, buffer: B
                ) -> Transfer<R, B, Self::Dma, Self>
                {
                    {
                        let buffer = buffer.borrow().as_ref();
                        chan.cpar().write(|w| unsafe {
                            w.pa().bits(&(*$USARTX::ptr()).tdr as *const _ as usize as u32)
                        });

                        chan.cmar().write(|w| unsafe {
                            w.ma().bits(buffer.as_ptr() as usize as u32)
                        });

                        chan.cndtr().write(|w| unsafe{
                            w.ndt().bits(buffer.len() as u16)
                        });

                        // TODO can we weaken this compiler barrier?
                        // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                        // the next statement, which starts the DMA transfer
                        atomic::compiler_fence(Ordering::SeqCst);

                        unsafe {
                            chan.ccr().modify(|_, w| {
                                w.mem2mem()
                                    .clear_bit()
                                    .pl()
                                    .bits(0b01)
                                    .msize()
                                    .bits(0b0)
                                    .psize()
                                    .bits(0b0)
                                    .minc()
                                    .set_bit()
                                    .pinc()
                                    .clear_bit()
                                    .circ()
                                    .clear_bit()
                                    .dir()
                                    .set_bit()
                                    .en()
                                    .set_bit()
                            });
                        }
                    }

                    Transfer::r(buffer, chan, self)
                }
            }
    };
}

use crate::dma::DmaChannel;

impl DmaChannel for Rx<USART1> {
    type Dma = dma1::C5;
}

impl DmaChannel for Tx<USART1> {
    type Dma = dma1::C4;
}

impl DmaChannel for Rx<USART2> {
    type Dma = dma1::C6;
}

impl DmaChannel for Tx<USART2> {
    type Dma = dma1::C7;
}

impl DmaChannel for Rx<USART3> {
    type Dma = dma1::C3;
}

impl DmaChannel for Tx<USART3> {
    type Dma = dma1::C2;
}

/// ReadDma
pub trait ReadDma<B>: DmaChannel
    where B: AsMut<[u8]>,
          Self: core::marker::Sized
{
    /// circ
    fn circ_read(self,
                 chan: Self::Dma,
                 buffer: &'static mut [B; 2])
                 -> CircBuffer<B, Self::Dma>;
    /// exact
    fn read_exact(self,
                  chan: Self::Dma,
                  buffer: &'static mut B)
                  -> Transfer<W, &'static mut B, Self::Dma, Self>;
}

/// WriteDma
pub trait WriteDma<A, B>: DmaChannel
    where A: AsRef<[u8]>,
          B: Static<A>,
          Self: core::marker::Sized
{
    /// Write all
    fn write_all(self,
                 chan: Self::Dma,
                 buffer: B)
                 -> Transfer<R, B, Self::Dma, Self>;
}

// XXX: we can't use GATs yet, so had to retort to macros
serial!(USART1,
        USART1_EXTI25,
        apb2enr,
        apb2rstr,
        usart1en,
        usart1rst,
        pclk2,
        AF7,
        HighSpeed, // XXX: not sure, maybe we should allow setting this
        [PA9, PB6, PC4, PE0,],
        [PA10, PB7, PC5, PE1,]);
serial!(USART2,
        USART2_EXTI26,
        apb1enr,
        apb1rstr,
        usart2en,
        usart2rst,
        pclk1,
        AF7,
        HighSpeed,
        [PA2, PA14, PB3, PD5,],
        [PA3, PA15, PB4, PD6,]);
serial!(USART3,
        USART3_EXTI28,
        apb1enr,
        apb1rstr,
        usart3en,
        usart3rst,
        pclk1,
        AF7,
        HighSpeed,
        [PB10, PC10, PD8,],
        [PB11, PC11, PD9, PE15,]);
