//! Serial Peripheral Interface (SPI) bus

use core::ptr;

use hal::spi::{FullDuplex, Mode, Phase, Polarity};
use nb;
use stm32f30x::{RCC, SPI1, SPI2, SPI3};

use gpio::{AltFn, PullType, AF5, AF6};
use gpio::{HighSpeed, PinMode, PushPull};
use gpio::{PA5, PA6, PA7};
use gpio::{PB13, PB14, PB15, PB3, PB4, PB5};
use gpio::{PC10, PC11, PC12};
use rcc::Clocks;
use time::Hertz;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

/// SPI extension for SPI
pub trait SpiExt<SPI, ISCK, IMISO, IMOSI, SCK, MISO, MOSI> {
    /// Configures the SPI peripheral to operate in full duplex master mode.
    /// Consumes SPI peripheral and triple of (SCK, MISO, MOSI) pins.
    /// Returns [`Spi`].
    ///
    /// [`Spi`]: ./struct.Spi.html
    fn spi<F>(self,
              pins: (ISCK, IMISO, IMOSI),
              mode: Mode,
              freq: F,
              clocks: Clocks)
              -> Spi<SPI, (SCK, MISO, MOSI)>
        where F: Into<Hertz<u32>>;
}

#[cfg_attr(rustfmt, rustfmt_skip)]
macro_rules! spi {
    ($SPIX:ident,
     $apbenr:ident,
     $apbrstr:ident,
     $spiXen:ident,
     $spiXrst:ident,
     $pclkX:ident,
     $afn:ident,
     $speed:ident,
     sck: [$($sck: ident, )+],
     miso: $miso: tt,
     mosi: $mosi: tt
    ) => {
        spi!{
            $SPIX,
            $apbenr,
            $apbrstr,
            $spiXen,
            $spiXrst,
            $pclkX,
            $afn,
            $speed,
            [$(
                ($sck, $miso, ),
            )+],
            $mosi
        }
    };
    ($SPIX:ident,
     $apbenr:ident,
     $apbrstr:ident,
     $spiXen:ident,
     $spiXrst:ident,
     $pclkX:ident,
     $afn:ident,
     $speed:ident,
     [$(($sck: ident,
         [$($miso: ident, )+], ),
     )+],
     $mosi: tt
    ) => {
        spi!{
            $SPIX,
            $apbenr,
            $apbrstr,
            $spiXen,
            $spiXrst,
            $pclkX,
            $afn,
            $speed,
            [$(
                ($sck,
                 [$(
                     ($miso, $mosi),
                 )+]),
            )+]
        }
    };
    ($SPIX:ident,
     $apbenr:ident,
     $apbrstr:ident,
     $spiXen:ident,
     $spiXrst:ident,
     $pclkX:ident,
     $afn:ident,
     $speed:ident,
     [$(($sck: ident,
         [$(($miso: ident, [$($mosi: ident,)+]), )+]
     ), )+]
    ) => {
        $(
            $(
                $(
                    impl<PT: PullType, PM: PinMode>
                        SpiExt<$SPIX,
                    $sck<PT, PM>,
                    $miso<PT, PM>,
                    $mosi<PT, PM>,
                    $sck<PT, AltFn<$afn, PushPull, $speed>>,
                    $miso<PT, AltFn<$afn, PushPull, $speed>>,
                    $mosi<PT, AltFn<$afn, PushPull, $speed>>> for $SPIX
                    {
                        fn spi<F>(
                            self,
                            pins: ($sck<PT, PM>, $miso<PT, PM>, $mosi<PT, PM>),
                            mode: Mode,
                            freq: F,
                            clocks: Clocks)
                            -> Spi<$SPIX,
                        ($sck<PT, AltFn<$afn, PushPull, $speed>>,
                         $miso<PT, AltFn<$afn, PushPull, $speed>>,
                         $mosi<PT, AltFn<$afn, PushPull, $speed>>)>
                        where F: Into<Hertz<u32>>
                        {
                            let outpins = (pins.0.alternating($afn).output_speed($speed),
                                           pins.1.alternating($afn).output_speed($speed),
                                           pins.2.alternating($afn).output_speed($speed));
                            let apbenr = unsafe { &(*RCC::ptr()).$apbenr };
                            let apbrstr = unsafe { &(*RCC::ptr()).$apbrstr };
                            // enable or reset $SPIX
                            apbenr.modify(|_, w| w.$spiXen().enabled());
                            apbrstr.modify(|_, w| w.$spiXrst().set_bit());
                            apbrstr.modify(|_, w| w.$spiXrst().clear_bit());

                            // FRXTH: RXNE event is generated if the FIFO level is greater
                            // than or equal to        8-bit
                            // DS: 8-bit data size
                            // SSOE: Slave Select output disabled
                            self.cr2.write(|w| unsafe {
                                w.frxth()
                                    .set_bit()
                                    .ds()
                                    .bits(0b111)
                                    .ssoe()
                                    .clear_bit()
                            });

                            let br = match clocks.$pclkX().0 / freq.into().0 {
                                0 => unreachable!(),
                                1...2 => 0b000,
                                3...5 => 0b001,
                                6...11 => 0b010,
                                12...23 => 0b011,
                                24...39 => 0b100,
                                40...95 => 0b101,
                                96...191 => 0b110,
                                _ => 0b111,
                            };

                            // CPHA: phase
                            // CPOL: polarity
                            // MSTR: master mode
                            // BR: 1 MHz
                            // SPE: SPI disabled
                            // LSBFIRST: MSB first
                            // SSM: enable software slave management (NSS pin free for
                            // other uses) SSI: set nss high = master mode
                            // CRCEN: hardware CRC calculation disabled
                            // BIDIMODE: 2 line unidirectional (full duplex)
                            self.cr1.write(|w| unsafe {
                                w.cpha()
                                    .bit(mode.phase
                                         == Phase::CaptureOnSecondTransition)
                                    .cpol()
                                    .bit(mode.polarity == Polarity::IdleHigh)
                                    .mstr()
                                    .set_bit()
                                    .br()
                                    .bits(br)
                                    .spe()
                                    .set_bit()
                                    .lsbfirst()
                                    .clear_bit()
                                    .ssi()
                                    .set_bit()
                                    .ssm()
                                    .set_bit()
                                    .crcen()
                                    .clear_bit()
                                    .bidimode()
                                    .clear_bit()
                            });

                            Spi { spi: self,
                                  pins: outpins, }
                        }
                    }
                )+
            )+
        )+

        impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
            /// Releases the SPI peripheral and associated pins
            pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                (self.spi, self.pins)
            }
        }

        impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                let sr = self.spi.sr.read();

                Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.rxne().bit_is_set() {
                    // NOTE(read_volatile) read only 1 byte (the svd2rust API
                    // only allows reading a half-word)
                    return Ok(unsafe {
                        ptr::read_volatile(&self.spi.dr as *const _
                                           as *const u8)
                    });
                } else {
                    nb::Error::WouldBlock
                })
            }

            fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                let sr = self.spi.sr.read();

                Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    unsafe {
                        ptr::write_volatile(&self.spi.dr as *const _ as *mut u8,
                                            byte)
                    }
                    return Ok(());
                } else {
                    nb::Error::WouldBlock
                })
            }
        }

        impl<PINS> ::hal::blocking::spi::transfer::Default<u8>
            for Spi<$SPIX, PINS>
        {}

        impl<PINS> ::hal::blocking::spi::write::Default<u8>
            for Spi<$SPIX, PINS>
        {}
    };
}

spi!(SPI1,
     apb2enr,
     apb2rstr,
     spi1en,
     spi1rst,
     pclk2,
     AF5,
     HighSpeed,
     sck: [PA5, PB3,],
     miso: [PA6, PB4,],
     mosi: [PA7, PB5,]);

spi!(SPI2,
     apb1enr,
     apb1rstr,
     spi2en,
     spi2rst,
     pclk1,
     AF5,
     HighSpeed,
     sck: [PB13,],
     miso: [PB14,],
     mosi: [PB15,]);
spi!(SPI3,
     apb1enr,
     apb1rstr,
     spi3en,
     spi3rst,
     pclk1,
     AF6,
     HighSpeed,
     sck: [PB3, PC10,],
     miso: [PB4, PC11,],
     mosi: [PB5, PC12,]);
