//! HAL for the STM32F30x family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the STM32F30x
//! family of microcontrollers.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! # Requirements
//!
//! This crate requires `arm-none-eabi-gcc` to be installed and available in
//! `$PATH` to build.
//!
//! # Usage
//!
//! To build applications (binary crates) using this crate follow the
//! [cortex-m-quickstart] instructions and add this crate as a dependency in
//! step number 5 and make sure you enable the "rt" Cargo feature of this crate.
//!
//! [cortex-m-quickstart]: https://docs.rs/cortex-m-quickstart
//!
//! # Examples
//!
//! Examples of *using* these abstractions can be found in [`proving-ground`]
//! repo.
//!
//! [`proving-ground`]: https://github.com/copterust/proving-ground

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate bobbin_bits;
extern crate cast;
extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32f30x;
extern crate void;

pub mod bb;
pub mod delay;
pub mod dma;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod pwm;
pub mod rcc;
pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
