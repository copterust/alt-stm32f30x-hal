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
#![allow(unused_unsafe)]
#![no_std]

#[cfg(feature = "stm32f301")]
pub use stm32f3::stm32f301 as pac;

#[cfg(feature = "stm32f302")]
pub use stm32f3::stm32f302 as pac;

#[cfg(feature = "stm32f303")]
pub use stm32f3::stm32f303 as pac;

#[cfg(feature = "stm32f373")]
pub use stm32f3::stm32f373 as pac;

#[cfg(feature = "stm32f334")]
pub use stm32f3::stm32f3x4 as pac;

#[cfg(feature = "stm32f328")]
pub use stm32f3::stm32f3x8 as pac;

#[cfg(feature = "device-selected")]
pub use crate::pac as device;

#[cfg(feature = "device-selected")]
pub use crate::pac as stm32;

#[cfg(feature = "device-selected")]
pub mod bb;
#[cfg(feature = "device-selected")]
pub mod delay;
#[cfg(feature = "device-selected")]
pub mod dma;
#[cfg(feature = "device-selected")]
pub mod exti;
#[cfg(feature = "device-selected")]
pub mod flash;
#[cfg(feature = "device-selected")]
pub mod gpio;
#[cfg(feature = "device-selected")]
pub mod i2c;
#[cfg(feature = "device-selected")]
pub mod prelude;
#[cfg(feature = "device-selected")]
pub mod pwm;
#[cfg(feature = "device-selected")]
pub mod rcc;
#[cfg(feature = "device-selected")]
pub mod serial;
#[cfg(feature = "device-selected")]
pub mod spi;
#[cfg(feature = "device-selected")]
pub mod syscfg;
#[cfg(feature = "device-selected")]
pub mod time;
#[cfg(feature = "device-selected")]
pub mod timer;
