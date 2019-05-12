//! Prelude

pub use crate::dma::DmaChannel as _stm32f30x_hal_dma_DmaChannel;
pub use crate::dma::DmaExt as _stm32f30x_hal_dma_DmaExt;
pub use crate::exti::ExtiExt as _stm32f30x_hal_exti_ExtiExt;
pub use crate::flash::FlashExt as _stm32f30x_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32f30x_hal_gpio_GpioExt;
pub use crate::i2c::I2cExt as _stm32f30x_hal_i2ci_I2cExt;
pub use crate::pwm::PwmExt as _stm32f103xx_hal_pwm_PwmExt;
pub use crate::rcc::RccExt as _stm32f30x_hal_rcc_RccExt;
pub use crate::serial::ReadDma as _stm32f30x_hal_serial_ReadDma;
pub use crate::serial::SerialExt as _stm32f30x_hal_serial_SerialExt;
pub use crate::serial::WriteDma as _stm32f30x_hal_serial_WriteDma;
pub use crate::spi::SpiExt as _stm32f30x_hal_spi_SpiExt;
pub use crate::syscfg::SyscfgExt as _stm32f30x_hal_syscfg_SyscfgExt;
pub use crate::time::U32BitrateExt as _stm32f30x_hal_time_U32Ext;

// instead of emb-hal::prelude::*;
pub use hal::blocking::delay::DelayMs as _embedded_hal_blocking_delay_DelayMs;
pub use hal::blocking::delay::DelayUs as _embedded_hal_blocking_delay_DelayUs;
pub use hal::blocking::i2c::{
    Read as _embedded_hal_blocking_i2c_Read,
    Write as _embedded_hal_blocking_i2c_Write,
    WriteRead as _embedded_hal_blocking_i2c_WriteRead,
};
pub use hal::blocking::serial::Write as _embedded_hal_blocking_serial_Write;
pub use hal::blocking::spi::{
    Transfer as _embedded_hal_blocking_spi_Transfer,
    Write as _embedded_hal_blocking_spi_Write,
};
pub use hal::digital::v2::InputPin as _embedded_hal_digital_InputPin;
pub use hal::digital::v2::OutputPin as _embedded_hal_digital_OutputPin;
pub use hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use hal::serial::Read as _embedded_hal_serial_Read;
pub use hal::serial::Write as _embedded_hal_serial_Write;
pub use hal::spi::FullDuplex as _embedded_hal_spi_FullDuplex;
pub use hal::timer::CountDown as _embedded_hal_timer_CountDown;
pub use hal::Pwm as _embedded_hal_Pwm;
pub use hal::PwmPin as _embedded_hal_PwmPin;
