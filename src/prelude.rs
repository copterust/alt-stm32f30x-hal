//! Prelude

pub use crate::dma::DmaChannel as _stm32f30x_hal_dma_DmaChannel;
pub use crate::dma::DmaExt as _stm32f30x_hal_dma_DmaExt;
pub use crate::flash::FlashExt as _stm32f30x_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32f30x_hal_gpio_GpioExt;
pub use crate::i2c::I2cExt as _stm32f30x_hal_i2ci_I2cExt;
pub use crate::pwm::PwmExt as _stm32f103xx_hal_pwm_PwmExt;
pub use crate::rcc::RccExt as _stm32f30x_hal_rcc_RccExt;
pub use crate::serial::ReadDma as _stm32f30x_hal_serial_ReadDma;
pub use crate::serial::SerialExt as _stm32f30x_hal_serial_SerialExt;
pub use crate::serial::WriteDma as _stm32f30x_hal_serial_WriteDma;
pub use crate::spi::SpiExt as _stm32f30x_hal_spi_SpiExt;
pub use crate::time::U32BitrateExt as _stm32f30x_hal_time_U32Ext;
pub use hal::prelude::*;
