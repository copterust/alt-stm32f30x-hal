//! Prelude

pub use crate::flash::FlashExt as _stm32f30x_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32f30x_hal_gpio_GpioExt;
pub use crate::pwm::PwmExt as _stm32f103xx_hal_pwm_PwmExt;
pub use crate::rcc::RccExt as _stm32f30x_hal_rcc_RccExt;
pub use crate::serial::SerialExt as _stm32f30x_hal_rcc_SerialExt;
pub use crate::time::U32Ext as _stm32f30x_hal_time_U32Ext;
pub use hal::prelude::*;
