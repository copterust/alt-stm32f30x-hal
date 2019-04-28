//! SYSCFG Clock Configuration

use crate::rcc;
use stm32f30x::{self, SYSCFG};

/// Extension trait that constrains the `SYSCFG` peripheral
pub trait SyscfgExt {
    /// Constrains the `SYSCFG` peripheral so it plays nicely with the other
    /// abstractions
    fn constrain(self, apb2: &mut rcc::APB2) -> Syscfg;
}

impl SyscfgExt for SYSCFG {
    fn constrain(self, apb2: &mut rcc::APB2) -> Syscfg {
        apb2.enr().write(|w| w.syscfgen().enabled());
        Syscfg { _0: () }
    }
}

/// Syscfg abstraction
pub struct Syscfg {
    _0: (),
}

impl Syscfg {
    pub(crate) fn exticr1(&mut self) -> &stm32f30x::syscfg::EXTICR1 {
        unsafe { &(*SYSCFG::ptr()).exticr1 }
    }

    pub(crate) fn exticr2(&mut self) -> &stm32f30x::syscfg::EXTICR2 {
        unsafe { &(*SYSCFG::ptr()).exticr2 }
    }
}
