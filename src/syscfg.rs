//! SYSCFG Clock Configuration

use crate::pac::{self, SYSCFG};
use crate::rcc;

/// Extension trait that constrains the `SYSCFG` peripheral
pub trait SyscfgExt {
    /// Constrains the `SYSCFG` peripheral so it plays nicely with the other
    /// abstractions
    fn constrain(self, apb2: &mut rcc::APB2) -> Syscfg;
}

impl SyscfgExt for SYSCFG {
    fn constrain(self, apb2: &mut rcc::APB2) -> Syscfg {
        apb2.enr().modify(|_, w| w.syscfgen().enabled());

        Syscfg { _0: () }
    }
}

/// Syscfg abstraction
pub struct Syscfg {
    _0: (),
}

impl Syscfg {
    pub(crate) fn exticr1(&mut self) -> &pac::syscfg::EXTICR1 {
        unsafe { &(*SYSCFG::ptr()).exticr1 }
    }

    pub(crate) fn exticr2(&mut self) -> &pac::syscfg::EXTICR2 {
        unsafe { &(*SYSCFG::ptr()).exticr2 }
    }

    pub(crate) fn exticr4(&mut self) -> &pac::syscfg::EXTICR4 {
        unsafe { &(*SYSCFG::ptr()).exticr4 }
    }
}
