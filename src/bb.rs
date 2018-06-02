//! Bit banding

use core::ptr;

/// clear
pub fn clear<T>(register: *const T, bit: u8) {
    write(register, bit, false);
}

/// set
pub fn set<T>(register: *const T, bit: u8) {
    write(register, bit, true);
}

/// write
pub fn write<T>(register: *const T, bit: u8, set: bool) {
    let addr = register as usize;

    assert!(addr >= 0x4000_0000 && addr <= 0x4010_0000);
    assert!(bit < 32);

    let bit = bit as usize;
    let bb_addr = (0x4200_0000 + (addr - 0x4000_0000) * 32) + 4 * bit;
    unsafe { ptr::write_volatile(bb_addr as *mut u32, if set { 1 } else { 0 }) }
}
