#[cfg(feature = "lower_va_range")]
pub const VA_BASE: usize = 0;
#[cfg(not(feature = "lower_va_range"))]
pub const VA_BASE: usize = 0xFFFF_0000_0000_0000;

pub const PERIPHERALS_START: usize = 0x3F00_0000;
pub const PERIPHERALS_END: usize = 0x4000_1000;
pub const CONTROL_START: usize = 0x4000_0000;
pub const IO_BASE: usize = VA_BASE + PERIPHERALS_START;
pub const CONTROL_BASE: usize = VA_BASE + CONTROL_START;

/// Convert physical address to bus address (ref: peripherals page 6)
#[inline]
pub const fn phys_to_bus(paddr: u32) -> u32 {
    paddr | 0xC0000000
}

/// Convert physical address to bus address (ref: peripherals page 6)
#[inline]
pub const fn bus_to_phys(baddr: u32) -> u32 {
    baddr & !0xC0000000
}

/// Convert I/O peripherals address to bus address (ref: peripherals page 6)
#[inline]
pub const fn io_to_bus(paddr: u32) -> u32 {
    (paddr & 0xFFFFFF) | 0x7E000000
}

/// Convert bus address to I/O peripherals address (ref: peripherals page 6)
#[inline]
pub const fn bus_to_io(baddr: u32) -> u32 {
    (baddr & 0xFFFFFF) | 0x3F000000
}
