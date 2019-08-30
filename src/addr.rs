pub const PHYSICAL_MEMORY_OFFSET: usize = 0xFFFF_0000_0000_0000;
#[cfg(feature = "bare_metal")]
pub const KERNEL_OFFSET: usize = 0;
#[cfg(not(feature = "bare_metal"))]
pub const KERNEL_OFFSET: usize = PHYSICAL_MEMORY_OFFSET;

pub const PHYSICAL_IO_START: usize = 0x3F00_0000;
pub const PHYSICAL_IO_END: usize = 0x4000_0000;
pub const IO_BASE: usize = KERNEL_OFFSET + PHYSICAL_IO_START;

/// Convert physical address to virtual address
#[inline]
pub const fn phys_to_virt(paddr: usize) -> usize {
    PHYSICAL_MEMORY_OFFSET + paddr
}

/// Convert virtual address to physical address
#[inline]
pub const fn virt_to_phys(vaddr: usize) -> usize {
    vaddr - PHYSICAL_MEMORY_OFFSET
}

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
