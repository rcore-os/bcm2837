#[cfg(feature = "zero_kernel_offset")]
pub const KERNEL_OFFSET: usize = 0;
#[cfg(not(feature = "zero_kernel_offset"))]
pub const KERNEL_OFFSET: usize = 0xFFFF_0000_0000_0000;

pub const RAW_IO_BASE: usize = 0x3F00_0000;
pub const IO_BASE: usize = KERNEL_OFFSET + RAW_IO_BASE;

pub const CLOCK_FREQ: usize = 500000000; // PLLD
pub const CLOCK_DIVIDER: usize = 2;

pub const ARM_DMA_BASE: usize = IO_BASE + 0x7000;
pub const GPU_IO_BASE: usize = KERNEL_OFFSET + 0x7E000000;
pub const GPU_CACHED_BASE: usize = KERNEL_OFFSET + 0x40000000;
pub const GPU_UNCACHED_BASE: usize = KERNEL_OFFSET + 0xC0000000;
pub const GPU_MEM_BASE: usize = GPU_UNCACHED_BASE;

// PWM (ref: peripherals page 141)
pub const ARM_PWM_BASE: usize = (IO_BASE + 0x20C000);
pub const ARM_PWM_CTL: usize = (ARM_PWM_BASE + 0x00);
pub const ARM_PWM_STA: usize = (ARM_PWM_BASE + 0x04);
pub const ARM_PWM_DMAC: usize = (ARM_PWM_BASE + 0x08);
pub const ARM_PWM_RNG1: usize = (ARM_PWM_BASE + 0x10);
pub const ARM_PWM_DAT1: usize = (ARM_PWM_BASE + 0x14);
pub const ARM_PWM_FIF1: usize = (ARM_PWM_BASE + 0x18);
pub const ARM_PWM_RNG2: usize = (ARM_PWM_BASE + 0x20);
pub const ARM_PWM_DAT2: usize = (ARM_PWM_BASE + 0x24);

// emmc section
pub const RAW_EMMC_BASE: usize = 0x3F30_0000;
pub const EMMC_BASE: usize = KERNEL_OFFSET + RAW_EMMC_BASE;
