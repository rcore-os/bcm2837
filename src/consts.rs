#[cfg(feature = "zero_kernel_offset")]
pub const KERNEL_OFFSET: usize = 0;
#[cfg(not(feature = "zero_kernel_offset"))]
pub const KERNEL_OFFSET: usize = 0xFFFF_0000_0000_0000;

pub const RAW_IO_BASE: usize = 0x3F00_0000;
pub const IO_BASE: usize = KERNEL_OFFSET + RAW_IO_BASE;

pub const CLOCK_FREQ: usize = 500000000; // PLLD
pub const CLOCK_DIVIDER: usize = 2;

pub const ARM_DMA_BASE: usize = IO_BASE + 0x7000;
// pub const ARM_DMA_CHAN_BASE : usize = KERNEL_OFFSET + 0x7E007000;
pub const GPU_IO_BASE: usize = KERNEL_OFFSET + 0x7E000000;

pub const GPU_CACHED_BASE: usize = KERNEL_OFFSET + 0x40000000;
pub const GPU_UNCACHED_BASE: usize = KERNEL_OFFSET + 0xC0000000;
pub const GPU_MEM_BASE: usize = GPU_UNCACHED_BASE;

//
// PWM control register
//
pub const ARM_PWM_CTL_PWEN1: usize = (1 << 0);
pub const ARM_PWM_CTL_MODE1: usize = (1 << 1);
pub const ARM_PWM_CTL_RPTL1: usize = (1 << 2);
pub const ARM_PWM_CTL_SBIT1: usize = (1 << 3);
pub const ARM_PWM_CTL_POLA1: usize = (1 << 4);
pub const ARM_PWM_CTL_USEF1: usize = (1 << 5);
pub const ARM_PWM_CTL_CLRF1: usize = (1 << 6);
pub const ARM_PWM_CTL_MSEN1: usize = (1 << 7);
pub const ARM_PWM_CTL_PWEN2: usize = (1 << 8);
pub const ARM_PWM_CTL_MODE2: usize = (1 << 9);
pub const ARM_PWM_CTL_RPTL2: usize = (1 << 10);
pub const ARM_PWM_CTL_SBIT2: usize = (1 << 11);
pub const ARM_PWM_CTL_POLA2: usize = (1 << 12);
pub const ARM_PWM_CTL_USEF2: usize = (1 << 13);
pub const ARM_PWM_CTL_MSEN2: usize = (1 << 15);

//
// PWM status register
//
pub const ARM_PWM_STA_FULL1: usize = (1 << 0);
pub const ARM_PWM_STA_EMPT1: usize = (1 << 1);
pub const ARM_PWM_STA_WERR1: usize = (1 << 2);
pub const ARM_PWM_STA_RERR1: usize = (1 << 3);
pub const ARM_PWM_STA_GAPO1: usize = (1 << 4);
pub const ARM_PWM_STA_GAPO2: usize = (1 << 5);
pub const ARM_PWM_STA_GAPO3: usize = (1 << 6);
pub const ARM_PWM_STA_GAPO4: usize = (1 << 7);
pub const ARM_PWM_STA_BERR: usize = (1 << 8);
pub const ARM_PWM_STA_STA1: usize = (1 << 9);
pub const ARM_PWM_STA_STA2: usize = (1 << 10);
pub const ARM_PWM_STA_STA3: usize = (1 << 11);
pub const ARM_PWM_STA_STA4: usize = (1 << 12);

//
// PWM DMA configuration register
//
pub const ARM_PWM_DMAC_DREQ__SHIFT: usize = 0;
pub const ARM_PWM_DMAC_PANIC__SHIFT: usize = 8;
pub const ARM_PWM_DMAC_ENAB: usize = (1 << 31);

//
// DMA controller
//
pub const CS_RESET: usize = (1 << 31);
pub const CS_ABORT: usize = (1 << 30);
pub const CS_WAIT_FOR_OUTSTANDING_WRITES: usize = (1 << 28);
pub const CS_PANIC_PRIORITY_SHIFT: usize = 20;
pub const DEFAULT_PANIC_PRIORITY: usize = 15;
pub const CS_PRIORITY_SHIFT: usize = 16;
pub const DEFAULT_PRIORITY: usize = 1;
pub const CS_ERROR: usize = (1 << 8);
pub const CS_INT: usize = (1 << 2);
pub const CS_END: usize = (1 << 1);
pub const CS_ACTIVE: usize = (1 << 0);
pub const TI_PERMAP_SHIFT: usize = 16;
pub const TI_BURST_LENGTH_SHIFT: usize = 12;
pub const DEFAULT_BURST_LENGTH: usize = 0;
pub const TI_SRC_IGNORE: usize = (1 << 11);
pub const TI_SRC_DREQ: usize = (1 << 10);
pub const TI_SRC_WIDTH: usize = (1 << 9);
pub const TI_SRC_INC: usize = (1 << 8);
pub const TI_DEST_DREQ: usize = (1 << 6);
pub const TI_DEST_WIDTH: usize = (1 << 5);
pub const TI_DEST_INC: usize = (1 << 4);
pub const TI_WAIT_RESP: usize = (1 << 3);
pub const TI_TDMODE: usize = (1 << 1);
pub const TI_INTEN: usize = (1 << 0);
pub const TXFR_LEN_XLENGTH_SHIFT: usize = 0;
pub const TXFR_LEN_YLENGTH_SHIFT: usize = 16;
pub const TXFR_LEN_MAX: usize = 0x3FFFFFFF;
pub const TXFR_LEN_MAX_LITE: usize = 0xFFFF;
pub const STRIDE_SRC_SHIFT: usize = 0;
pub const STRIDE_DEST_SHIFT: usize = 16;
pub const DEBUG_LITE: usize = (1 << 28);
pub const ARM_DMA_INT_STATUS: usize = (ARM_DMA_BASE + 0xFE0);
pub const ARM_DMA_ENABLE: usize = (ARM_DMA_BASE + 0xFF0);

//
// Pulse Width Modulator
//
pub const ARM_PWM_BASE: usize = (IO_BASE + 0x20C000);
pub const ARM_PWM_CTL: usize = (ARM_PWM_BASE + 0x00);
pub const ARM_PWM_STA: usize = (ARM_PWM_BASE + 0x04);
pub const ARM_PWM_DMAC: usize = (ARM_PWM_BASE + 0x08);
pub const ARM_PWM_RNG1: usize = (ARM_PWM_BASE + 0x10);
pub const ARM_PWM_DAT1: usize = (ARM_PWM_BASE + 0x14);
pub const ARM_PWM_FIF1: usize = (ARM_PWM_BASE + 0x18);
pub const ARM_PWM_RNG2: usize = (ARM_PWM_BASE + 0x20);
pub const ARM_PWM_DAT2: usize = (ARM_PWM_BASE + 0x24);

// DMA channel resource management
pub const DMA_CHANNEL_MAX: usize = 12; // channels 0-12 are supported
pub const DMA_CHANNEL__MASK: usize = 0x0F; // explicit channel number 0-12
pub const DMA_CHANNEL_NONE: usize = 0x80; // returned if no channel available
pub const DMA_CHANNEL_NORMAL: usize = 0x81; // normal DMA engine requested
pub const DMA_CHANNEL_LITE: usize = 0x82; // lite (or normal) DMA engine requested

// GPIO CLOCK
pub const ARM_CM_BASE: usize = IO_BASE + 0x101000;
// pub const ARM_CM_BASE: u32 = 0x3F000000 + 0x101000;
// pub const ARM_CM_BASE: u32 = 0x7e101000;
pub const ARM_CM_GP0CTL: usize = ARM_CM_BASE + 0x70;
pub const ARM_CM_GP0DIV: usize = ARM_CM_BASE + 0x74;
pub const ARM_CM_PASSWD: u32 = (0x5A << 24);

// emmc section
pub const RAW_EMMC_BASE: usize = 0x3F30_0000;
pub const EMMC_BASE: usize = KERNEL_OFFSET + RAW_EMMC_BASE;
