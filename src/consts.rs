#[cfg(feature = "zero_kernel_offset")]
pub const KERNEL_OFFSET: usize = 0;
#[cfg(not(feature = "zero_kernel_offset"))]
pub const KERNEL_OFFSET: usize = 0xFFFF_0000_0000_0000;

pub const RAW_IO_BASE: usize = 0x3F00_0000;
pub const IO_BASE: usize = KERNEL_OFFSET + RAW_IO_BASE;


pub const CLOCK_FREQ	: u32	= 500000000;		// PLLD
pub const CLOCK_DIVIDER	: u32	= 2;

pub const ARM_IO_BASE	: u32 =	0x3F000000;
pub const ARM_DMA_BASE : u32 =  (ARM_IO_BASE + 0x7000);
pub const GPU_IO_BASE	: u32 = 	0x7E000000;

pub const GPU_CACHED_BASE	: u32 = 	0x40000000;
pub const GPU_UNCACHED_BASE	: u32 = 0xC0000000;
pub const GPU_MEM_BASE : u32 = GPU_UNCACHED_BASE;

//
// PWM control register
//
pub const ARM_PWM_CTL_PWEN1: u32	= (1 << 0);
pub const ARM_PWM_CTL_MODE1: u32	= (1 << 1);
pub const ARM_PWM_CTL_RPTL1: u32	= (1 << 2);
pub const ARM_PWM_CTL_SBIT1: u32	= (1 << 3);
pub const ARM_PWM_CTL_POLA1: u32	= (1 << 4);
pub const ARM_PWM_CTL_USEF1: u32	= (1 << 5);
pub const ARM_PWM_CTL_CLRF1: u32	= (1 << 6);
pub const ARM_PWM_CTL_MSEN1: u32	= (1 << 7);
pub const ARM_PWM_CTL_PWEN2: u32	= (1 << 8);
pub const ARM_PWM_CTL_MODE2: u32	= (1 << 9);
pub const ARM_PWM_CTL_RPTL2: u32	= (1 << 10);
pub const ARM_PWM_CTL_SBIT2: u32	= (1 << 11);
pub const ARM_PWM_CTL_POLA2: u32	= (1 << 12);
pub const ARM_PWM_CTL_USEF2: u32	= (1 << 13);
pub const ARM_PWM_CTL_MSEN2: u32	= (1 << 14);

//
// PWM status register
//
pub const ARM_PWM_STA_FULL1	: u32 = (1 << 0);
pub const ARM_PWM_STA_EMPT1	: u32 = (1 << 1);
pub const ARM_PWM_STA_WERR1	: u32 = (1 << 2);
pub const ARM_PWM_STA_RERR1	: u32 = (1 << 3);
pub const ARM_PWM_STA_GAPO1	: u32 = (1 << 4);
pub const ARM_PWM_STA_GAPO2	: u32 = (1 << 5);
pub const ARM_PWM_STA_GAPO3	: u32 = (1 << 6);
pub const ARM_PWM_STA_GAPO4	: u32 = (1 << 7);
pub const ARM_PWM_STA_BERR	: u32 = (1 << 8);
pub const ARM_PWM_STA_STA1	: u32 = (1 << 9);
pub const ARM_PWM_STA_STA2	: u32 = (1 << 10);
pub const ARM_PWM_STA_STA3	: u32 = (1 << 11);
pub const ARM_PWM_STA_STA4	: u32 = (1 << 12);

//
// PWM DMA configuration register
//
pub const ARM_PWM_DMAC_DREQ__SHIFT : u32 = 0;
pub const ARM_PWM_DMAC_PANIC__SHIFT : u32 = 8;
pub const ARM_PWM_DMAC_ENAB : u32 = (1 << 31);

//
// DMA controller
//
pub const CS_RESET	: u32 = 		(1 << 31);
pub const CS_ABORT		: u32 = 	(1 << 30);
pub const CS_WAIT_FOR_OUTSTANDING_WRITES: u32 = 	(1 << 28);
pub const CS_PANIC_PRIORITY_SHIFT	: u32 = 	20;
pub const DEFAULT_PANIC_PRIORITY	: u32 = 	15;
pub const CS_PRIORITY_SHIFT		: u32 = 16;
pub const DEFAULT_PRIORITY	: u32 = 	1;
pub const CS_ERROR		: u32 = 	(1 << 8);
pub const CS_INT			: u32 = 	(1 << 2);
pub const CS_END		: u32 = 		(1 << 1);
pub const CS_ACTIVE		: u32 = 	(1 << 0);
pub const TI_PERMAP_SHIFT	: u32 = 		16;
pub const TI_BURST_LENGTH_SHIFT	: u32 = 	12;
pub const DEFAULT_BURST_LENGTH	: u32 = 	0;
pub const TI_SRC_IGNORE		: u32 = 	(1 << 11);
pub const TI_SRC_DREQ		: u32 = 	(1 << 10);
pub const TI_SRC_WIDTH		: u32 = 	(1 << 9);
pub const TI_SRC_INC		: u32 = 	(1 << 8);
pub const TI_DEST_DREQ		: u32 = 	(1 << 6);
pub const TI_DEST_WIDTH		: u32 = 	(1 << 5);
pub const TI_DEST_INC		: u32 = 	(1 << 4);
pub const TI_WAIT_RESP		: u32 = 	(1 << 3);
pub const TI_TDMODE		: u32 = 	(1 << 1);
pub const TI_INTEN		: u32 = 	(1 << 0);
pub const TXFR_LEN_XLENGTH_SHIFT	: u32 = 	0;
pub const TXFR_LEN_YLENGTH_SHIFT	: u32 = 	16;
pub const TXFR_LEN_MAX		: u32 = 	0x3FFFFFFF;
pub const TXFR_LEN_MAX_LITE	: u32 = 	0xFFFF;
pub const STRIDE_SRC_SHIFT	: u32 = 	0;
pub const STRIDE_DEST_SHIFT	: u32 = 	16;
pub const DEBUG_LITE		: u32 = 	(1 << 28);
pub const ARM_DMA_INT_STATUS	: u32 = 	(ARM_DMA_BASE + 0xFE0);
pub const ARM_DMA_ENABLE		: u32 = 	(ARM_DMA_BASE + 0xFF0);

//
// Pulse Width Modulator
//
pub const  ARM_PWM_BASE		: u32 = (ARM_IO_BASE + 0x20C000);
pub const  ARM_PWM_CTL		: u32 = (ARM_PWM_BASE + 0x00);
pub const  ARM_PWM_STA		: u32 = (ARM_PWM_BASE + 0x04);
pub const  ARM_PWM_DMAC		: u32 = (ARM_PWM_BASE + 0x08);
pub const  ARM_PWM_RNG1		: u32 = (ARM_PWM_BASE + 0x10);
pub const  ARM_PWM_DAT1		: u32 = (ARM_PWM_BASE + 0x14);
pub const  ARM_PWM_FIF1		: u32 = (ARM_PWM_BASE + 0x18);
pub const  ARM_PWM_RNG2		: u32 = (ARM_PWM_BASE + 0x20);
pub const  ARM_PWM_DAT2		: u32 = (ARM_PWM_BASE + 0x24);


// DMA channel resource management
pub const  DMA_CHANNEL_MAX		: u32 = 12;			// channels 0-12 are supported
pub const  DMA_CHANNEL__MASK	: u32 = 0x0F;		// explicit channel number 0-12
pub const  DMA_CHANNEL_NONE	: u32 = 0x80;		// returned if no channel available
pub const  DMA_CHANNEL_NORMAL	: u32 = 0x81;		// normal DMA engine requested
pub const  DMA_CHANNEL_LITE	: u32 = 0x82;		// lite (or normal) DMA engine requested