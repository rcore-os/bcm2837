use crate::addr::{io_to_bus, phys_to_bus, IO_BASE};
use crate::pwm::ARM_PWM_FIF1;
use crate::timer::*;
use volatile::Volatile;

use aarch64::barrier;
use aarch64::cache::*;

const ARM_DMA_BASE: usize = IO_BASE + 0x7000;

// DMA channel resource management (ref: peripherals page 39)
pub const DMA_CHANNEL_MAX: usize = 12; // channels 0-12 are supported
pub const DMA_CHANNEL__MASK: usize = 0x0F; // explicit channel number 0-12
pub const DMA_CHANNEL_NONE: usize = 0x80; // returned if no channel available
pub const DMA_CHANNEL_NORMAL: usize = 0x81; // normal DMA engine requested
pub const DMA_CHANNEL_LITE: usize = 0x82; // lite (or normal) DMA engine requested

// DMA controller (ref: peripherals page 39)
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

#[allow(non_snake_case)]
fn DMA_CHANNEL_BASE(channel: usize) -> usize {
    ARM_DMA_BASE + ((channel) * 0x100)
}

#[repr(C)]
#[allow(non_snake_case)]
struct DMAControlBlock {
    TI: Volatile<u32>,
    SOURCE_AD: Volatile<u32>,
    DEST_AD: Volatile<u32>,
    TXFR_LEN: Volatile<u32>,
    STRIDE: Volatile<u32>,
    NEXTCONBK: Volatile<u32>,
    reserved: [Volatile<u32>; 2],
}

#[repr(C)]
#[allow(non_snake_case)]
struct DMAChannelRegister {
    CS: Volatile<u32>,
    CONBLK_AD: Volatile<u32>,
    TI: Volatile<u32>,
    SOURCE_AD: Volatile<u32>,
    DEST_AD: Volatile<u32>,
    TXFR_LEN: Volatile<u32>,
    STRIDE: Volatile<u32>,
    NEXTCONBZK: Volatile<u32>,
    DEBUG: Volatile<u32>,
}

pub struct DMA {
    dma_control_block: &'static mut DMAControlBlock,
    dma_buffer_vaddr: usize,
    dma_channel_register: &'static mut DMAChannelRegister,
    dma_control_block_paddr: usize,
    dma_buffer_paddr: usize,
    dma_enable_register: &'static mut Volatile<u32>,
    dma_channel: usize,
    dma_chunk_size: usize,
}

impl DMA {
    pub fn new(
        dma_channel: usize,
        dma_chunk_size: usize,
        dma_cb_vaddr: usize,
        dma_cb_paddr: usize,
        dma_buffer_vaddr: usize,
        dma_buffer_paddr: usize,
    ) -> DMA {
        DMA {
            dma_control_block: unsafe { &mut *(dma_cb_vaddr as *mut DMAControlBlock) },
            dma_buffer_vaddr: dma_buffer_vaddr,
            dma_channel_register: unsafe {
                &mut *(DMA_CHANNEL_BASE(dma_channel) as *mut DMAChannelRegister)
            },
            dma_control_block_paddr: dma_cb_paddr,
            dma_buffer_paddr: dma_buffer_paddr,
            dma_enable_register: unsafe { &mut *(ARM_DMA_ENABLE as *mut Volatile<u32>) },
            dma_channel: dma_channel,
            dma_chunk_size: dma_chunk_size,
        }
    }

    pub fn start(&mut self) {
        // Enable channel in global ENABLE register
        unsafe {
            barrier::wmb();
        }
        self.dma_enable_register
            .write(self.dma_enable_register.read() | (1 << self.dma_channel));

        // Setup DMA control block
        let dreq_source_pwm = 5;
        self.dma_control_block.TI.write(
            (((dreq_source_pwm as usize) << TI_PERMAP_SHIFT)
                | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
                | TI_SRC_WIDTH
                | TI_SRC_INC
                | TI_DEST_DREQ
                | TI_WAIT_RESP
                | TI_INTEN) as u32,
        );
        self.dma_control_block
            .SOURCE_AD
            .write(phys_to_bus(self.dma_buffer_paddr as u32));
        self.dma_control_block
            .DEST_AD
            .write(io_to_bus(ARM_PWM_FIF1 as u32));
        self.dma_control_block.NEXTCONBK.write(0); // currently do not support block loop
        self.dma_control_block.STRIDE.write(0);
        self.dma_control_block
            .TXFR_LEN
            .write((self.dma_chunk_size * 4) as u32);
        self.dma_control_block.reserved[0].write(0);
        self.dma_control_block.reserved[1].write(0);
        delay_us(2000);

        // Invalidate cache of DMA control block and buffer
        DCache::<Clean, PoC>::flush_area(
            (self.dma_control_block as *const DMAControlBlock) as usize,
            32,
            SY,
        );
        DCache::<Clean, PoC>::flush_area(self.dma_buffer_vaddr, self.dma_chunk_size * 4, SY);

        // Set DMA channel register
        self.dma_channel_register
            .CS
            .write(self.dma_channel_register.CS.read() | (1 << 30)); // Abort
        self.dma_channel_register
            .CS
            .write(self.dma_channel_register.CS.read() | (1 << 31)); // Reset
        self.dma_channel_register
            .CS
            .write(self.dma_channel_register.CS.read() | (1 << 1)); // Clear END status
        self.dma_channel_register
            .CONBLK_AD
            .write(phys_to_bus(self.dma_control_block_paddr as u32)); // Write CB addr
        delay_us(2000);
        unsafe {
            barrier::wmb();
        }
        self.dma_channel_register.CS.write(
            (CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)) as u32,
        ); // Set CS
        self.dma_channel_register.CS.write(
            (CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)
                | CS_ACTIVE) as u32,
        ); // Set CS active
    }

    pub fn stop(&mut self) {
        self.dma_channel_register
            .CS
            .write(self.dma_channel_register.CS.read() | (1 << 30)); // Abort
        self.dma_channel_register
            .CS
            .write(self.dma_channel_register.CS.read() | (1 << 31)); // Reset
        self.dma_channel_register
            .CS
            .write(self.dma_channel_register.CS.read() | (1 << 1)); // Clear END status
    }

    fn print_dma_register(&self) {
        warn!("Read from DMA:");
        warn!("    CS: {}", self.dma_channel_register.CS.read());
        warn!(
            "    CONBLK_AD: {}",
            self.dma_channel_register.CONBLK_AD.read()
        );
        warn!("    TI: {}", self.dma_channel_register.TI.read());
        warn!(
            "    SOURCE_AD: {}",
            self.dma_channel_register.SOURCE_AD.read()
        );
        warn!("    DEST_AD: {}", self.dma_channel_register.DEST_AD.read());
        warn!(
            "    TXFR_LEN: {}",
            self.dma_channel_register.TXFR_LEN.read()
        );
        warn!(
            "    NEXTCONBK: {}",
            self.dma_channel_register.NEXTCONBZK.read()
        );
        warn!("    DEBUG: {}", self.dma_channel_register.DEBUG.read());
        warn!("");
    }

    pub fn is_active(&self) -> bool {
        self.print_dma_register();
        if self.dma_channel_register.CONBLK_AD.read() != 0 {
            return true;
        } else {
            return false;
        }
    }
}
