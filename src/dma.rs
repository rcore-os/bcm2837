use crate::consts::*;
use crate::timer::*;
use crate::util::*;
use crate::pwm::*;

use alloc::vec::*;
use core::mem::size_of;
use core::ptr;
use volatile::{ReadOnly, Volatile, WriteOnly};

use aarch64::asm::*;
use aarch64::barrier;

#[inline(always)]
fn dmb() {
    unsafe {
        barrier::wmb();
        barrier::rmb();
    }
}

fn BUS_ADDRESS(addr: usize) -> usize {
    ((addr) & !0xC0000000) | GPU_MEM_BASE
}

fn DMA_CHANNEL_BASE(channel: usize) -> usize {
    ARM_DMA_BASE + ((channel) * 0x100)
}

#[repr(C)]
#[allow(non_snake_case)]
struct DMAControlBlock{
    TI: Volatile<u32>,
    SOURCE_AD: Volatile<u32>,
    DEST_AD: Volatile<u32>,
    TXFR_LEN: Volatile<u32>,
    STRIDE: Volatile<u32>,
    NEXTCONBK: Volatile<u32>,
    reserved: [Volatile<u32>; 2]
}

#[repr(C)]
#[allow(non_snake_case)]
struct DMAChannelRegister{
    CS: Volatile<u32>,
    CONBLK_AD: Volatile<u32>,
    TI: Volatile<u32>,
    SOURCE_AD: Volatile<u32>,
    DEST_AD: Volatile<u32>,
    TXFR_LEN: Volatile<u32>,
    STRIDE: Volatile<u32>,
    NEXTCONBZK: Volatile<u32>,
    DEBUG: Volatile<u32>
}

pub struct DMA {
    dma_control_block: &'static mut DMAControlBlock,
    dma_buffer_vaddr: usize,
    dma_channel_register: &'static mut DMAChannelRegister,
    dma_control_block_paddr: usize,
    dma_buffer_paddr: usize,
    dma_enable_register: &'static mut Volatile<u32>,
    dma_channel: usize,
    dma_chunk_size: usize
}

impl DMA {
    pub fn new(dma_channel: usize, dma_chunk_size: usize,
        dma_cb_vaddr: usize, dma_cb_paddr: usize, dma_buffer_vaddr: usize, dma_buffer_paddr: usize) -> DMA {
        DMA {
            dma_control_block: unsafe { &mut *(dma_cb_vaddr as *mut DMAControlBlock) },
            dma_buffer_vaddr: dma_buffer_vaddr,
            dma_channel_register: unsafe { &mut *(DMA_CHANNEL_BASE(dma_channel) as *mut DMAChannelRegister) },
            dma_control_block_paddr: dma_cb_paddr,
            dma_buffer_paddr: dma_buffer_paddr,
            dma_enable_register: unsafe { &mut *(ARM_DMA_ENABLE as *mut Volatile<u32>) },
            dma_channel: dma_channel,
            dma_chunk_size: dma_chunk_size
        }
    }

    pub fn start(&mut self) {
        // Enable channel in global ENABLE register
        unsafe { barrier::wmb(); }
        self.dma_enable_register.write(self.dma_enable_register.read() | (1 << self.dma_channel));

        // Setup DMA control block
        let DREQSourcePWM = 5;
        self.dma_control_block.TI.write((((DREQSourcePWM as usize) << TI_PERMAP_SHIFT)
            | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
			| TI_SRC_WIDTH
            | TI_SRC_INC
            | TI_DEST_DREQ
            | TI_WAIT_RESP
            | TI_INTEN) as u32);
        self.dma_control_block.SOURCE_AD.write(BUS_ADDRESS(self.dma_buffer_paddr) as u32);
        self.dma_control_block.DEST_AD.write(((ARM_PWM_FIF1 & 0xFFFFFF) + GPU_IO_BASE) as u32);
        // self.dma_control_block.NEXTCONBK.write(BUS_ADDRESS(self.dma_control_block_paddr) as u32); // debug
        self.dma_control_block.NEXTCONBK.write(0); 
        self.dma_control_block.STRIDE.write(0);
        self.dma_control_block.TXFR_LEN.write((self.dma_chunk_size * 4) as u32);
        self.dma_control_block.reserved[0].write(0);
        self.dma_control_block.reserved[1].write(0);
        delay_us(2000);

        // Invalidate cache of DMA control block and buffer
        flush_dcache_range((self.dma_control_block as *const DMAControlBlock) as usize, 
            (self.dma_control_block as *const DMAControlBlock) as usize + 32);
        flush_dcache_range(self.dma_buffer_vaddr, self.dma_chunk_size * 4);

        // Set DMA channel register
        self.dma_channel_register.CS.write(self.dma_channel_register.CS.read() | (1 << 30)); // Abort
        self.dma_channel_register.CS.write(self.dma_channel_register.CS.read() | (1 << 31)); // Reset
        self.dma_channel_register.CS.write(self.dma_channel_register.CS.read() | (1 << 1)); // Clear END status
        self.dma_channel_register.CONBLK_AD.write(BUS_ADDRESS(self.dma_control_block_paddr) as u32); // Write CB addr
        delay_us(2000);
        unsafe { barrier::wmb(); }
        self.dma_channel_register.CS.write((CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)) as u32
        ); // Set CS
        self.dma_channel_register.CS.write((CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)
                | CS_ACTIVE) as u32
        ); // Set CS active
    }

    pub fn stop(&mut self) {
        self.dma_channel_register.CS.write(self.dma_channel_register.CS.read() | (1 << 30)); // Abort
        self.dma_channel_register.CS.write(self.dma_channel_register.CS.read() | (1 << 31)); // Reset
        self.dma_channel_register.CS.write(self.dma_channel_register.CS.read() | (1 << 1)); // Clear END status
    }

    fn print_dma_register(&self) {
        unsafe {
            warn!("Read from DMA:");
            warn!("    CS: {}", self.dma_channel_register.CS.read());
            warn!("    CONBLK_AD: {}", self.dma_channel_register.CONBLK_AD.read());
            warn!("    TI: {}", self.dma_channel_register.TI.read());
            warn!("    SOURCE_AD: {}", self.dma_channel_register.SOURCE_AD.read());
            warn!("    DEST_AD: {}", self.dma_channel_register.DEST_AD.read());
            warn!("    TXFR_LEN: {}", self.dma_channel_register.TXFR_LEN.read());
            warn!("    NEXTCONBK: {}", self.dma_channel_register.NEXTCONBZK.read());
            warn!("    DEBUG: {}", self.dma_channel_register.DEBUG.read());
            warn!("");
            
        }
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
