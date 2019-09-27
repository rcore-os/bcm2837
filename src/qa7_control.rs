//! BCM2386 Quad-A7 control
//! ref: BCM2836 ARM-local peripherals (Quad-A7 control)

use crate::addr::CONTROL_BASE;
use volatile::*;

/// Local timer, IRQs, mailboxes registers (ref: QA7 chapter 4, page 7)
#[allow(non_snake_case)]
#[repr(C)]
pub struct Registers {
    pub CONTROL: Volatile<u32>, // 0x0
    _unused0: [Volatile<u32>; 2],
    pub GPU_ROUTE: Volatile<u32>, // 0xc
    _unused1: [Volatile<u32>; 5],
    pub LOCAL_TIMER_ROUTING: Volatile<u32>, // 0x24
    _unused2: [Volatile<u32>; 3],
    pub LOCAL_TIMER_CTL: Volatile<u32>,   // 0x34
    pub LOCAL_TIMER_FLAGS: Volatile<u32>, // 0x38
    _unused3: Volatile<u32>,
    pub CORE_TIMER_IRQCNTL: [Volatile<u32>; 4],   // 0x40
    pub CORE_MAILBOX_IRQCNTL: [Volatile<u32>; 4], // 0x50
    pub CORE_IRQ_SRC: [Volatile<u32>; 4],         // 0x60
    pub CORE_FIQ_SRC: [Volatile<u32>; 4],         // 0x70
}

/// Core interrupt sources (ref: QA7 4.10, page 16)
#[repr(u8)]
#[allow(dead_code)]
#[allow(non_snake_case)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum CoreInterruptSource {
    CNTPSIRQ = 0,
    CNTPNSIRQ = 1,
    CNTHPIRQ = 2,
    CNTVIRQ = 3,
    Mailbox0 = 4,
    Mailbox1 = 5,
    Mailbox2 = 6,
    Mailbox3 = 7,
    Gpu = 8,
    Pmu = 9,
    AxiOutstanding = 10,
    LocalTimer = 11,
}

/// Quad-A7 control
pub struct QA7Control {
    pub registers: &'static mut Registers,
}

impl QA7Control {
    #[inline]
    pub fn new() -> Self {
        QA7Control {
            registers: unsafe { &mut *(CONTROL_BASE as *mut Registers) },
        }
    }

    #[inline]
    pub fn is_irq_pending(&self, core: usize, irq: CoreInterruptSource) -> bool {
        self.registers.CORE_IRQ_SRC[core].read() & (1 << (irq as u8)) != 0
    }
}
