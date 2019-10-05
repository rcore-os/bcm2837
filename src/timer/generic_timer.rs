use super::BasicTimer;
use crate::qa7_control::{CoreInterruptSource, QA7Control};
use aarch64::{asm, regs::*};

/// Core timers interrupts (ref: QA7 4.6, page 13)
#[repr(u8)]
#[allow(dead_code)]
#[allow(non_snake_case)]
#[derive(Copy, Clone, PartialEq, Debug)]
enum CoreTimerControl {
    CNTPSIRQ = 0,
    CNTPNSIRQ = 1,
    CNTHPIRQ = 2,
    CNTVIRQ = 3,
}

/// The ARM generic timer.
pub struct GenericTimer {
    control: QA7Control,
}

impl BasicTimer for GenericTimer {
    #[inline]
    fn freq() -> u64 {
         // 62500000 on qemu, 19200000 on real machine
        CNTFRQ_EL0.get() as u64
    }

    #[inline]
    fn new() -> Self {
        GenericTimer {
            control: QA7Control::new(),
        }
    }

    #[inline]
    fn init(&mut self) {
        self.control.registers.CORE_TIMER_IRQCNTL[asm::cpuid()]
            .write(1 << (CoreTimerControl::CNTPNSIRQ as u8));
        CNTP_CTL_EL0.write(CNTP_CTL_EL0::ENABLE::SET);
    }

    #[inline]
    fn stop(&mut self) {
        self.control.registers.CORE_TIMER_IRQCNTL[asm::cpuid()].write(0);
    }

    #[inline]
    fn read(&self) -> u64 {
        (CNTPCT_EL0.get() * 1000000 / Self::freq()) as u64
    }

    #[inline]
    fn tick_in(&mut self, us: usize) {
        let count = Self::freq() * (us as u64) / 1000000;
        // max `68719476` us (0xffff_ffff / 38400000 * 62500000).
        debug_assert!(count <= u32::max_value() as u64);
        CNTP_TVAL_EL0.set(count as u32);
    }

    #[inline]
    fn is_pending(&self) -> bool {
        self.control
            .is_irq_pending(asm::cpuid(), CoreInterruptSource::CNTPNSIRQ)
    }
}
