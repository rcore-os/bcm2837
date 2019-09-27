use super::BasicTimer;
use crate::qa7_control::{CoreInterruptSource, QA7Control};

const LOCAL_TIMER_VALUE_MASK: u32 = 0x0FFF_FFFF;
const LOCAL_TIMER_FREQ: u64 = 38400000;

/// Local timer flags of `LOCAL_TIMER_CTL` and `LOCAL_TIMER_FLAGS`
#[repr(u32)]
#[derive(Copy, Clone, PartialEq, Debug)]
enum LocalTimerFlags {
    TimerEnable = 1 << 28,
    InterruptEnable = 1 << 29,
    Reloaded = 1 << 30,
    InterruptFlag = 1 << 31,
}

/// The BCM2836 local timer.
pub struct LocalTimer {
    control: QA7Control,
}

impl BasicTimer for LocalTimer {
    #[inline]
    fn freq() -> u64 {
        LOCAL_TIMER_FREQ
    }

    #[inline]
    fn new() -> Self {
        LocalTimer {
            control: QA7Control::new(),
        }
    }

    #[inline]
    fn init(&mut self) {
        self.control.registers.LOCAL_TIMER_ROUTING.write(0);
    }

    #[inline]
    fn stop(&mut self) {
        self.control.registers.LOCAL_TIMER_CTL.write(0);
        self.control
            .registers
            .LOCAL_TIMER_FLAGS
            .write(LocalTimerFlags::InterruptFlag as u32);
    }

    fn read(&self) -> u64 {
        unimplemented!()
    }

    #[inline]
    fn tick_in(&mut self, us: usize) {
        let count = Self::freq() * (us as u64) / 1000000;
        // max `6990506` us (0xfff_ffff / 38400000 * 1000000).
        debug_assert!(count <= LOCAL_TIMER_VALUE_MASK as u64);
        self.control.registers.LOCAL_TIMER_CTL.write(
            LocalTimerFlags::TimerEnable as u32
                | LocalTimerFlags::InterruptEnable as u32
                | count as u32 & LOCAL_TIMER_VALUE_MASK,
        );
        self.control
            .registers
            .LOCAL_TIMER_FLAGS
            .write(LocalTimerFlags::Reloaded as u32 | LocalTimerFlags::InterruptFlag as u32);
    }

    #[inline]
    fn is_pending(&self) -> bool {
        self.control
            .is_irq_pending(0, CoreInterruptSource::LocalTimer)
    }
}
