use crate::consts::*;
use crate::timer::*;
use crate::util::*;
use alloc::vec::*;
use core::mem::size_of;
use core::ptr;
use volatile::{ReadOnly, Volatile, WriteOnly};

#[repr(C)]
#[allow(non_snake_case)]
struct PWMAddressMap{
    CTL: Volatile<u32>,
    STA: Volatile<u32>,
    DMAC: Volatile<u32>,
    reserve1: Volatile<u32>,
    RNG1: Volatile<u32>,
    DAT1: Volatile<u32>,
    FIF1: Volatile<u32>,
    reserve2: Volatile<u32>,
    RNG2: Volatile<u32>,
    DAT2: Volatile<u32>
}

pub struct PWMOutput {
    pwm_address_map: &'static mut PWMAddressMap
}

impl PWMOutput {
    pub fn new() -> PWMOutput {
        PWMOutput {
            pwm_address_map: unsafe { &mut *(ARM_PWM_BASE as *mut DMAEnableRegister) }
        }
    }

    pub fn start(sample_rate: usize) {
        self.pwm_address_map.RNG1.write((CLOCK_FREQ / CLOCK_DIVIDER + sample_rate / 2) / sample_rate);
        self.pwm_address_map.RNG2.write((CLOCK_FREQ / CLOCK_DIVIDER + sample_rate / 2) / sample_rate);
        self.pwm_address_map.CTL.write(self.pwm_address_map.CTL.read()
            | ARM_PWM_CTL_PWEN1 | ARM_PWM_CTL_PWEN2
            | ARM_PWM_CTL_USEF1 | ARM_PWM_CTL_USEF2
            | ARM_PWM_CTL_CLRF1)
    }

    pub fn write(channel: usize, data: usize) {
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_BERR {
            warn!("pwm write error!");
        }
        if channel == 1 {
            self.pwm_address_map.DAT1.write(data);
        } else {
            self.pwm_address_map.DAT2.write(data);
        }
    }

    pub fn stop() {
        self.pwm_address_map.CTL.write(0);
    }
}
