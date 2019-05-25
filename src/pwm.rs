use crate::consts::*;
use crate::timer::*;
use crate::util::*;
use alloc::vec::*;
use core::mem::size_of;
use core::ptr;
use volatile::{ReadOnly, Volatile, WriteOnly};

const CLK_CTL_BUSY: u32 = (1 << 7);
const CLK_CTL_KILL: u32 = (1 << 5);
const CLK_CTL_ENAB: u32 = (1 << 4);

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
            pwm_address_map: unsafe { &mut *(ARM_PWM_BASE as *mut PWMAddressMap) }
        }
    }

    pub fn start(&mut self, range: usize, use_fifo: bool) {
        // warn!("range: {}", ((CLOCK_FREQ / CLOCK_DIVIDER + sample_rate / 2) / sample_rate));
        // init gpio clock
        let clk_clock = 6;
        let clk_source = 6 as u32;
        let n_div_i = 2 as u32;
        let n_div_f = 0 as u32;
        let n_div_mash = 0 as u32;
        let gpio_clk_ctl: &'static mut Volatile<u32> = 
            unsafe { &mut *((ARM_CM_GP0CTL + (clk_clock * 8)) as *mut Volatile<u32>) };
        let gpio_clk_div: &'static mut Volatile<u32> = 
            unsafe { &mut *((ARM_CM_GP0DIV + (clk_clock * 8)) as *mut Volatile<u32>) };
        // stop
        gpio_clk_ctl.write(ARM_CM_PASSWD | CLK_CTL_KILL);
        while gpio_clk_ctl.read() & CLK_CTL_BUSY != 0 {
            //warn!("ctl: {}", gpio_clk_ctl.read());
            // do nothing
        }
        // start
        gpio_clk_div.write(ARM_CM_PASSWD | (n_div_i << 12) | (n_div_f << 0));
        delay_us(10);
        gpio_clk_ctl.write(ARM_CM_PASSWD | (n_div_mash << 9) | (clk_source << 0));
        delay_us(10);
        gpio_clk_ctl.write(gpio_clk_ctl.read() | ARM_CM_PASSWD | CLK_CTL_ENAB);
        while gpio_clk_ctl.read() & CLK_CTL_BUSY == 0 {
            //warn!("ctl: {}", gpio_clk_ctl.read());
            // do nothing
        }

        //self.pwm_address_map.RNG1.write(((CLOCK_FREQ / CLOCK_DIVIDER + sample_rate / 2) / sample_rate) as u32);
        //self.pwm_address_map.RNG2.write(((CLOCK_FREQ / CLOCK_DIVIDER + sample_rate / 2) / sample_rate) as u32);
        delay_us(2000);
        self.pwm_address_map.RNG1.write(range as u32);
        self.pwm_address_map.RNG2.write(range as u32);
        self.pwm_address_map.CTL.write(self.pwm_address_map.CTL.read() |
            ( ARM_PWM_CTL_PWEN1 | ARM_PWM_CTL_PWEN2) as u32);
            //| ARM_PWM_CTL_MSEN1 | ARM_PWM_CTL_MSEN2) as u32);
        self.pwm_address_map.STA.write(0);
        if use_fifo {
            self.pwm_address_map.CTL.write(self.pwm_address_map.CTL.read() |
                (ARM_PWM_CTL_USEF1 | ARM_PWM_CTL_USEF2 | ARM_PWM_CTL_CLRF1) as u32);
        }
        delay_us(2000);
        warn!("PWM CTL: {}", self.pwm_address_map.CTL.read());
        delay_us(2000);
    }

    pub fn write(&mut self, channel: usize, data: u32) {
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_BERR as u32 != 0 {
            warn!("pwm write error!: {}", self.pwm_address_map.STA.read());
            self.pwm_address_map.STA.write(ARM_PWM_STA_BERR as u32);
        }
        if channel == 1 {
            self.pwm_address_map.DAT1.write(data);
        } else {
            self.pwm_address_map.DAT2.write(data);
        }
    }

    pub fn writeFIFO(&mut self, data: u32) {
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_BERR as u32 != 0 {
            // warn!("pwm beer error!: {}", self.pwm_address_map.STA.read());
            self.pwm_address_map.STA.write(ARM_PWM_STA_BERR as u32);
        }
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_WERR1 as u32 != 0 {
            // warn!("pwm write1 error!: {}", self.pwm_address_map.STA.read());
            self.pwm_address_map.STA.write(ARM_PWM_STA_WERR1 as u32);
        }
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_GAPO1 as u32 != 0 {
            // warn!("pwm gap1 error!: {}", self.pwm_address_map.STA.read());
            self.pwm_address_map.STA.write(ARM_PWM_STA_GAPO1 as u32);
        }
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_GAPO2 as u32 != 0 {
            // warn!("pwm gap2 error!: {}", self.pwm_address_map.STA.read());
            self.pwm_address_map.STA.write(ARM_PWM_STA_GAPO2 as u32);
        }
        while self.pwm_address_map.STA.read() & ARM_PWM_STA_FULL1 as u32 != 0 {
            // warn!("pwm FIFO queue full! sta: {}", self.pwm_address_map.STA.read());
            // do nothing
        }
        self.pwm_address_map.FIF1.write(data);
    }

    pub fn stop(&mut self) {
        self.pwm_address_map.CTL.write(0);
    }
}
