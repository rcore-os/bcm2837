use crate::addr::IO_BASE;
use crate::timer::*;
use volatile::Volatile;

// PWM (ref: peripherals page 141)
const ARM_PWM_BASE: usize = IO_BASE + 0x20C000;
pub const ARM_PWM_CTL: usize = (ARM_PWM_BASE + 0x00);
pub const ARM_PWM_STA: usize = (ARM_PWM_BASE + 0x04);
pub const ARM_PWM_DMAC: usize = (ARM_PWM_BASE + 0x08);
pub const ARM_PWM_RNG1: usize = (ARM_PWM_BASE + 0x10);
pub const ARM_PWM_DAT1: usize = (ARM_PWM_BASE + 0x14);
pub const ARM_PWM_FIF1: usize = (ARM_PWM_BASE + 0x18);
pub const ARM_PWM_RNG2: usize = (ARM_PWM_BASE + 0x20);
pub const ARM_PWM_DAT2: usize = (ARM_PWM_BASE + 0x24);

// GPIO clock (ref: peripherals page 107)
const CLK_CTL_BUSY: u32 = (1 << 7);
const CLK_CTL_KILL: u32 = (1 << 5);
const CLK_CTL_ENAB: u32 = (1 << 4);
pub const ARM_CM_BASE: usize = IO_BASE + 0x101000;
pub const ARM_CM_GP0CTL: usize = ARM_CM_BASE + 0x70;
pub const ARM_CM_GP0DIV: usize = ARM_CM_BASE + 0x74;
pub const ARM_CM_PASSWD: u32 = (0x5A << 24);

// PWM control register
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

// PWM status register
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

// PWM DMA configuration register
pub const ARM_PWM_DMAC_DREQ__SHIFT: usize = 0;
pub const ARM_PWM_DMAC_PANIC__SHIFT: usize = 8;
pub const ARM_PWM_DMAC_ENAB: usize = (1 << 31);

// PWM address map (ref: peripherals pages 141)
#[repr(C)]
#[allow(non_snake_case)]
struct PWMAddressMap {
    CTL: Volatile<u32>,
    STA: Volatile<u32>,
    DMAC: Volatile<u32>,
    reserve1: Volatile<u32>,
    RNG1: Volatile<u32>,
    DAT1: Volatile<u32>,
    FIF1: Volatile<u32>,
    reserve2: Volatile<u32>,
    RNG2: Volatile<u32>,
    DAT2: Volatile<u32>,
}

pub struct PWMOutput {
    pwm_address_map: &'static mut PWMAddressMap,
}

impl PWMOutput {
    pub fn new() -> PWMOutput {
        PWMOutput {
            pwm_address_map: unsafe { &mut *(ARM_PWM_BASE as *mut PWMAddressMap) },
        }
    }

    pub fn start(&mut self, range: usize, use_fifo: bool) {
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
            // do nothing
        }
        // start
        gpio_clk_div.write(ARM_CM_PASSWD | (n_div_i << 12) | (n_div_f << 0));
        delay_us(10);
        gpio_clk_ctl.write(ARM_CM_PASSWD | (n_div_mash << 9) | (clk_source << 0));
        delay_us(10);
        gpio_clk_ctl.write(gpio_clk_ctl.read() | ARM_CM_PASSWD | CLK_CTL_ENAB);
        while gpio_clk_ctl.read() & CLK_CTL_BUSY == 0 {
            // do nothing
        }

        delay_us(2000);
        self.pwm_address_map.RNG1.write(range as u32);
        self.pwm_address_map.RNG2.write(range as u32);
        self.pwm_address_map.CTL.write(
            self.pwm_address_map.CTL.read() | (ARM_PWM_CTL_PWEN1 | ARM_PWM_CTL_PWEN2) as u32,
        );
        self.pwm_address_map.STA.write(0);
        if use_fifo {
            self.pwm_address_map.CTL.write(
                self.pwm_address_map.CTL.read()
                    | (ARM_PWM_CTL_USEF1 | ARM_PWM_CTL_USEF2 | ARM_PWM_CTL_CLRF1) as u32,
            );
        }
        delay_us(2000);
    }

    pub fn dma_start(&mut self) {
        self.pwm_address_map.DMAC.write(
            (ARM_PWM_DMAC_ENAB | (7 << ARM_PWM_DMAC_PANIC__SHIFT) | (7 << ARM_PWM_DMAC_DREQ__SHIFT))
                as u32,
        );
    }

    pub fn write(&mut self, channel: usize, data: u32) {
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_BERR as u32 != 0 {
            self.pwm_address_map.STA.write(ARM_PWM_STA_BERR as u32);
        }
        if channel == 1 {
            self.pwm_address_map.DAT1.write(data);
        } else {
            self.pwm_address_map.DAT2.write(data);
        }
    }

    pub fn write_fifo(&mut self, data: u32) {
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_BERR as u32 != 0 {
            self.pwm_address_map.STA.write(ARM_PWM_STA_BERR as u32);
        }
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_WERR1 as u32 != 0 {
            self.pwm_address_map.STA.write(ARM_PWM_STA_WERR1 as u32);
        }
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_GAPO1 as u32 != 0 {
            self.pwm_address_map.STA.write(ARM_PWM_STA_GAPO1 as u32);
        }
        if self.pwm_address_map.STA.read() & ARM_PWM_STA_GAPO2 as u32 != 0 {
            self.pwm_address_map.STA.write(ARM_PWM_STA_GAPO2 as u32);
        }
        while self.pwm_address_map.STA.read() & ARM_PWM_STA_FULL1 as u32 != 0 {
            // do nothing
        }
        self.pwm_address_map.FIF1.write(data);
    }

    pub fn stop(&mut self) {
        self.pwm_address_map.CTL.write(0);
    }
}
