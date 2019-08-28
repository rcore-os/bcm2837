use crate::addr::IO_BASE;
use volatile::Volatile;

const EMMC_BASE: usize = IO_BASE + 0x300000;

#[repr(C)]
#[allow(non_snake_case)]
pub struct EmmcRegisters {
    pub ARG2: Volatile<u32>,
    pub BLKSIZECNT: Volatile<u32>,
    pub ARG1: Volatile<u32>,
    pub CMDTM: Volatile<u32>,
    pub RESP: [Volatile<u32>; 4],
    pub DATA: Volatile<u32>,
    pub STATUS: Volatile<u32>,
    pub CONTROL0: Volatile<u32>,
    pub CONTROL1: Volatile<u32>,
    pub INTERRUPT: Volatile<u32>,
    pub IRPT_MASK: Volatile<u32>,
    pub IRPT_EN: Volatile<u32>,
    pub CONTROL2: Volatile<u32>,
    __CAPABILITIES_NOT_SUPPORTED: [Volatile<u32>; 2],
    __reserved0: [Volatile<u32>; 2],
    pub FORCE_IRPT: Volatile<u32>,
    __reserved1: [Volatile<u32>; 7],
    pub BOOT_TIMEOUT: Volatile<u32>,
    pub DBG_SEL: Volatile<u32>,
    __reserved2: [Volatile<u32>; 2],
    pub EXRDFIFO_CFG: Volatile<u32>,
    pub EXRDFIFO_EN: Volatile<u32>,
    pub TUNE_STEP: Volatile<u32>,
    pub TUNE_STEPS_STD: Volatile<u32>,
    pub TUNE_STEPS_DDR: Volatile<u32>,
    __reserved3: [Volatile<u32>; 23],
    pub SPI_INT_SPT: Volatile<u32>,
    __reserved4: [Volatile<u32>; 2],
    pub SLOTISR_VER: Volatile<u32>,
}

pub struct Emmc {
    pub registers: &'static mut EmmcRegisters,
}

impl Emmc {
    pub fn new() -> Emmc {
        Emmc {
            registers: unsafe { &mut *(EMMC_BASE as *mut EmmcRegisters) },
        }
    }
}
