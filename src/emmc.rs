use crate::consts::EMMC_BASE;
use volatile::{ReadOnly, Volatile, WriteOnly};

#[repr(C)]
#[allow(non_snake_case)]
struct EmmcRegisters {
    ARG2: Volatile<u32>,
    BLKSIZECNT: Volatile<u32>,
    ARG1: Volatile<u32>,
    CMDTM: Volatile<u32>,
    RESP: [Volatile<u32>; 4],
    DATA: Volatile<u32>,
    STATUS: Volatile<u32>,
    CONTROL0: Volatile<u32>,
    CONTROL1: Volatile<u32>,
    INTERRUPT: Volatile<u32>,
    IRPT_MASK: Volatile<u32>,
    IRPT_EN: Volatile<u32>,
    CONTROL2: Volatile<u32>,
    __CAPABILITIES_NOT_SUPPORTED: [Volatile<u32>; 2],
    __reserved0: [Volatile<u32>; 2],
    FORCE_IRPT: Volatile<u32>,
    __reserved1: [Volatile<u32>; 7],
    BOOT_TIMEOUT: Volatile<u32>,
    DBG_SEL: Volatile<u32>,
    __reserved2: [Volatile<u32>; 2],
    EXRDFIFO_CFG: Volatile<u32>,
    EXRDFIFO_EN: Volatile<u32>,
    TUNE_STEP: Volatile<u32>,
    TUNE_STEPS_STD: Volatile<u32>,
    TUNE_STEPS_DDR: Volatile<u32>,
    __reserved3: [Volatile<u32>; 23],
    SPI_INT_SPT: Volatile<u32>,
    __reserved4: [Volatile<u32>; 2],
    SLOTISR_VER: Volatile<u32>,
}

struct Emmc {
    registers: &'static mut EmmcRegisters,
}

impl Emmc {
    pub fn new() -> Emmc {
        Emmc {
            registers : unsafe { &mut *(EMMC_BASE as *mut EmmcRegisters) }
        }
    }
}
