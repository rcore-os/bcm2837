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

fn ARM_DMACHAN_CONBLK_AD(chan: usize) -> usize {
    ARM_DMA_BASE + (chan * 0x100) + 0x04
}

fn BUS_ADDRESS(addr: usize) -> usize {
    ((addr) & !0xC0000000) | GPU_MEM_BASE
}

fn ARM_DMACHAN_CS(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x00
}
fn ARM_DMACHAN_NEXTCONBK(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x1C
}

fn ARM_DMACHAN_DEST_AD(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x10
}

fn ARM_DMACHAN_SRC_AD(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0xc
}

fn ARM_DMACHAN_DEBUG(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x20
}

fn ARM_DMACHAN_TI(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x8
}

fn ARM_DMACHAN_TXFR_LEN(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x14
}

fn ARM_DMA_CHANNEL_BASE(channel: usize) -> usize {
    ARM_DMA_BASE + ((channel) * 0x100)
}

// enum TPWMSoundState
// {
const PWMSoundIdle: usize = 0;
const PWMSoundRunning: usize = 1;
const PWMSoundCancelled: usize = 2;
const PWMSoundTerminating: usize = 3;
const PWMSoundError: usize = 4;
const PWMSoundUnknown: usize = 5;
// }

// impl PartialEq for TPWMSoundState {
//     fn eq(&self, other: &TPWMSoundState) -> bool {
//         return *self == *other;
//     }
// }

enum TDREQ {
    DREQSourceNone = 0,
    DREQSourcePCMTX = 2,
    DREQSourcePCMRX = 3,
    DREQSourcePWM = 5,
    DREQSourceSPITX = 6,
    DREQSourceSPIRX = 7,
    DREQSourceEMMC = 11,
    DREQSourceUARTTX = 12,
    DREQSourceUARTRX = 14,
}

struct TDMAControlBlock {
    nTransferInformation: u32,
    nSourceAddress: u32,
    nDestinationAddress: u32,
    nTransferLength: u32,
    n2DModeStride: u32,
    nNextControlBlockAddress: u32,
    nReserved: [u32; 2],
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

#[repr(C)]
#[allow(non_snake_case)]
struct DMAEnableRegister{
    ENABLE: Volatile<u32>,
}

pub struct PWMSoundDevice {
    m_nChunkSize: usize,
    m_nRange: usize,

    m_State: usize, // TPWMSoundState,

    m_bIRQConnected: bool,

    m_nDMAChannel: usize,
    m_pDMABuffer: [usize; 2],
    m_pDMABufferPADDR: [usize; 2],
    m_pControlBlockBuffer: [Vec<u8>; 2],
    m_pControlBlock: [*mut TDMAControlBlock; 2],
    m_pControlBlockPADDR: [usize; 2],
    m_pControlBlockVADDR: [usize; 2],

    // dma_control_block: [&'static mut DMAControlBlockRegisters; 2],
    // dma_cb_addr: [usize; 2],

    m_nNextBuffer: usize, // 0 or 1

    m_pSoundData: *const u8,
    m_nSamples: usize,
    m_nChannels: usize,
    m_nBitsPerSample: usize, /*
                             CSpinLock m_SpinLock;

                             CGPIOPin   m_Audio1;
                             CGPIOPin   m_Audio2;
                             CGPIOClock m_Clock;
                             */
    
    dma_channel_register: &'static mut DMAChannelRegister,
    dma_control_block: &'static mut DMAControlBlock,
    dma_enable_register: &'static mut DMAEnableRegister,
    pwm_output: PWMOutput
}

fn allocateDMAChannel(nChannel: usize) -> usize {
    if nChannel & !DMA_CHANNEL__MASK == 0 {
        // explicit channel allocation
        return nChannel;
    } else {
        // arbitrary channel allocation
        let i = if nChannel == DMA_CHANNEL_NORMAL {
            6
        } else {
            DMA_CHANNEL_MAX
        };
        return i;
    }
}

impl PWMSoundDevice {
    unsafe fn SetupDMAControlBlock(&mut self, nID: usize) {
        assert!(nID <= 1);

        // self.m_pDMABuffer[nID] = Vec::<u32>::new();
        // self.m_pDMABuffer[nID].resize(self.m_nChunkSize, 0);
        // assert!(self.m_pDMABuffer[nID] != 0);

        // self.m_pControlBlockBuffer[nID] = Vec::<u8>::new();
        // self.m_pControlBlockBuffer[nID].resize(size_of::<TDMAControlBlock>() + 31, 0);
        // assert!(self.m_pControlBlockBuffer[nID] != 0);
        // self.m_pControlBlock[nID] = ((self.m_pControlBlockBuffer[nID].as_ptr().offset(31) as usize)
        //    & !(31usize)) as *mut TDMAControlBlock;

        /*
        (*self.m_pControlBlock[nID]).nTransferInformation = (((TDREQ::DREQSourcePWM as usize)
            << TI_PERMAP_SHIFT)
            | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
            | TI_SRC_WIDTH
            | TI_SRC_INC
            | TI_DEST_DREQ
            | TI_WAIT_RESP
            | TI_INTEN) as u32;
        */

        /*
        (*self.m_pControlBlock[nID]).nTransferInformation = (((TDREQ::DREQSourcePWM as usize)
            << TI_PERMAP_SHIFT)
            | TI_SRC_INC
            | TI_DEST_INC
            | TI_WAIT_RESP
            | TI_INTEN) as u32;

         (*self.m_pControlBlock[nID]).nSourceAddress =
            BUS_ADDRESS(self.m_pDMABufferPADDR[nID]) as u32;
        // (*self.m_pControlBlock[nID]).nSourceAddress = self.m_pDMABufferPADDR[nID] as u32;

        //warn!("3");
        warn!(
            "{} {} {}",
            self.m_pDMABuffer[nID],
            self.m_pDMABufferPADDR[nID],
            (*self.m_pControlBlock[nID]).nSourceAddress
        );

        (*self.m_pControlBlock[nID]).nDestinationAddress =
            ((ARM_PWM_FIF1 & 0xFFFFFF) + GPU_IO_BASE) as u32;
        (*self.m_pControlBlock[nID]).n2DModeStride = 0;
        (*self.m_pControlBlock[nID]).nReserved[0] = 0;
        (*self.m_pControlBlock[nID]).nReserved[1] = 0;

        warn!("write to pwm addr: {}", (*self.m_pControlBlock[nID]).nDestinationAddress);
        */

        self.dma_control_block.TI.write((((TDREQ::DREQSourcePWM as usize)
            << TI_PERMAP_SHIFT)
            | TI_SRC_INC
            | TI_DEST_INC
            | TI_WAIT_RESP
            | TI_INTEN) as u32);
        self.dma_control_block.SOURCE_AD.write(BUS_ADDRESS(self.m_pDMABufferPADDR[nID]) as u32);
        self.dma_control_block.DEST_AD.write(((ARM_PWM_FIF1 & 0xFFFFFF) + GPU_IO_BASE) as u32);
        self.dma_control_block.NEXTCONBK.write(0);
        self.dma_control_block.STRIDE.write(0);
        self.dma_control_block.reserved[0].write(0);
        self.dma_control_block.reserved[1].write(0);

        flush_dcache_range(self.m_pControlBlockVADDR[0], 32);
        let _paddr = address_translate(self.m_pControlBlockVADDR[0]);
        warn!("pwm vaddr {}, paddr {}, aarch64 paddr {}", 
            self.m_pControlBlockVADDR[0],
            self.m_pControlBlockPADDR[0],
            _paddr
        );

        // self.m_pControlBlockPADDR[0] = _paddr;

        
        //warn!("write to pwm addr: {}", (*self.m_pControlBlock[nID]).nDestinationAddress);
    }

    /// \param nSampleRate	sample rate in Hz
    /// \param nChunkSize	twice the number of samples (words) to be handled\n
    ///			with one call to GetChunk() (one word per stereo channel)
    /// default: nSampleRate: 44100, nChunkSize: 2048
    pub fn new(nSampleRate: usize, nChunkSize: usize, vaddr0: usize, paddr0: usize, vaddr1: usize, paddr1: usize, cb0_vaddr: usize, cb0_paddr: usize) -> PWMSoundDevice {
        PWMSoundDevice {
            m_nChunkSize: (nChunkSize),
            m_nRange: ((CLOCK_FREQ / CLOCK_DIVIDER + nSampleRate / 2) / nSampleRate),
            // m_Audio1 (GPIOPinAudioLeft, GPIOModeAlternateFunction0),
            // m_Audio2 (GPIOPinAudioRight, GPIOModeAlternateFunction0),
            // m_Clock (GPIOClockPWM, GPIOClockSourcePLLD),
            m_bIRQConnected: (false),
            m_State: PWMSoundIdle,
            m_nDMAChannel: 5, // allocateDMAChannel(DMA_CHANNEL_LITE),
            m_pDMABuffer: [vaddr0, vaddr1],
            m_pDMABufferPADDR: [paddr0, paddr1],
            m_pControlBlockBuffer: [Vec::<u8>::new(), Vec::<u8>::new()],
            //m_pControlBlock: [ptr::null_mut(); 2],
            m_pControlBlock: [cb0_vaddr as *mut TDMAControlBlock, 0 as *mut TDMAControlBlock],
            m_pControlBlockPADDR: [cb0_paddr, 0],
            m_pControlBlockVADDR: [cb0_vaddr, 0],
            m_nNextBuffer: 0,
            m_pSoundData: ptr::null(),
            m_nSamples: 0,
            m_nChannels: 0,
            m_nBitsPerSample: 0,
            dma_channel_register: unsafe { &mut *(ARM_DMA_CHANNEL_BASE(5) as *mut DMAChannelRegister) },
            dma_control_block: unsafe { &mut *(cb0_vaddr as *mut DMAControlBlock) },
            dma_enable_register: unsafe { &mut *(ARM_DMA_ENABLE as *mut DMAEnableRegister) },
            pwm_output: PWMOutput::new()
        }
    }

    pub fn init(&mut self) {
        unsafe {
            warn!("test test\n");
            self.SetupDMAControlBlock(0);
            // self.SetupDMAControlBlock(1);
            // (*self.m_pControlBlock[0]).nNextControlBlockAddress = 0;
                // BUS_ADDRESS(self.m_pControlBlock[1] as usize) as u32;
            // (*self.m_pControlBlock[1]).nNextControlBlockAddress =
            //    BUS_ADDRESS(self.m_pControlBlock[0] as usize) as u32;

            // start clock and PWM device
            // self.RunPWM();

            // enable and reset DMA channel
            // PeripheralEntry();

            // assert! (self.m_nDMAChannel <= DMA_CHANNEL_MAX);
            /*
            write32(
                ARM_DMA_ENABLE,
                read32(ARM_DMA_ENABLE) | (1 << self.m_nDMAChannel),
            );
            */
            self.pwm_output.start(5669, true);
            self.dma_enable_register.ENABLE.write(
                self.dma_enable_register.ENABLE.read() | (1 << self.m_nDMAChannel));

            delay_us(1000);
            // warn!("read from dma enable register: {}", read32(ARM_DMA_ENABLE));

            // write32(ARM_DMACHAN_CS(self.m_nDMAChannel), CS_RESET as u32);

            /*
            while read32(ARM_DMACHAN_CS(self.m_nDMAChannel)) & CS_RESET as u32 != 0 {
                // do nothing
            }
            */

            // PeripheralExit ();

            // CDeviceNameService::Get ()->AddDevice ("sndpwm", this, FALSE);
        }
    }

    fn GetRangeMin(&self) -> usize {
        0
    }

    fn GetRangeMax(&self) -> usize {
        self.m_nRange - 1
    }

    fn GetRange(&self) -> usize {
        self.GetRangeMax()
    }

    fn PrintStatus(&self) {
        unsafe {
            warn!("Read from DMA:");
            warn!("CS: {}", self.dma_channel_register.CS.read());
            warn!("CONBLK_AD: {}", self.dma_channel_register.CONBLK_AD.read());
            warn!("TI: {}", self.dma_channel_register.TI.read());
            warn!("SOURCE_AD: {}", self.dma_channel_register.SOURCE_AD.read());
            warn!("DEST_AD: {}", self.dma_channel_register.DEST_AD.read());
            warn!("TXFR_LEN: {}", self.dma_channel_register.TXFR_LEN.read());
            warn!("NEXTCONBK: {}", self.dma_channel_register.NEXTCONBZK.read());
            warn!("DEBUG: {}", self.dma_channel_register.DEBUG.read());
            warn!("");
            dmb();
            /*
            warn!("(read from dma) cb ad {}", read32(ARM_DMACHAN_CONBLK_AD(self.m_nDMAChannel)));
            warn!("(read from dma) next cb ad {}", read32(ARM_DMACHAN_NEXTCONBK(self.m_nDMAChannel)));
            warn!("(read from dma) debug {}", read32(ARM_DMACHAN_DEBUG(self.m_nDMAChannel)));
            warn!("(read from dma) trlen {}", read32(ARM_DMACHAN_TXFR_LEN(self.m_nDMAChannel)));
            warn!("(read from dma) ti {}", read32(ARM_DMACHAN_TI(self.m_nDMAChannel)));
            warn!("(read from dma) cs {}", read32(ARM_DMACHAN_CS(self.m_nDMAChannel)));
            warn!("(read from dma) source ad {} dst ad {}", read32(ARM_DMACHAN_SRC_AD(self.m_nDMAChannel)), 
                read32(ARM_DMACHAN_DEST_AD(self.m_nDMAChannel)));
            warn!("");
            warn!("");
            */
        }
    }

    unsafe fn Start(&mut self) -> bool {
        assert!(self.m_State == PWMSoundIdle);

        // fill buffer 0
        self.m_nNextBuffer = 0;

        if !self.GetNextChunk() {
            return false;
        }

        self.m_State = PWMSoundRunning;

        // connect IRQ
        assert!(self.m_nDMAChannel <= DMA_CHANNEL_MAX);

        /*
        if !self.m_bIRQConnected {
            // assert!(self.m_pInterruptSystem != 0);
            // self.m_pInterruptSystem->ConnectIRQ (ARM_IRQ_DMA0+m_nDMAChannel, InterruptStub, this);
            self.m_bIRQConnected = true;
        }
        */

        // enable PWM DMA operation

        /*
        write32(
            ARM_PWM_DMAC,
            (ARM_PWM_DMAC_ENAB | (7 << ARM_PWM_DMAC_PANIC__SHIFT) | (7 << ARM_PWM_DMAC_DREQ__SHIFT))
                as u32,
        );

        // switched this on when playback stops to avoid clicks, switch it off here
        
        write32(
            ARM_PWM_CTL,
            read32(ARM_PWM_CTL) & !(ARM_PWM_CTL_RPTL1 | ARM_PWM_CTL_RPTL2) as u32,
        );
        */
        // start DMA
        // assert!((read32(ARM_DMACHAN_CS(self.m_nDMAChannel)) & CS_INT as u32) == 0);
        // assert!((read32(ARM_DMA_INT_STATUS) & (1 << self.m_nDMAChannel)) == 0);

        self.pwm_output.dma_start();

        delay_us(2000);

        // abort 
        dmb();
        self.dma_channel_register.CS.write(
            self.dma_channel_register.CS.read() | (1 << 30)
        );
        /*
        write32(
            ARM_DMACHAN_CS(self.m_nDMAChannel),
            read32(ARM_DMACHAN_CS(self.m_nDMAChannel)) | (1 << 30)
        );
        */

        delay_us(2000);
        
        // reset
        dmb();
        self.dma_channel_register.CS.write(
            self.dma_channel_register.CS.read() | (1 << 31)
        );
        /* 
        write32(
            ARM_DMACHAN_CS(self.m_nDMAChannel),
            read32(ARM_DMACHAN_CS(self.m_nDMAChannel)) | (1 << 31)
        );
        */

        delay_us(2000);
        
        // clear end
        dmb();
        self.dma_channel_register.CS.write(
            self.dma_channel_register.CS.read() | (1 << 1)
        );
        /*
        write32(
            ARM_DMACHAN_CS(self.m_nDMAChannel),
            read32(ARM_DMACHAN_CS(self.m_nDMAChannel)) | (1 << 1)
        );
        */
        delay_us(2000);
        
        // write cb addr
        dmb();
        self.dma_channel_register.CONBLK_AD.write(
            BUS_ADDRESS(self.m_pControlBlockPADDR[0] as usize) as u32
        );
        /*
        write32(
            ARM_DMACHAN_CONBLK_AD(self.m_nDMAChannel),
            BUS_ADDRESS(self.m_pControlBlockPADDR[0] as usize) as u32,
        );
        */

        delay_us(2000);

        warn!("##### after writing cb, before writing cs #####");
        self.PrintStatus();
        

        delay_us(2000);

        // manually write
        /*
        write32(ARM_DMACHAN_TI(self.m_nDMAChannel), (*self.m_pControlBlock[0]).nTransferInformation);
        write32(ARM_DMACHAN_SRC_AD(self.m_nDMAChannel), (*self.m_pControlBlock[0]).nSourceAddress);
        write32(ARM_DMACHAN_DEST_AD(self.m_nDMAChannel), (*self.m_pControlBlock[0]).nDestinationAddress);
        write32(ARM_DMACHAN_TXFR_LEN(self.m_nDMAChannel), (*self.m_pControlBlock[0]).nTransferLength);
        write32(ARM_DMACHAN_NEXTCONBK(self.m_nDMAChannel), 0);
        delay(2000);
        warn!("##### after manually write #####");
        self.PrintStatus();
        */

        dmb();
        self.dma_channel_register.CS.write(
            (CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)) as u32
        );

        delay_us(2000);
        warn!("##### after writing priority");
        self.PrintStatus();
        
        dmb();
        self.dma_channel_register.CS.write(
            (CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)
                | CS_ACTIVE) as u32
        );
        delay_us(2000);
        /*
        write32(
            ARM_DMACHAN_CS(self.m_nDMAChannel),
            (CS_WAIT_FOR_OUTSTANDING_WRITES
                | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
                | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)
                | CS_ACTIVE) as u32,
        );
        */

        warn!("##### after writing cs #####");
        self.PrintStatus();
    

        // fill buffer 1
        /*
        if !self.GetNextChunk() {
            // m_SpinLock.Acquire ();
            warn!("fail to fill buffer 1");

            if self.m_State == PWMSoundRunning {
                write32 (ARM_DMACHAN_NEXTCONBK (self.m_nDMAChannel), 0);

                self.m_State = PWMSoundTerminating;
            }

            // m_SpinLock.Release ();
        }
        */

        return true;
    }

    fn Cancel(&mut self) {
        // m_SpinLock.Acquire ();
        if self.m_State == PWMSoundRunning {
            self.m_State = PWMSoundCancelled;
        }

        // m_SpinLock.Release ();
    }

    fn IsActive(&self) -> bool {
        unsafe {
            warn!("##### in isactive #####");
            self.PrintStatus();
            delay_us(2000);

            if self.dma_channel_register.CONBLK_AD.read() != 0 {
            // if read32(ARM_DMACHAN_CONBLK_AD(self.m_nDMAChannel)) != 0 {
                return true;
            } else {
                return false;
            }
        }
        /*
        if self.m_State != PWMSoundIdle {
            return true;
        } else {
            return false;
        }
        */
    }

    pub fn Playback(
        &mut self,
        pSoundData: *const u8,
        nSamples: usize,
        nChannels: usize,
        nBitsPerSample: usize,
    ) {
        //assert!(!self.IsActive());
        assert!(pSoundData != ptr::null());
        assert!(nChannels == 1 || nChannels == 2);
        assert!(nBitsPerSample == 8 || nBitsPerSample == 16);

        self.m_pSoundData = pSoundData;
        self.m_nSamples = nSamples;
        self.m_nChannels = nChannels;
        self.m_nBitsPerSample = nBitsPerSample;

        unsafe {
            self.Start();
        }
    }

    pub fn PlaybackActive(&self) -> bool {
        self.IsActive()
    }

    pub fn CancelPlayback(&mut self) {
        self.Cancel();
    }

    /// \brief May overload this to provide the sound samples!
    /// \param pBuffer	buffer where the samples have to be placed
    /// \param nChunkSize	size of the buffer in words (same as given to constructor)
    /// \return Number of words written to the buffer (normally nChunkSize),\n
    ///	    Transfer will stop if 0 is returned
    /// \note Each sample consists of two words (Left channel, right channel)\n
    ///	  Each word must be between GetRangeMin() and GetRangeMax()
    /// virtual unsigned GetChunk (usize *pBuffer, unsigned nChunkSize);

    unsafe fn GetNextChunk(&mut self) -> bool {
        let nChunkSize: usize = self.GetChunk(self.m_nChunkSize);
        if nChunkSize == 0 {
            return false;
        }

        let nTransferLength: usize = nChunkSize * size_of::<u32>();
        // assert!(nTransferLength <= TXFR_LEN_MAX_LITE);

        // assert!(self.m_pControlBlock[self.m_nNextBuffer] != ptr::null_mut());
        // (*self.m_pControlBlock[self.m_nNextBuffer]).nTransferLength = nTransferLength as u32;
        self.dma_control_block.TXFR_LEN.write(nTransferLength as u32);

        /*
        CleanAndInvalidateDataCacheRange(
            self.m_pDMABufferPADDR[self.m_nNextBuffer],
            nTransferLength,
        );
        CleanAndInvalidateDataCacheRange(
            self.m_pControlBlock[self.m_nNextBuffer] as usize,
            size_of::<TDMAControlBlock>() as usize,
        );
        */

        self.m_nNextBuffer ^= 1;

        return true;
    }

    unsafe fn GetChunk(&mut self, nChunkSize: usize) -> usize {
        let pBuffer: *mut u32 = self.m_pDMABuffer[self.m_nNextBuffer] as *mut u32;
        assert!(pBuffer != ptr::null_mut());
        assert!(nChunkSize > 0);
        assert!((nChunkSize & 1) == 0);

        let mut nResult = 0;

        if self.m_nSamples == 0 {
            return nResult;
        }

        assert!(self.m_pSoundData != ptr::null());
        assert!(self.m_nChannels == 1 || self.m_nChannels == 2);
        assert!(self.m_nBitsPerSample == 8 || self.m_nBitsPerSample == 16);

        let mut nSample = 0;
        while nSample < nChunkSize {
            // two channels
            let mut nValue = (*self.m_pSoundData) as u32;
            self.m_pSoundData = self.m_pSoundData.offset(1);
            
            /*
            if self.m_nBitsPerSample > 8 {
                nValue |= (*self.m_pSoundData as usize) << 8;
                self.m_pSoundData = self.m_pSoundData.offset(1);
                nValue = (nValue + 0x8000) & 0xFFFF; // signed -> unsigned (16 bit)
            }
            
            if self.m_nBitsPerSample >= 12 {
                nValue >>= self.m_nBitsPerSample - 12;
            } else {
                nValue <<= 12 - self.m_nBitsPerSample;
            }*/
            nValue <<= 12 - self.m_nBitsPerSample;

            *pBuffer.offset(nSample as isize) = nValue as u32;
            nSample += 1;

            /*
            if self.m_nChannels == 2 {
                nValue = (*self.m_pSoundData) as usize;
                self.m_pSoundData = self.m_pSoundData.offset(1);
                if self.m_nBitsPerSample > 8 {
                    nValue |= (*self.m_pSoundData as usize) << 8;
                    self.m_pSoundData = self.m_pSoundData.offset(1);
                    nValue = (nValue + 0x8000) & 0xFFFF; // signed -> unsigned (16 bit)
                }

                if self.m_nBitsPerSample >= 12 {
                    nValue >>= self.m_nBitsPerSample - 12;
                } else {
                    nValue <<= 12 - self.m_nBitsPerSample;
                }
            }
            */

            *pBuffer.offset(nSample as isize) = nValue as u32;
            nSample += 1;

            nResult += 2;

            self.m_nSamples -= 1;
            if self.m_nSamples == 0 {
                break;
            }
        }

        return nResult;
    }

    unsafe fn StopPWM(&mut self) {
        self.pwm_output.stop();
        /*
        write32(ARM_PWM_DMAC, 0);
        write32(ARM_PWM_CTL, 0); // disable PWM channel 0 and 1

        delay_us(2000);
        // stop gpio clock
        // not implement
        delay_us(2000);
        */
    }

    unsafe fn InterruptHandler(&mut self) {
        /*
        assert!(self.m_State != PWMSoundIdle);
        assert!(self.m_nDMAChannel <= DMA_CHANNEL_MAX);

        // #ifndef NDEBUG
        let nIntStatus = read32(ARM_DMA_INT_STATUS);
        // #endif
        let nIntMask = 1 << self.m_nDMAChannel;
        assert!(nIntStatus & nIntMask != 0);
        write32(ARM_DMA_INT_STATUS, nIntMask as u32);

        let nCS = read32(ARM_DMACHAN_CS(self.m_nDMAChannel));
        assert!(nCS & CS_INT as u32 != 0);
        write32(ARM_DMACHAN_CS(self.m_nDMAChannel), nCS); // reset CS_INT

        if nCS & CS_ERROR as u32 != 0 {
            self.m_State = PWMSoundError;
            return;
        }

        // m_SpinLock.Acquire ();

        if self.m_State == PWMSoundRunning && !self.GetNextChunk()
            || (self.m_State == PWMSoundCancelled)
        {
            write32(ARM_DMACHAN_NEXTCONBK(self.m_nDMAChannel), 0);
            // avoid clicks
            write32(
                ARM_PWM_CTL,
                read32(ARM_PWM_CTL) | ARM_PWM_CTL_RPTL1 as u32 | ARM_PWM_CTL_RPTL2 as u32,
            );
            self.m_State = PWMSoundTerminating;
        } else if self.m_State == PWMSoundTerminating {
            self.m_State = PWMSoundIdle;
        }
        */
        // m_SpinLock.Release ();
    }

    unsafe fn InterruptStub(pParam: &mut PWMSoundDevice) {
        pParam.InterruptHandler();
    }
}
