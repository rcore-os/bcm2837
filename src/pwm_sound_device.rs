use crate::consts::*;
use crate::util::*;
use crate::timer::*;
use alloc::vec::*;
use core::mem::size_of;
use core::ptr;
use core::convert::TryFrom;
use core::cmp::PartialEq;

// macro_rules! ARM_DMACHAN_TI { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x08) }
// macro_rules! ARM_DMACHAN_SOURCE_AD { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x0C) }
// macro_rules! ARM_DMACHAN_DEST_AD { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x10) }
// macro_rules! ARM_DMACHAN_TXFR_LEN { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x14) }
// macro_rules! ARM_DMACHAN_STRIDE { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x18) }
// macro_rules! ARM_DMACHAN_DEBUG { ($( $chan:expr ),*)	=>	(ARM_DMA_BASE + (($chan) * 0x100) + 0x20) }

fn CleanAndInvalidateDataCacheRange(nAddress: usize, nLength: usize) {

}

fn ARM_DMACHAN_CONBLK_AD(chan: usize) -> usize  {
    ARM_DMA_BASE + (chan * 0x100) + 0x04
}
fn BUS_ADDRESS(addr: usize) -> usize {
    (((addr) & !0xC0000000) | GPU_MEM_BASE)
}

fn ARM_DMACHAN_CS(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) *0x100) + 0x00
}
fn ARM_DMACHAN_NEXTCONBK(chan: usize) -> usize {
    ARM_DMA_BASE + ((chan) * 0x100) + 0x1C
}

enum TPWMSoundState
{
    PWMSoundIdle = 0,
    PWMSoundRunning = 1,
    PWMSoundCancelled = 2,
    PWMSoundTerminating = 3,
    PWMSoundError = 4,
    PWMSoundUnknown = 5
}

impl PartialEq for TPWMSoundState {
    fn eq(&self, other: &TPWMSoundState) -> bool {
        return *self == *other;
    }
}

enum TDREQ
{
    DREQSourceNone	 = 0,
    DREQSourcePCMTX	 = 2,
    DREQSourcePCMRX	 = 3,
    DREQSourcePWM	 = 5,
    DREQSourceSPITX	 = 6,
    DREQSourceSPIRX	 = 7,
    DREQSourceEMMC	 = 11,
    DREQSourceUARTTX = 12,
    DREQSourceUARTRX = 14
}

struct TDMAControlBlock
{
    nTransferInformation: u32,
    nSourceAddress: u32,
    nDestinationAddress: u32,
    nTransferLength: u32,
    n2DModeStride: u32,
    nNextControlBlockAddress: u32,
    nReserved: [u32; 2]
}

pub struct PWMSoundDevice {
    m_nChunkSize: usize,
    m_nRange: usize,


    m_State: TPWMSoundState,

    m_bIRQConnected: bool,

    m_nDMAChannel: usize,
    m_pDMABuffer: [Vec<u32>; 2],
    m_pControlBlockBuffer: [Vec<u8>; 2],
    m_pControlBlock: [*mut TDMAControlBlock; 2],

    m_nNextBuffer: usize,			// 0 or 1

    m_pSoundData: *const u8,
    m_nSamples: usize,
    m_nChannels: usize,
    m_nBitsPerSample: usize

    /*
    CSpinLock m_SpinLock;

    CGPIOPin   m_Audio1;
    CGPIOPin   m_Audio2;
    CGPIOClock m_Clock;
    */
}

fn allocateDMAChannel(nChannel: usize) -> usize {
    if (nChannel & !DMA_CHANNEL__MASK) == 0 {
        // explicit channel allocation
        return nChannel;
    } else {
        // arbitrary channel allocation
        let i = if nChannel == DMA_CHANNEL_NORMAL { 6 } else { DMA_CHANNEL_MAX };
        return i;
    }

}

impl PWMSoundDevice {
    unsafe fn SetupDMAControlBlock(&mut self, nID: usize) {
        assert!(nID <= 1);

        self.m_pDMABuffer[nID] = Vec::<u32>::new();
        self.m_pDMABuffer[nID].resize(self.m_nChunkSize, 0);
        // assert!(self.m_pDMABuffer[nID] != 0);

        self.m_pControlBlockBuffer[nID] = Vec::<u8>::new();
        self.m_pControlBlockBuffer[nID].resize(size_of::<TDMAControlBlock>() + 31, 0);
        // assert!(self.m_pControlBlockBuffer[nID] != 0);
        self.m_pControlBlock[nID] = ((self.m_pControlBlockBuffer[nID].as_ptr().offset(31) as usize) & !(31usize)) as *mut TDMAControlBlock;

        (*self.m_pControlBlock[nID]).nTransferInformation = (((TDREQ::DREQSourcePWM as usize) << TI_PERMAP_SHIFT)
        | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
        | TI_SRC_WIDTH
        | TI_SRC_INC
        | TI_DEST_DREQ
        | TI_WAIT_RESP
        | TI_INTEN) as u32;
        (*self.m_pControlBlock[nID]).nSourceAddress           = BUS_ADDRESS(self.m_pDMABuffer[nID].as_ptr() as usize) as u32;
        (*self.m_pControlBlock[nID]).nDestinationAddress      = ((ARM_PWM_FIF1 & 0xFFFFFF) + GPU_IO_BASE) as u32;
        (*self.m_pControlBlock[nID]).n2DModeStride            = 0;
        (*self.m_pControlBlock[nID]).nReserved[0]	       = 0;
        (*self.m_pControlBlock[nID]).nReserved[1]	       = 0;
    }



    /// \param nSampleRate	sample rate in Hz
    /// \param nChunkSize	twice the number of samples (words) to be handled\n
    ///			with one call to GetChunk() (one word per stereo channel)
    /// default: nSampleRate: 44100, nChunkSize: 2048
    pub fn new(nSampleRate : usize, nChunkSize : usize) -> PWMSoundDevice {
        PWMSoundDevice {
            m_nChunkSize: (nChunkSize),
            m_nRange: ((CLOCK_FREQ / CLOCK_DIVIDER + nSampleRate/2) / nSampleRate),
            // m_Audio1 (GPIOPinAudioLeft, GPIOModeAlternateFunction0),
            // m_Audio2 (GPIOPinAudioRight, GPIOModeAlternateFunction0),
            // m_Clock (GPIOClockPWM, GPIOClockSourcePLLD),
            m_bIRQConnected: (false),
            m_State: TPWMSoundState::PWMSoundIdle,
            m_nDMAChannel: allocateDMAChannel (DMA_CHANNEL_LITE),
            m_pDMABuffer: [Vec::<u32>::new(), Vec::<u32>::new()],
            m_pControlBlockBuffer: [Vec::<u8>::new(), Vec::<u8>::new()],
            m_pControlBlock: [ptr::null_mut(); 2],
            m_nNextBuffer: 0,
            m_pSoundData: ptr::null(),
            m_nSamples: 0,
            m_nChannels: 0,
            m_nBitsPerSample: 0
        }
    }

    pub fn init(&mut self) {
        unsafe {
            self.SetupDMAControlBlock(0);
            self.SetupDMAControlBlock(1);
            (*self.m_pControlBlock[0]).nNextControlBlockAddress = BUS_ADDRESS(self.m_pControlBlock[1] as usize) as u32;
            (*self.m_pControlBlock[1]).nNextControlBlockAddress = BUS_ADDRESS(self.m_pControlBlock[0] as usize) as u32;

            // start clock and PWM device
            self.RunPWM();

            // enable and reset DMA channel
            // PeripheralEntry();

            // assert! (self.m_nDMAChannel <= DMA_CHANNEL_MAX);
            write32(ARM_DMA_ENABLE, read32(ARM_DMA_ENABLE) | (1 << self.m_nDMAChannel));
            delay_us(1000);

            write32(ARM_DMACHAN_CS(self.m_nDMAChannel), CS_RESET as u32);
            while read32(ARM_DMACHAN_CS(self.m_nDMAChannel)) & CS_RESET as u32 != 0
                {
                    // do nothing
                }

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

    unsafe fn Start(&mut self) -> bool {
        assert!(self.m_State == TPWMSoundState::PWMSoundIdle);

        // fill buffer 0
        self.m_nNextBuffer = 0;

        if !self.GetNextChunk() {
            return false;
        }

        self.m_State = TPWMSoundState::PWMSoundRunning;

        // connect IRQ
        assert!(self.m_nDMAChannel <= DMA_CHANNEL_MAX);

        if !self.m_bIRQConnected {
            // assert!(self.m_pInterruptSystem != 0);
            // self.m_pInterruptSystem->ConnectIRQ (ARM_IRQ_DMA0+m_nDMAChannel, InterruptStub, this);
            self.m_bIRQConnected = true;
        }

        // enable PWM DMA operation

        write32 (ARM_PWM_DMAC,   (ARM_PWM_DMAC_ENAB
            | (7 << ARM_PWM_DMAC_PANIC__SHIFT)
            | (7 << ARM_PWM_DMAC_DREQ__SHIFT)) as u32);

        // switched this on when playback stops to avoid clicks, switch it off here
        write32 (ARM_PWM_CTL, read32 (ARM_PWM_CTL) & !(ARM_PWM_CTL_RPTL1 | ARM_PWM_CTL_RPTL2) as u32);


        // start DMA

        assert!((read32 (ARM_DMACHAN_CS (self.m_nDMAChannel)) & CS_INT as u32) == 0);
        assert!((read32 (ARM_DMA_INT_STATUS) & (1 << self.m_nDMAChannel)) == 0);

        assert!(self.m_pControlBlock[0] != ptr::null_mut());
        write32 (ARM_DMACHAN_CONBLK_AD(self.m_nDMAChannel), BUS_ADDRESS (self.m_pControlBlock[0] as usize) as u32);


        write32 (ARM_DMACHAN_CS (self.m_nDMAChannel),   (CS_WAIT_FOR_OUTSTANDING_WRITES
            | (DEFAULT_PANIC_PRIORITY << CS_PANIC_PRIORITY_SHIFT)
            | (DEFAULT_PRIORITY << CS_PRIORITY_SHIFT)
            | CS_ACTIVE) as u32);

        // fill buffer 1
        if !self.GetNextChunk() {
            // m_SpinLock.Acquire ();

            if self.m_State == TPWMSoundState::PWMSoundRunning {
                write32 (ARM_DMACHAN_NEXTCONBK (self.m_nDMAChannel), 0);

                self.m_State = TPWMSoundState::PWMSoundTerminating;
            }

            // m_SpinLock.Release ();
        }

        return true;
    }

    fn Cancel(&mut self) {
        // m_SpinLock.Acquire ();
        if (self.m_State == TPWMSoundState::PWMSoundRunning) {
            self.m_State = TPWMSoundState::PWMSoundCancelled;
        }

        // m_SpinLock.Release ();
    }


    fn IsActive(&self) -> bool {
        if self.m_State != TPWMSoundState::PWMSoundIdle {
            return true;
        } else {
            return false;
        }
    }


    pub fn Playback(&mut self, pSoundData: *const u8, nSamples: usize, nChannels: usize, nBitsPerSample: usize) {
        assert!(!self.IsActive ());
        assert!(pSoundData != ptr::null());
        assert!(nChannels == 1 || nChannels == 2);
        assert!(nBitsPerSample == 8 || nBitsPerSample == 16);

        self.m_pSoundData = pSoundData;
        self.m_nSamples	= nSamples;
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
        let nChunkSize : usize = self.GetChunk (self.m_nChunkSize);
        if nChunkSize == 0 {
            return false;
        }

        let nTransferLength : usize = nChunkSize * size_of::<usize>();
        assert!(nTransferLength <= TXFR_LEN_MAX_LITE);

        assert!(self.m_pControlBlock[self.m_nNextBuffer] != ptr::null_mut());
        (*self.m_pControlBlock[self.m_nNextBuffer]).nTransferLength = nTransferLength as u32;

        CleanAndInvalidateDataCacheRange(self.m_pDMABuffer[self.m_nNextBuffer].as_ptr() as usize, nTransferLength);
        CleanAndInvalidateDataCacheRange(self.m_pControlBlock[self.m_nNextBuffer] as usize, size_of::<TDMAControlBlock>() as usize);

        self.m_nNextBuffer ^= 1;

        return true;
    }

    unsafe fn GetChunk(&mut self, nChunkSize: usize) -> usize {
        let pBuffer: *mut u32 = self.m_pDMABuffer[self.m_nNextBuffer].as_mut_ptr();
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
        while nSample < nChunkSize / 2 {		// 2 channels on outpu
            let mut nValue : usize = *self.m_pSoundData as usize;
            self.m_pSoundData = self.m_pSoundData.offset(1);
            if (self.m_nBitsPerSample > 8) {
                nValue |= (*self.m_pSoundData as usize) << 8;
                self.m_pSoundData = self.m_pSoundData.offset(1);
                nValue = (nValue + 0x8000) & 0xFFFF;		// signed -> unsigned (16 bit)
            }

            if (self.m_nBitsPerSample >= 12) {
                nValue >>= self.m_nBitsPerSample - 12;
            }
            else {
                nValue <<= 12 - self.m_nBitsPerSample;
            }

            *pBuffer.offset(nSample as isize) = nValue as u32;
            nSample += 1;

            if (self.m_nChannels == 2)
            {
                nValue = (*self.m_pSoundData) as usize;
                self.m_pSoundData = self.m_pSoundData.offset(1);
                if self.m_nBitsPerSample > 8 {
                    nValue |= (*self.m_pSoundData as usize) << 8;
                    self.m_pSoundData = self.m_pSoundData.offset(1);
                    nValue = (nValue + 0x8000) & 0xFFFF;	// signed -> unsigned (16 bit)
                }

                if self.m_nBitsPerSample >= 12 {
                    nValue >>= self.m_nBitsPerSample - 12;
                }
                else {
                    nValue <<= 12 - self.m_nBitsPerSample;
                }
            }

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

    unsafe fn RunPWM(&mut self) {
        // start gpio clock
        // not implement

        delay_us(2000);

        assert!((1 << 8) <= self.m_nRange && self.m_nRange < (1 << 16));
        write32 (ARM_PWM_RNG1, self.m_nRange as u32);
        write32 (ARM_PWM_RNG2, self.m_nRange as u32);

        write32 (ARM_PWM_CTL,   (ARM_PWM_CTL_PWEN1 | ARM_PWM_CTL_USEF1
            | ARM_PWM_CTL_PWEN2 | ARM_PWM_CTL_USEF2
            | ARM_PWM_CTL_CLRF1) as u32);

        delay_us(2000);
    }

    unsafe fn StopPWM(&mut self) {
        write32 (ARM_PWM_DMAC, 0);
        write32 (ARM_PWM_CTL, 0);			// disable PWM channel 0 and 1

        delay_us(2000);
        // stop gpio clock
        // not implement
        delay_us(2000);
    }

    unsafe fn InterruptHandler(&mut self) {
        assert!(self.m_State != TPWMSoundState::PWMSoundIdle);
        assert!(self.m_nDMAChannel <= DMA_CHANNEL_MAX);

        // #ifndef NDEBUG
        let nIntStatus = read32 (ARM_DMA_INT_STATUS);
        // #endif
        let nIntMask = 1 << self.m_nDMAChannel;
        assert!(nIntStatus & nIntMask != 0);
        write32 (ARM_DMA_INT_STATUS, nIntMask as u32);

        let nCS = read32 (ARM_DMACHAN_CS (self.m_nDMAChannel));
        assert!(nCS & CS_INT as u32 != 0);
        write32 (ARM_DMACHAN_CS (self.m_nDMAChannel), nCS);	// reset CS_INT

        if nCS & CS_ERROR as u32 != 0{
            self.m_State = TPWMSoundState::PWMSoundError;
            return;
        }

        // m_SpinLock.Acquire ();

        if (self.m_State == TPWMSoundState::PWMSoundRunning && !self.GetNextChunk()) || (self.m_State == TPWMSoundState::PWMSoundCancelled) {
            write32(ARM_DMACHAN_NEXTCONBK(self.m_nDMAChannel), 0);
            // avoid clicks
            write32(ARM_PWM_CTL, read32(ARM_PWM_CTL) | ARM_PWM_CTL_RPTL1 as u32| ARM_PWM_CTL_RPTL2 as u32);
            self.m_State = TPWMSoundState::PWMSoundTerminating;
        } else if (self.m_State == TPWMSoundState::PWMSoundTerminating) {
            self.m_State = TPWMSoundState::PWMSoundIdle;
        }

        // m_SpinLock.Release ();
    }

    unsafe fn InterruptStub(pParam: &mut PWMSoundDevice) {
        pParam.InterruptHandler();
    }

}
