use crate::consts::*;
use crate::util::*;
use crate::timer::*;
use alloc::vec::*;
use core::mem::size_of;
use core::ptr;
use core::convert::TryFrom;

// macro_rules! ARM_DMACHAN_CONBLK_AD { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x04) }
// macro_rules! ARM_DMACHAN_TI { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x08) }
// macro_rules! ARM_DMACHAN_SOURCE_AD { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x0C) }
// macro_rules! ARM_DMACHAN_DEST_AD { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x10) }
// macro_rules! ARM_DMACHAN_TXFR_LEN { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x14) }
// macro_rules! ARM_DMACHAN_STRIDE { ($( $chan:expr ),*) => (ARM_DMA_BASE + (($chan) * 0x100) + 0x18) }
// macro_rules! ARM_DMACHAN_NEXTCONBK { ($( $chan:expr ),*) =>	(ARM_DMA_BASE + (($chan) * 0x100) + 0x1C) }
// macro_rules! ARM_DMACHAN_DEBUG { ($( $chan:expr ),*)	=>	(ARM_DMA_BASE + (($chan) * 0x100) + 0x20) }

fn BUS_ADDRESS(addr: u32) -> u32 {
    (((addr) & !0xC0000000) | GPU_MEM_BASE)
}

fn ARM_DMACHAN_CS(chan: u32) -> u32 {
    ARM_DMA_BASE + ((chan) *0x100) + 0x00
}
enum TPWMSoundState
{
    PWMSoundIdle,
    PWMSoundRunning,
    PWMSoundCancelled,
    PWMSoundTerminating,
    PWMSoundError,
    PWMSoundUnknown
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
    m_nChunkSize: u32,
    m_nRange: u32,


    m_State: TPWMSoundState,

    m_bIRQConnected: bool,

    m_nDMAChannel: u32,
    m_pDMABuffer: [Vec<u32>; 2],
    m_pControlBlockBuffer: [Vec<u8>; 2],
    m_pControlBlock: [*mut TDMAControlBlock; 2],

    m_nNextBuffer: u32,			// 0 or 1

    /*
    CSpinLock m_SpinLock;

    CGPIOPin   m_Audio1;
    CGPIOPin   m_Audio2;
    CGPIOClock m_Clock;
    */
}

fn allocateDMAChannel(nChannel: u32) -> u32 {
    if (nChannel & !DMA_CHANNEL__MASK) == 0 {
        // explicit channel allocation
        return nChannel;
    } else {
        // arbitrary channel allocation
        let i = if nChannel == DMA_CHANNEL_NORMAL { 6u32 } else { DMA_CHANNEL_MAX };
        return i;
    }

}

impl PWMSoundDevice {
    unsafe fn SetupDMAControlBlock(&mut self, nID: usize) {
        assert!(nID <= 1);

        self.m_pDMABuffer[nID] = Vec::<u32>::new();
        self.m_pDMABuffer[nID].resize(to_usize(self.m_nChunkSize), 0);
        // assert!(self.m_pDMABuffer[nID] != 0);

        self.m_pControlBlockBuffer[nID] = Vec::<u8>::new();
        self.m_pControlBlockBuffer[nID].resize(size_of::<TDMAControlBlock>() + 31, 0);
        // assert!(self.m_pControlBlockBuffer[nID] != 0);
        self.m_pControlBlock[nID] = ((self.m_pControlBlockBuffer[nID].as_ptr().offset(31) as u32) & !31) as *mut TDMAControlBlock;;

        (*self.m_pControlBlock[nID]).nTransferInformation = ((TDREQ::DREQSourcePWM as u32) << TI_PERMAP_SHIFT)
        | (DEFAULT_BURST_LENGTH << TI_BURST_LENGTH_SHIFT)
        | TI_SRC_WIDTH
        | TI_SRC_INC
        | TI_DEST_DREQ
        | TI_WAIT_RESP
        | TI_INTEN;
        (*self.m_pControlBlock[nID]).nSourceAddress           = BUS_ADDRESS(self.m_pDMABuffer[nID].as_ptr() as u32);
        (*self.m_pControlBlock[nID]).nDestinationAddress      = (ARM_PWM_FIF1 & 0xFFFFFF) + GPU_IO_BASE;
        (*self.m_pControlBlock[nID]).n2DModeStride            = 0;
        (*self.m_pControlBlock[nID]).nReserved[0]	       = 0;
        (*self.m_pControlBlock[nID]).nReserved[1]	       = 0;
    }



    /// \param nSampleRate	sample rate in Hz
    /// \param nChunkSize	twice the number of samples (words) to be handled\n
    ///			with one call to GetChunk() (one word per stereo channel)
    /// default: nSampleRate: 44100, nChunkSize: 2048
    pub fn new(nSampleRate : u32, nChunkSize : u32) -> PWMSoundDevice {
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
            m_nNextBuffer: 0
        }
    }

    pub unsafe fn init(&mut self) {
        self.SetupDMAControlBlock (0);
        self.SetupDMAControlBlock (1);
        (*self.m_pControlBlock[0]).nNextControlBlockAddress = BUS_ADDRESS(self.m_pControlBlock[1] as u32);
        (*self.m_pControlBlock[1]).nNextControlBlockAddress = BUS_ADDRESS(self.m_pControlBlock[0] as u32);

        // start clock and PWM device
        self.RunPWM();

        // enable and reset DMA channel
        // PeripheralEntry();

        // assert! (self.m_nDMAChannel <= DMA_CHANNEL_MAX);
        write32 (ARM_DMA_ENABLE as *mut u32, read32 (ARM_DMA_ENABLE as *const u32) | (1 << self.m_nDMAChannel));
        delay(1000);

        write32 (ARM_DMACHAN_CS (self.m_nDMAChannel) as *mut u32, CS_RESET);
        while (read32 (ARM_DMACHAN_CS (self.m_nDMAChannel) as *const u32) & CS_RESET != 0)
            {
                // do nothing
            }

        // PeripheralExit ();

        // CDeviceNameService::Get ()->AddDevice ("sndpwm", this, FALSE);
    }


    /// \brief May overload this to provide the sound samples!
    /// \param pBuffer	buffer where the samples have to be placed
    /// \param nChunkSize	size of the buffer in words (same as given to constructor)
    /// \return Number of words written to the buffer (normally nChunkSize),\n
    ///	    Transfer will stop if 0 is returned
    /// \note Each sample consists of two words (Left channel, right channel)\n
    ///	  Each word must be between GetRangeMin() and GetRangeMax()
    /// virtual unsigned GetChunk (u32 *pBuffer, unsigned nChunkSize);

    fn GetNextChunk(&mut self) -> bool {
        true
    }

    fn RunPWM(&mut self) {
        panic!();
    }

    fn StopPWM(&mut self) {

    }

    fn InterruptHandler(&mut self) {

    }

    /*
    static fn InterruptStub(void *pParam);

    void SetupDMAControlBlock (unsigned nID);
    */

}
