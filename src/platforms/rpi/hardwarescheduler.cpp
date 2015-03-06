#include "hardwarescheduler.h"

#include <sys/mman.h> //for mmap
#include <sys/time.h> //for timespec
#include <time.h> //for timespec / nanosleep
#include <signal.h> //for sigaction
#include <unistd.h> //for lseek, read, etc.
#include <stdlib.h> //for exit
#include <cassert>
#include <fcntl.h> //for file opening
#include <errno.h> //for errno
#include <pthread.h> //for pthread_setschedparam
#include <chrono>

#include "primitiveiopin.h"
#include "outputevent.h"
#include "mitpi.h"
#include "schedulerbase.h"
#include "common/logging.h"
#include "platforms/auto/thisthreadsleep.h" //for SleepT 
#include "compileflags.h" //for RUNNING_IN_VM

#if MAX_RPI_PIN_ID < 32
    #define NUM_GPIO_WORDS 1
#else
    #define NUM_GPIO_WORDS 2
#endif

#define GPIO_BASE 0x20200000 //base address of the GPIO control registers.
#define GPIO_BASE_BUS 0x7E200000 //this is the physical bus address of the GPIO module. This is only used when other peripherals directly connected to the bus (like DMA) need to read/write the GPIOs
#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size
#define GPFSEL0   0x00000000 //gpio function select. There are 6 of these (32 bit registers)
#define GPFSEL1   0x00000004
#define GPFSEL2   0x00000008
#define GPFSEL3   0x0000000c
#define GPFSEL4   0x00000010
#define GPFSEL5   0x00000014
//bits 2-0 of GPFSEL0: set to 000 to make Pin 0 an output. 001 is an input. Other combinations represent alternate functions
//bits 3-5 are for pin 1.
//...
//bits 27-29 are for pin 9.
//GPFSEL1 repeats, but bits 2-0 are Pin 10, 27-29 are pin 19.
//...
#define GPSET0    0x0000001C //GPIO Pin Output Set. There are 2 of these (32 bit registers)
#define GPSET1    0x00000020
//writing a '1' to bit N of GPSET0 makes that pin HIGH.
//writing a '0' has no effect.
//GPSET0[0-31] maps to pins 0-31
//GPSET1[0-21] maps to pins 32-53
#define GPCLR0    0x00000028 //GPIO Pin Output Clear. There are 2 of these (32 bits each)
#define GPCLR1    0x0000002C
//GPCLR acts the same way as GPSET, but clears the pin instead.
#define GPLEV0    0x00000034 //GPIO Pin Level. There are 2 of these (32 bits each)

//physical addresses for the DMA peripherals, as found in the processor documentation:
#define DMA_BASE 0x20007000
#define DMACH(n) (0x100*(n))
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
//#define DMACH0   0x00000000
//#define DMACH1   0x00000100
//#define DMACH2   0x00000200
//#define DMACH3   0x00000300
//...
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ABORT (1<<30)
#define DMA_CS_DISDEBUG (1<<28) //DMA will not stop when debug signal is asserted
#define DMA_CS_PRIORITY(x) ((x)&0xf << 16) //higher priority DMA transfers are serviced first, it would appear
#define DMA_CS_PRIORITY_MAX DMA_CS_PRIORITY(15)
#define DMA_CS_PANIC_PRIORITY(x) ((x)&0xf << 20)
#define DMA_CS_PANIC_PRIORITY_MAX DMA_CS_PANIC_PRIORITY(15)
#define DMA_CS_END (1<<1)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_NO_WIDE_BURSTS (1<<26)
#define DMA_CB_TI_PERMAP_NONE (0<<16)
#define DMA_CB_TI_PERMAP_DSI  (1<<16)
//... (more found on page 61 of BCM2835 pdf
#define DMA_CB_TI_PERMAP_PWM  (5<<16)
//...
#define DMA_CB_TI_SRC_DREQ    (1<<10)
#define DMA_CB_TI_SRC_INC     (1<<8)
#define DMA_CB_TI_DEST_DREQ   (1<<6)
#define DMA_CB_TI_DEST_INC    (1<<4)
#define DMA_CB_TI_TDMODE      (1<<1)


//https://dev.openwrt.org/browser/trunk/target/linux/brcm2708/patches-3.10/0070-bcm2708_fb-DMA-acceleration-for-fb_copyarea.patch?rev=39770
// says that YLENGTH should actually be written as # of copies *MINUS ONE*
#define DMA_CB_TXFR_LEN_YLENGTH(y) (((y-1)&0x4fff) << 16)
#define DMA_CB_TXFR_LEN_XLENGTH(x) ((x)&0xffff)
#define DMA_CB_TXFR_YLENGTH_MASK (0x4fff << 16)
#define DMA_CB_STRIDE_D_STRIDE(x)  (((x)&0xffff) << 16)
#define DMA_CB_STRIDE_S_STRIDE(x)  ((x)&0xffff)


//Dma Control Blocks must be located at addresses that are multiples of 32 bytes
#define DMA_CONTROL_BLOCK_ALIGNMENT 32 

#define PWM_BASE 0x2020C000
#define PWM_BASE_BUS 0x7E20C000
#define PWM_CTL  0x00000000 //control register
#define PWM_STA  0x00000004 //status register
#define PWM_DMAC 0x00000008 //DMA control register
#define PWM_RNG1 0x00000010 //channel 1 range register (# output bits to use per sample)
#define PWM_DAT1 0x00000014 //channel 1 data
#define PWM_FIF1 0x00000018 //channel 1 fifo (write to this register to queue an output)
#define PWM_RNG2 0x00000020 //channel 2 range register
#define PWM_DAT2 0x00000024 //channel 2 data

#define PWM_CTL_USEFIFO2 (1<<13)
#define PWM_CTL_REPEATEMPTY2 (1<<10)
#define PWM_CTL_ENABLE2 (1<<8)
#define PWM_CTL_CLRFIFO (1<<6)
#define PWM_CTL_USEFIFO1 (1<<5)
#define PWM_CTL_REPEATEMPTY1 (1<<2)
#define PWM_CTL_ENABLE1 (1<<0)

#define PWM_STA_BUSERR (1<<8)
#define PWM_STA_GAPERRS (0xf << 4)
#define PWM_STA_FIFOREADERR (1<<3)
#define PWM_STA_FIFOWRITEERR (1<<2)
#define PWM_STA_ERRS PWM_STA_BUSERR | PWM_STA_GAPERRS | PWM_STA_FIFOREADERR | PWM_STA_FIFOWRITEERR

#define PWM_DMAC_EN (1<<31)
#define PWM_DMAC_PANIC(P) (((P)&0xff)<<8)
#define PWM_DMAC_DREQ(D) (((D)&0xff)<<0)

//The following is undocumented :( Taken from http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
#define CLOCK_BASE 0x20101000
#define CM_PWMCTL 0xa0
#define CM_PWMDIV 0xa4
//each write to CM_PWMTL and CM_PWMDIV requires the password to be written:
#define CM_PWMCTL_PASSWD 0x5a000000
#define CM_PWMDIV_PASSWD 0x5a000000
//MASH is used to achieve fractional clock dividers by introducing artificial jitter.
//if you want constant frequency (even if it may not be at 100% CORRECT frequency), use MASH0
//if clock divisor is integral, then there's no need to use MASH, and anything above MASH1 can introduce jitter.
#define CM_PWMCTL_MASH(x) (((x)&0x3) << 9)
#define CM_PWMCTL_MASH0 CM_PWMTRL_MASH(0)
#define CM_PWMCTL_MASH1 CM_PWMTRL_MASH(1)
#define CM_PWMCTL_MASH2 CM_PWMTRL_MASH(2)
#define CM_PWMCTL_MASH3 CM_PWMTRL_MASH(3)
#define CM_PWMCTL_FLIP (1<<8) //use to inverse clock polarity
#define CM_PWMCTL_BUSY (1<<7) //read-only flag that indicates clock generator is running.
#define CM_PWMCTL_KILL (1<<5) //write a 1 to stop & reset clock generator. USED FOR DEBUG ONLY
#define CM_PWMCTL_ENAB (1<<4) //gracefully stop/start clock generator. BUSY flag will go low once clock is off.
#define CM_PWMCTL_SRC(x) ((x)&0xf) //clock source. 0=gnd. 1=oscillator. 2-3=debug. 4=PLLA per. 5=PLLC per. 6=PLLD per. 7=HDMI aux. 8-15=GND
#define CM_PWMCTL_SRC_OSC CM_PWMCTL_SRC(1)
#define CM_PWMCTL_SRC_PLLA CM_PWMCTL_SRC(4)
#define CM_PWMCTL_SRC_PLLC CM_PWMCTL_SRC(5)
#define CM_PWMCTL_SRC_PLLD CM_PWMCTL_SRC(6)

//max clock divisor is 4095
#define CM_PWMDIV_DIVI(x) (((x)&0xfff) << 12)
#define CM_PWMDIV_DIVF(x) ((x)&0xfff)


namespace plat {
namespace rpi {

//initialize static variables:
DmaChannelHeader *UnwrappedHardwareScheduler::dmaHeader(0);

struct DmaChannelHeader {
    //Note: dma channels 7-15 are 'LITE' dma engines (or is it 8-15?), with reduced performance & functionality.
    //Note: only CS, CONBLK_AD and DEBUG are directly writeable
    volatile uint32_t CS; //Control and Status
        //31    RESET; set to 1 to reset DMA
        //30    ABORT; set to 1 to abort current DMA control block (next one will be loaded & continue)
        //29    DISDEBUG; set to 1 and DMA won't be paused when debug signal is sent
        //28    WAIT_FOR_OUTSTANDING_WRITES(0x10000000); set to 1 and DMA will wait until peripheral says all writes have gone through before loading next CB
        //24-74 reserved
        //20-23 PANIC_PRIORITY; 0 is lowest priority
        //16-19 PRIORITY; bus scheduling priority. 0 is lowest
        //9-15  reserved
        //8     ERROR; read as 1 when error is encountered. error can be found in DEBUG register.
        //7     reserved
        //6     WAITING_FOR_OUTSTANDING_WRITES; read as 1 when waiting for outstanding writes
        //5     DREQ_STOPS_DMA(0x20); read as 1 if DREQ is currently preventing DMA
        //4     PAUSED(0x10); read as 1 if DMA is paused
        //3     DREQ; copy of the data request signal from the peripheral, if DREQ is enabled. reads as 1 if data is being requested (or PERMAP=0), else 0
        //2     INT; set when current CB ends and its INTEN=1. Write a 1 to this register to clear it
        //1     END; set when the transfer defined by current CB is complete. Write 1 to clear.
        //0     ACTIVE(0x01); write 1 to activate DMA (load the CB before hand)
    volatile uint32_t CONBLK_AD; //Control Block Address
    volatile uint32_t TI; //transfer information; see DmaControlBlock.TI for description
    volatile uint32_t SOURCE_AD; //Source address
    volatile uint32_t DEST_AD; //Destination address
    volatile uint32_t TXFR_LEN; //transfer length. ONLY THE LOWER 16 BITS ARE USED IN LITE DMA ENGINES
    volatile uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1. NOT AVAILABLE IN LITE DMA ENGINES
    volatile uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    volatile uint32_t DEBUG; //controls debug settings
        //29-31 unused
        //28    LITE (0x10000000)
        //25-27 VERSION
        //16-24 DMA_STATE (dma engine state machine)
        //8-15  DMA_ID    (AXI bus id)
        //4-7   OUTSTANDING_WRITES
        //3     unused
        //2     READ_ERROR
        //1     WRITE_ERROR
        //0     READ_LAST_NOT_SET_ERROR
};

struct DmaControlBlock {
    volatile uint32_t TI; //transfer information
        //31:27 unused
        //26    NO_WIDE_BURSTS
        //21:25 WAITS; number of cycles to wait between each DMA read/write operation
        //16:20 PERMAP(0x000Y0000); peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
        //12:15 BURST_LENGTH
        //11    SRC_IGNORE; set to 1 to not perform reads. Used to manually fill caches
        //10    SRC_DREQ; set to 1 to have the DREQ from PERMAP gate requests.
        //9     SRC_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //8     SRC_INC;   set to 1 to automatically increment the source address after each read (you'll want this if you're copying a range of memory)
        //7     DEST_IGNORE; set to 1 to not perform writes.
        //6     DEST_DREQ; set to 1 to have the DREQ from PERMAP gate *writes*
        //5     DEST_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //4     DEST_INC;   set to 1 to automatically increment the destination address after each read (Tyou'll want this if you're copying a range of memory)
        //3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
        //2     unused (0)
        //1     TDMODE; set to 1 to enable 2D mode
        //0     INTEN;  set to 1 to generate an interrupt upon completion
    volatile uint32_t SOURCE_AD; //Source address
    volatile uint32_t DEST_AD; //Destination address
    volatile uint32_t TXFR_LEN; //transfer length.
        //in 2D mode, TXFR_LEN is separated into two half-words to indicate Y transfers of length X, and STRIDE is added to the src/dest address after each transfer of length X.
        //30:31 unused
        //16-29 YLENGTH
        //0-15  XLENGTH
    volatile uint32_t STRIDE; //2D Mode Stride (amount to increment/decrement src/dest after each 1d copy when in 2d mode). Only used if TI.TDMODE = 1
        //16-31 D_STRIDE; signed (2's complement) byte increment/decrement to apply to destination addr after each XLENGTH transfer
        //0-15  S_STRIDE; signed (2's complement) byte increment/decrement to apply to source addr after each XLENGTH transfer
    volatile uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
};

struct PwmHeader {
    volatile uint32_t CTL;  // 0x00000000 //control register
        //16-31 reserved
        //15 MSEN2 (0: PWM algorithm, 1:M/S transmission used)
        //14 reserved
        //13 USEF2 (0: data register is used for transmission, 1: FIFO is used for transmission)
        //12 POLA2 (0: 0=low, 1=high. 1: 0=high, 1=low (inversion))
        //11 SBIT2; defines the state of the output when no transmission is in place
        //10 RPTL2; 0: transmission interrupts when FIFO is empty. 1: last data in FIFO is retransmitted when FIFO is empty
        //9  MODE2; 0: PWM mode. 1: serializer mode
        //8  PWMEN2; 0: channel is disabled. 1: channel is enabled
        //7  MSEN1;
        //6  CLRF1; writing a 1 to this bit clears the channel 1 (and channel 2?) fifo
        //5  USEF1;
        //4  POLA1;
        //3  SBIT1;
        //2  RPTL1;
        //1  MODE1;
        //0  PWMEN1;   
    volatile uint32_t STA;  // 0x00000004 //status register
        //13-31 reserved
        //9-12 STA1-4; indicates whether each channel is transmitting
        //8    BERR; Bus Error Flag. Write 1 to clear
        //4-7  GAPO1-4; Gap Occured Flag. Write 1 to clear
        //3    RERR1; Fifo Read Error Flag (attempt to read empty fifo). Write 1 to clear
        //2    WERR1; Fifo Write Error Flag (attempt to write to full fifo). Write 1 to clear
        //1    EMPT1; Reads as 1 if fifo is empty
        //0    FULL1; Reads as 1 if fifo is full
    volatile uint32_t DMAC; // 0x00000008 //DMA control register
        //31   ENAB; set to 1 to enable DMA
        //16-30 reserved
        //8-15 PANIC; DMA threshold for panic signal
        //0-7  DREQ;  DMA threshold for DREQ signal
    uint32_t _padding1;
    volatile uint32_t RNG1; // 0x00000010 //channel 1 range register (# output bits to use per sample)
        //0-31 PWM_RNGi; #of bits to modulate PWM. (eg if PWM_RNGi=1024, then each 32-bit sample sent through the FIFO will be modulated into 1024 bits.)
    volatile uint32_t DAT1; // 0x00000014 //channel 1 data
        //0-31 PWM_DATi; Stores the 32-bit data to be sent to the PWM controller ONLY WHEN USEFi=0 (FIFO is disabled)
    volatile uint32_t FIF1; // 0x00000018 //channel 1 fifo (write to this register to queue an output)
        //writing to this register will queue a sample into the fifo. If 2 channels are enabled, then each even sample (0-indexed) is sent to channel 1, and odd samples are sent to channel 2. WRITE-ONLY
    uint32_t _padding2;
    volatile uint32_t RNG2; // 0x00000020 //channel 2 range register
    volatile uint32_t DAT2; // 0x00000024 //channel 2 data
        //0-31 PWM_DATi; Stores the 32-bit data to be sent to the PWM controller ONLY WHEN USEFi=1 (FIFO is enabled). TODO: Typo???
};

void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
    //set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
    //eg if x = 0b11001100
    //  writeBitmasked(&x, 0b00000110, 0b11110011),
    //  then x now = 0b11001110
    uint32_t cur = *dest;
    uint32_t revised = (cur & (~mask)) | (value & mask);
    *dest = revised;
    *dest = revised; //best to be safe when crossing memory boundaries
}

//custom structure used for storing the GPIO buffer.
//These BufferFrame's are DMA'd into the GPIO memory, potentially using the DmaEngine's Stride facility
struct GpioBufferFrame {
    uint32_t gpset[NUM_GPIO_WORDS];
    uint32_t gpclr[NUM_GPIO_WORDS];
    inline uint32_t* gpsetForPin(int pin) {
        assert(0 <= pin && pin < NUM_GPIO_WORDS*32);
        int idx = NUM_GPIO_WORDS == 1 ? 0 : (pin>31);
        return &gpset[idx];
    }
    inline uint32_t* gpclrForPin(int pin) {
        assert(0 <= pin && pin < NUM_GPIO_WORDS*32);
        int idx = NUM_GPIO_WORDS == 1 ? 0 : (pin>31);
        return &gpclr[idx];
    }
    inline void writeGpSet(int pin) {
        int shift = NUM_GPIO_WORDS == 1 ? pin : pin%32;
        *gpsetForPin(pin) |= (1<<shift);
    }
    inline void writeGpSet(int pin, bool val) {
        int shift = NUM_GPIO_WORDS == 1 ? pin : pin%32;
        writeBitmasked(gpsetForPin(pin), 1<<shift, val<<shift);
    }
    inline void writeGpClr(int pin) {
        int shift = NUM_GPIO_WORDS == 1 ? pin : pin%32;
        *gpclrForPin(pin) |= (1<<shift);
    }
    inline void writeGpClr(int pin, bool val) {
        int shift = NUM_GPIO_WORDS == 1 ? pin : pin%32;
        writeBitmasked(gpclrForPin(pin), 1<<shift, val<<shift);
    }
};

size_t ceilToPage(size_t size) {
    //round up to nearest page-size multiple
    if (size & (PAGE_SIZE-1)) {
        size += PAGE_SIZE - (size & (PAGE_SIZE-1));
    }
    return size;
}

uintptr_t physToUncached(uintptr_t phys) {
    return phys | 0x40000000; //bus address of the ram is 0x40000000. With this binary-or, writes to the returned address will bypass the CPU (L1) cache, but not the L2 cache. 0xc0000000 should be the base address if L2 must also be bypassed. However, the DMA engine is aware of L2 cache - just not the L1 cache (source: http://en.wikibooks.org/wiki/Aros/Platforms/Arm_Raspberry_Pi_support#Framebuffer )
}

//allocate some memory and lock it so that its physical address will never change
uint8_t* makeLockedMem(size_t size) {
    size = ceilToPage(size);
    void *mem = mmap(
        nullptr,   //let kernel place memory where it wants
        size,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1, // File descriptor
    0); //no offset into file (file doesn't exist).
    if (mem == MAP_FAILED) {
        LOGE("rpi/hardwarescheduler.cpp: makeLockedMem failed\n");
        exit(1);
    }
    memset(mem, 0, size); //simultaneously zero the pages and force them into memory
    mlock(mem, size);
    return (uint8_t*)mem;
}

//free memory allocated with makeLockedMem
void freeLockedMem(void* mem, size_t size) {
    size = ceilToPage(size);
    munlock(mem, size);
    munmap(mem, size);
}

UnwrappedHardwareScheduler::DmaMem::DmaMem(const UnwrappedHardwareScheduler &dmaSched, std::size_t numBytes) {
    this->numPages = (numBytes+PAGE_SIZE-1) / PAGE_SIZE; //round up to nearest page so eg 4097 bytes is 2 pages.
    this->virtL1 = makeLockedMem(numBytes);
    this->virtL2Coherent = dmaSched.makeUncachedMemView(virtL1, numBytes);
    this->pageMap = new uintptr_t[numPages];
    for (unsigned int i=0; i<numPages; ++i) {
        pageMap[i] = dmaSched.virtToPhys((char*)virtL1 + i*PAGE_SIZE);
    }
}

uintptr_t UnwrappedHardwareScheduler::DmaMem::physAddrAtByteOffset(std::size_t bytes) const {
    return pageMap[bytes/PAGE_SIZE] + bytes % PAGE_SIZE;
}

uintptr_t UnwrappedHardwareScheduler::DmaMem::virtToPhys(void *virt) const {
    if ((uintptr_t)virt - (uintptr_t)virtL1 < numPages*PAGE_SIZE) {
        return physAddrAtByteOffset((uintptr_t)virt - (uintptr_t)virtL1);
    } else if ((uintptr_t)virt - (uintptr_t)virtL2Coherent < numPages*PAGE_SIZE) {
        return physAddrAtByteOffset((uintptr_t)virt - (uintptr_t)virtL2Coherent);
    } else {
        assert(false); //pointer is not owned by this object!
        return 0;
    }
}


UnwrappedHardwareScheduler::UnwrappedHardwareScheduler() 
  : _lastTimeAtFrame0(0)
  , _lastDmaSyncedTime(std::chrono::seconds(0)) {
    dmaCh = 5;
    SchedulerBase::registerExitHandler(&cleanup, SCHED_IO_EXIT_LEVEL);
    makeMaps();
    initSrcAndControlBlocks();
    initPwm();
    initDma();
}

void UnwrappedHardwareScheduler::cleanup() {
    LOG("platforms::rpi::UnwrappedHardwareScheduler::cleanup\n");
    //disable DMA. Otherwise, it will continue to run in the background, potentially overwriting future user data.
    if(dmaHeader) {
        writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
        mitpi::usleep(100);
        writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
    }
    //TODO: could also disable PWM, but that's not imperative.
}

void UnwrappedHardwareScheduler::makeMaps() {
    memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        LOGE("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    pagemapfd = open("/proc/self/pagemap", O_RDONLY);
    //now map /dev/mem into memory, but only map specific peripheral sections:
    //gpioBaseMem = mapPeripheral(GPIO_BASE);
    dmaBaseMem = mapPeripheral(DMA_BASE);
    pwmBaseMem = mapPeripheral(PWM_BASE);
    clockBaseMem = mapPeripheral(CLOCK_BASE);
}

volatile uint32_t* UnwrappedHardwareScheduler::mapPeripheral(int addr) const {
    return mitpi::mapPeripheral(memfd, addr);
}

void UnwrappedHardwareScheduler::initSrcAndControlBlocks() {    
    //configure DMA...
    //First, allocate memory for the source:
    size_t numSrcBlocks = SOURCE_BUFFER_FRAMES; //We want apx 1M blocks/sec.
    size_t srcPageBytes = numSrcBlocks*sizeof(struct GpioBufferFrame);
    srcMem = DmaMem(*this, srcPageBytes);
    
    //cast virtSrcPage to a GpioBufferFrame array:
    srcArray = (struct GpioBufferFrame*)srcMem.virtL2Coherent; //Note: calling virtToPhys on srcArray will return NULL. Use srcArrayCached for that.
    
    //allocate memory for the control blocks
    size_t cbPageBytes = numSrcBlocks * sizeof(struct DmaControlBlock) * 3; //3 cbs for each source block
    cbMem = DmaMem(*this, cbPageBytes);
    //fill the control blocks:
    cbArr = (struct DmaControlBlock*)cbMem.virtL2Coherent;
    
    //Allocate memory for the default src outputs (used in PWM, defaults to zeros)
    srcClrMem = DmaMem(*this, srcPageBytes);
    srcClrArray = (struct GpioBufferFrame*)srcClrMem.virtL2Coherent;
    
    LOG("platforms::rpi::UnwrappedHardwareScheduler::initSrcAndControlBlocks: #dma blocks: %zu, #src blocks: %zu\n", numSrcBlocks*3, numSrcBlocks);
    for (unsigned int i=0; i<numSrcBlocks*3; i += 3) {
        //pace DMA through PWM
        cbArr[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
        //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
        cbArr[i].SOURCE_AD = physToUncached(srcMem.physAddrAtByteOffset(i/3*sizeof(GpioBufferFrame)));
        cbArr[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
        cbArr[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(1) | DMA_CB_TXFR_LEN_XLENGTH(4);
        cbArr[i].STRIDE = i/3;
        cbArr[i].NEXTCONBK = physToUncached(cbMem.physAddrAtByteOffset((i+1)*sizeof(DmaControlBlock)));
        //copy buffer to GPIOs
        cbArr[i+1].TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
        cbArr[i+1].SOURCE_AD = physToUncached(srcMem.physAddrAtByteOffset(i/3*sizeof(GpioBufferFrame)));
        cbArr[i+1].DEST_AD = GPIO_BASE_BUS + GPSET0;
        cbArr[i+1].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(NUM_GPIO_WORDS*4);
        const int stride = 12 - 4*NUM_GPIO_WORDS; //gpset[0] and gpclr[0] are separated by 3 words (12 bytes).
        cbArr[i+1].STRIDE = DMA_CB_STRIDE_D_STRIDE(stride) | DMA_CB_STRIDE_S_STRIDE(0);
        cbArr[i+1].NEXTCONBK = physToUncached(cbMem.physAddrAtByteOffset((i+2)*sizeof(DmaControlBlock)));
        //clear buffer (TODO: investigate using a 4-word copy ("burst") )
        cbArr[i+2].TI = DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
        cbArr[i+2].SOURCE_AD = physToUncached(srcClrMem.physAddrAtByteOffset(i/3*sizeof(struct GpioBufferFrame)));
        cbArr[i+2].DEST_AD = physToUncached(srcMem.physAddrAtByteOffset(i/3*sizeof(GpioBufferFrame)));
        cbArr[i+2].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(1) | DMA_CB_TXFR_LEN_XLENGTH(sizeof(struct GpioBufferFrame));
        cbArr[i+2].STRIDE = i/3; //might be better to use the NEXT index
        int nextIdx = i+3 < numSrcBlocks*3 ? i+3 : 0; //last block should loop back to the first block
        cbArr[i+2].NEXTCONBK = physToUncached(cbMem.physAddrAtByteOffset(nextIdx*sizeof(DmaControlBlock)));
    }
}

uint8_t* UnwrappedHardwareScheduler::makeUncachedMemView(void* virtaddr, size_t bytes) const {
    //by default, writing to any virtual address will go through the CPU cache.
    //this function will return a pointer that behaves the same as virtaddr, but bypasses the CPU L1 cache (note that because of this, the returned pointer and original pointer should not be used in conjunction, else cache-related inconsistencies will arise)
    //Note: The original memory should not be unmapped during the lifetime of the uncached version, as then the OS won't know that our process still owns the physical memory.
    bytes = ceilToPage(bytes);
    //first, just allocate enough *virtual* memory for the operation. This is done so that we can do the later mapping to a contiguous range of virtual memory:
    void *mem = mmap(
        nullptr,   //let kernel place memory where it wants
        bytes,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1, // File descriptor
    0); //no offset into file (file doesn't exist).
    uint8_t *memBytes = (uint8_t*)mem;
    //now, free the virtual memory and immediately remap it to the physical addresses used in virtaddr
    munmap(mem, bytes); //Might not be necessary; MAP_FIXED indicates it can map an already-used page
    for (unsigned int offset=0; offset<bytes; offset += PAGE_SIZE) {
        void *mappedPage = mmap(memBytes+offset, PAGE_SIZE, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED, memfd, virtToUncachedPhys((uint8_t*)virtaddr+offset));
        if (mappedPage != memBytes+offset) { //We need these mappings to be contiguous over virtual memory (in order to replicate the virtaddr array), so we must ensure that the address we requested from mmap was actually used.
            LOGE("platforms::rpi::UnwrappedHardwareScheduler::makeUncachedMemView: failed to create an uncached view of memory at addr %p+0x%08x\n", virtaddr, offset);
            exit(1);
        }
    }
    memset(mem, 0, bytes); //Although the cached version might have been reset, those writes might not have made it through.
    return memBytes;
}

uintptr_t UnwrappedHardwareScheduler::virtToPhys(void* virt) const {
    //uintptr_t pgNum = (uintptr_t)(virt)/PAGE_SIZE;
    int pgNum = (uintptr_t)(virt)/PAGE_SIZE;
    int byteOffsetFromPage = (uintptr_t)(virt)%PAGE_SIZE;
    uint64_t physPage;
    ///proc/self/pagemap is a uint64_t array where the index represents the virtual page number and the value at that index represents the physical page number.
    //So if virtual address is 0x1000000, read the value at *array* index 0x1000000/PAGE_SIZE and multiply that by PAGE_SIZE to get the physical address.
    //because files are bytestreams, one must explicitly multiply each byte index by 8 to treat it as a uint64_t array.
    int err = lseek(pagemapfd, pgNum*8, SEEK_SET);
    if (err != pgNum*8) {
        LOGW("WARNING: platforms::rpi::UnwrappedHardwareScheduler::virtToPhys %p failed to seek (expected %i got %i. errno: %i)\n", virt, pgNum*8, err, errno);
    }
    read(pagemapfd, &physPage, 8);
    if (!physPage & (1ull<<63)) { //bit 63 is set to 1 if the page is present in ram
        LOGW("WARNING: platforms::rpi::UnwrappedHardwareScheduler::virtToPhys %p has no physical address\n", virt);
    }
    physPage = physPage & ~(0x1ffull << 55); //bits 55-63 are flags.
    uintptr_t mapped = (uintptr_t)(physPage*PAGE_SIZE + byteOffsetFromPage);
    return mapped;
}
uintptr_t UnwrappedHardwareScheduler::virtToUncachedPhys(void *virt) const {
    return physToUncached(virtToPhys(virt));
    //return virtToPhys(virt) | 0x40000000; //bus address of the ram is 0x40000000. With this binary-or, writes to the returned address will bypass the CPU (L1) cache, but not the L2 cache. 0xc0000000 should be the base address if L2 must also be bypassed. However, the DMA engine is aware of L2 cache - just not the L1 cache (source: http://en.wikibooks.org/wiki/Aros/Platforms/Arm_Raspberry_Pi_support#Framebuffer )
}

void UnwrappedHardwareScheduler::initPwm() {
    //configure PWM clock:
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL/4))&(~CM_PWMCTL_ENAB)); //disable clock
    do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
    *(clockBaseMem + CM_PWMDIV/4) = CM_PWMDIV_PASSWD | CM_PWMDIV_DIVI(CLOCK_DIV); //configure clock divider (running at 500MHz undivided)
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD; //source 500MHz base clock, no MASH.
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD | CM_PWMCTL_ENAB; //enable clock
    do {} while ((*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY) == 0); //wait for clock to activate
    
    //configure rest of PWM:
    struct PwmHeader *pwmHeader = (struct PwmHeader*)(pwmBaseMem);
    
    pwmHeader->DMAC = 0; //disable DMA
    pwmHeader->CTL |= PWM_CTL_CLRFIFO; //clear pwm
    mitpi::usleep(100);
    
    pwmHeader->STA = PWM_STA_ERRS; //clear PWM errors
    mitpi::usleep(100);
    
    pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(PWM_FIFO_SIZE) | PWM_DMAC_PANIC(PWM_FIFO_SIZE); //DREQ is activated at queue < PWM_FIFO_SIZE
    pwmHeader->RNG1 = BITS_PER_CLOCK; //used only for timing purposes; #writes to PWM FIFO/sec = PWM CLOCK / RNG1
    pwmHeader->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
}

void UnwrappedHardwareScheduler::initDma() {
    //enable DMA channel (it's probably already enabled, but we want to be sure):
    writeBitmasked(dmaBaseMem + DMAENABLE/4, 1 << dmaCh, 1 << dmaCh);
    
    //configure the DMA header to point to our control block:
    dmaHeader = (struct DmaChannelHeader*)(dmaBaseMem + DMACH(dmaCh)/4); //must divide by 4, as dmaBaseMem is uint32_t*
    //abort any previous DMA:
    //dmaHeader->NEXTCONBK = 0; //NEXTCONBK is read-only.
    dmaHeader->CS |= DMA_CS_ABORT; //make sure to disable dma first.
    mitpi::usleep(100); //give time for the abort command to be handled.
    
    dmaHeader->CS = DMA_CS_RESET;
    mitpi::usleep(100);
    
    writeBitmasked(&dmaHeader->CS, DMA_CS_END, DMA_CS_END); //clear the end flag
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    uint32_t firstAddr = physToUncached(cbMem.physAddrAtByteOffset(0));
    LOG("platforms::rpi::UnwrappedHardwareScheduler::initDma: starting DMA @ CONBLK_AD=0x%08x\n", firstAddr);
    dmaHeader->CONBLK_AD = firstAddr; //(uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
    dmaHeader->CS = DMA_CS_PRIORITY(14) | DMA_CS_PANIC_PRIORITY(14) | DMA_CS_DISDEBUG; //high priority (max is 15)
    dmaHeader->CS = DMA_CS_PRIORITY(14) | DMA_CS_PANIC_PRIORITY(14) | DMA_CS_DISDEBUG | DMA_CS_ACTIVE; //activate DMA. 
}

void UnwrappedHardwareScheduler::syncDmaTime() {
    //returns the last time that a frame idx=0 occured.
    EventClockT::time_point _now = EventClockT::now();
    if (_now > _lastDmaSyncedTime) {
        _lastDmaSyncedTime = _now;
        int srcIdx;
        EventClockT::time_point curTime1, curTime2;
        curTime2 = _now;
        do {
            //curTime1 = EventClockT::now();
            curTime1 = curTime2;
            srcIdx = dmaHeader->STRIDE; //the source index is stored in the otherwise-unused STRIDE register, for efficiency
            curTime2 = EventClockT::now();
        } while (std::chrono::duration_cast<std::chrono::microseconds>(curTime2-curTime1).count() > (RUNNING_IN_VM ? 250 : 1) || (srcIdx & DMA_CB_TXFR_YLENGTH_MASK)); //allow 1 uS variability, or 50 uS if running in a VM (valgrind)
        //Uncomment the following lines and the above declaration of _lastTimeAtFrame0 to log jitter information:
        int64_t curTimeAtFrame0 = std::chrono::duration_cast<std::chrono::microseconds>(curTime2.time_since_epoch()).count() - FRAME_TO_USEC(srcIdx);
        int timeDiff = (curTimeAtFrame0-_lastTimeAtFrame0)%FRAME_TO_USEC(SOURCE_BUFFER_FRAMES);
        if (timeDiff > FRAME_TO_USEC(SOURCE_BUFFER_FRAMES)/2) { //wrap-around
            timeDiff -= FRAME_TO_USEC(SOURCE_BUFFER_FRAMES);
        }
        LOGV("Timing diff: %i\n", timeDiff);
        if (timeDiff > 20) {
            LOGW("Warning: Dma timing is off by > 20 uS: %i us\n", timeDiff);
        }
        _lastTimeAtFrame0 = curTimeAtFrame0;
        //if timing diff is positive, then then curTimeAtFrame0 > _lastTimeAtFrame0
        //curTime2 - srcIdx2 > curTime1 - srcIdx1
        //curTime2 - curTime2 > srcIdx2 - srcIdx1
        //more uS have elapsed than frames; DMA cannot keep up
    }
}
bool UnwrappedHardwareScheduler::onIdleCpu(OnIdleCpuIntervalT interval) {
    if (interval == OnIdleCpuIntervalWide) {
        syncDmaTime();
    }
    return false;
}
void UnwrappedHardwareScheduler::queue(const OutputEvent &evt) {
    queue(evt.primitiveIoPin().id(), evt.state(), std::chrono::duration_cast<std::chrono::microseconds>(evt.time().time_since_epoch()).count());
}

void UnwrappedHardwareScheduler::queue(int pin, int mode, uint64_t micros) {
    //This function takes a pin, a mode (0=off, 1=on) and a time. It then manipulates the GpioBufferFrame array in order to ensure that the pin switches to the desired level at the desired time. It will sleep if necessary.
    //Sleep until we are on the right iteration of the circular buffer (otherwise we cannot queue the command)
    uint64_t desiredTime = micros - MAX_SCHED_AHEAD_USEC;
    SleepT::sleep_until(std::chrono::time_point<std::chrono::microseconds>(std::chrono::microseconds(desiredTime)));

    int64_t lastUsecAtFrame0 = _lastTimeAtFrame0;
    int usecFromFrame0 = micros - lastUsecAtFrame0;
    if (usecFromFrame0 < 0) { //need this check to prevent newIdx from being negative.
        LOGV("Warning: clearly missed a step (usecFromFrame0=%i)\n", usecFromFrame0);
        //attempt to recover:
        EventClockT::time_point realNow = EventClockT::now();
        micros = std::chrono::duration_cast<std::chrono::microseconds>(realNow.time_since_epoch()).count();
        micros += MIN_SCHED_AHEAD_USEC; //give ourselves a (128) uS buffer
        usecFromFrame0 = micros - lastUsecAtFrame0;
    }
    int framesFrom0 = USEC_TO_FRAME(usecFromFrame0);
    int newIdx = framesFrom0%SOURCE_BUFFER_FRAMES;

    //Now queue the command:
    if (mode == 0) { //turn output off
        //srcArray[newIdx].gpclr[pin>31] |= 1 << (pin%32);
        srcArray[newIdx].writeGpClr(pin);
    } else { //turn output on
        //srcArray[newIdx].gpset[pin>31] |= 1 << (pin%32);
        srcArray[newIdx].writeGpSet(pin);
    }
}

void UnwrappedHardwareScheduler::queuePwm(const PrimitiveIoPin &pin, float ratio, EventClockT::duration idealPeriod) {
    //PWM is achieved through changing the values that each source frame is reset to.
    //the way to choose which frames are '1' and which are '0' CAN be done like so (but it ISN'T, so read on!):
    //  Keep a counter, which is set to 0.
    //  each frame, increment it by ratio.
    //  if it exceeds 1, then set that output to '1' and subtract 1 from the counter. Else, set the output to 0.
    //  Example(r=0.4):   frame | counter | output
    //                    0     | 0.4     | 0
    //                    1     | 0.8     | 0
    //                    2     | 1.2->0.2| 1
    //                    3     | 0.6     | 0
    //                    4     | 1.0->0.0| 1
    //                 ... cycle repeats
    //  PWM cycle length (resolution) can be set easily to any number which is a divisor of the buffer length.
    //  For simplicity, the original code will use a cycle length equal to the buffer length.
    //
    //With things like heaters, we want less switching
    //We want it such that over a time, idealPeriod, there is only (at most) 1 transition from low to high.
    //Example: r (ratio)=0.3, L (idealPeriod)=5 frames
    //frame | charge | frame-count | output
    //-     | 0      | -           | -
    //0     | -0.7   | 0           | 1
    //1     | -0.4   | 1           | 0
    //2     | -0.1   | 2           | 0
    //3     | 0.2    | 3           | 0
    //4     | 0.5    | 4           | 0
    //5     | -0.2   | 0           | 1
    //6     | 0.1    | 1           | 0
    //7     | 0.4    | 2           | 0
    //8     | 0.7    | 3           | 0
    //9     | 1.0    | 4           | 0
    //10    | 0.3    | 0           | 1
    //11    | -0.4   | 1           | 1
    //12    | -0.1   | 2           | 0
    //The above appears to work decently!
    //algorithmically:
    //set charge, transitionCharge = 0, out=(r>=0.5)
    //each frame, add r to charge, add 1 to frame-count
    //  if charge < 0: out = 0
    //  if charge > 0 && transitionCharge > L: out = 1
    //  charge -= out
    auto pinId = pin.id();
    float minPeriod = std::chrono::duration_cast<std::chrono::duration<float> >(idealPeriod).count()*(float)(FRAMES_PER_SEC);
    float charge=0;
    float transitionCharge=0;
    bool out = (ratio >= 0.5);
    for (int idx=0; idx < SOURCE_BUFFER_FRAMES; ++idx) {
        charge += ratio;
        transitionCharge += 1;
        if (charge <= 0) {
            out = false;
        } else if (transitionCharge >= minPeriod) {
            out = true;
            transitionCharge -= minPeriod;
        }
        charge -= out;
        srcClrArray[idx].writeGpSet(pinId, out); //if OUT, then set SET and clear CLR
        srcClrArray[idx].writeGpClr(pinId, !out); //if !OUT, then clr SET and set CLR
    }
}

}
}
