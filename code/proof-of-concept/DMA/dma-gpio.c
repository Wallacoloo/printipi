/* (C) 2014 Colin Wallace
 * MIT License
 */
/*
 * processor documentation is at: http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * pg 38 for DMA
 * pg 61 for DMA DREQ PERMAP
 * pg 89 for gpio
 * pg 119 for PCM
 * pg 138 for PWM
 * pg 172 for timer info
 *
 * A few annotations for GPIO/DMA/PWM are available here: https://github.com/626Pilot/RaspberryPi-NeoPixel-WS2812/blob/master/ws2812-RPi.c
 *
 * The general idea is to have a buffer of N blocks, where each block is the same size as the gpio registers, 
 *   and have the DMA module continually copying the data in this buffer into those registers.
 * In this way, we can have (say) 32 blocks, and then be able to buffer the next 32 IO frames.
 *
 * How is DMA transfer rate controlled?
 * We can use the DREQ (data request) feature.
 *   PWM supports a configurable data consumption clock (defaults to 100MHz)
 *   PWM (and SPI, PCM) can fire a DREQ signal any time its fifo falls below a certain point.
 *   But we are never filling the FIFO, so DREQ would be permanently high.
 *   Could feed PWM with dummy data, and use 2 DMA channels (one to PWM, one to GPIO, both gated), but the write-time to GPIOs may vary from the PWM, so gating may be improper
 * Or we can use the WAITS portion of the CB header. This allows up to 31 cycle delay -> ~25MHz?
 *   Will have to manually determine timing characteristics though.
 * Or use 2 dma channels:
 *   Have one sending the data into PWM, which is DREQ limited
 *   Have another copying from PWM Fifo to GPIOs at a non-limited rate. This is peripheral -> peripheral, so I think it will have its own data bus.
 *     Unfortunately, the destination can only be one word. Luckily, we have 2 PWM channels - one for setting & one for clearing GPIOs. All gpios that are broken out into the header are in the first register (verified)
 *   Sadly, it appears that the PWM FIFO cannot be read from. One can read the current PWM output, but only if the FIFO is disabled, in which case the DREQ is too.
 *
 **Or use 1 dma channel, but additionally write to a dreq-able peripheral (PWM):
 *   By using control-blocks, one can copy a word to the GPIOs, then have the next CB copy a word to the PWM fifo, and repeat
 *   By having BOTH control-blocks be dreq-limited by the PWM's dreq, they can BOTH be rate-limited.
 *
 * http://www.raspberrypi.org/forums/viewtopic.php?f=44&t=26907
 *   Says gpu halts all DMA for 16us every 500ms. Bypassable.
 */
 
#include <sys/mman.h> //for mmap
#include <unistd.h> //for NULL
#include <stdio.h> //for printf
#include <stdlib.h> //for exit
#include <fcntl.h> //for file opening
#include <stdint.h> //for uint32_t

#define TIMER_BASE   0x20003000
#define TIMER_CLO    0x00000004 //lower 32-bits of 1 MHz timer
#define TIMER_CHI    0x00000008 //upper 32-bits
 

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
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
#define DMACH0   0x00000000
#define DMACH1   0x00000100
#define DMACH2   0x00000200
#define DMACH3   0x00000300
//...
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC    (1<<4)
#define DMA_CB_TI_DEST_DREQ   (1<<6)
#define DMA_CB_TI_SRC_INC     (1<<8)
#define DMA_CB_TI_SRC_DREQ    (1<<10)
#define DMA_CB_TI_PERMAP_NONE (0<<16)
#define DMA_CB_TI_PERMAP_DSI  (1<<16)
//... (more found on page 61 of BCM2835 pdf
#define DMA_CB_TI_PERMAP_PWM  (5<<16)
//...
#define DMA_CB_TI_NO_WIDE_BURSTS (1<<26)

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
#define PWM_DMAC_PANIC(P) ((P&0xff)<<8)
#define PWM_DMAC_DREQ(D) ((D&0xff)<<0)

//set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
//eg if x = 0b11001100
//  writeBitmasked(&x, 0b00000110, 0b11110011),
//  then x now = 0b11001110
void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
    uint32_t cur = *dest;
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //best to be safe 
}

struct DmaChannelHeader {
    volatile uint32_t CS; //Control and Status
        //31    RESET; set to 1 to reset DMA
        //30    ABORT; set to 1 to abort current DMA control block (next one will be loaded & continue)
        //29    DISDEBUG; set to 1 and DMA won't be paused when debug signal is sent
        //28    WAIT_FOR_OUTSTANDING_WRITES; set to 1 and DMA will wait until peripheral says all writes have gone through before loading next CB
        //24-74 reserved
        //20-23 PANIC_PRIORITY; 0 is lowest priority
        //16-19 PRIORITY; bus scheduling priority. 0 is lowest
        //9-15  reserved
        //8     ERROR; read as 1 when error is encountered. error can be found in DEBUG register.
        //7     reserved
        //6     WAITING_FOR_OUTSTANDING_WRITES; read as 1 when waiting for outstanding writes
        //5     DREQ_STOPS_DMA; read as 1 if DREQ is currently preventing DMA
        //4     PAUSED; read as 1 if DMA is paused
        //3     DREQ; copy of the data request signal from the peripheral, if DREQ is enabled. reads as 1 if data is being requested, else 0
        //2     INT; set when current CB ends and its INTEN=1. Write a 1 to this register to clear it
        //1     END; set when the transfer defined by current CB is complete. Write 1 to clear.
        //0     ACTIVE; write 1 to activate DMA (load the CB before hand)
    volatile uint32_t CONBLK_AD; //Control Block Address
    volatile uint32_t TI; //transfer information; see DmaControlBlock.TI for description
    volatile uint32_t SOURCE_AD; //Source address
    volatile uint32_t DEST_AD; //Destination address
    volatile uint32_t TXFR_LEN; //transfer length.
    volatile uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    volatile uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    volatile uint32_t DEBUG; //controls debug settings
};

struct DmaControlBlock {
    uint32_t TI; //transfer information
        //31:27 unused
        //26    NO_WIDE_BURSTS
        //21:25 WAITS; number of cycles to wait between each DMA read/write operation
        //16:20 PERMAP; peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
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
    volatile uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
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

//allocate a page & simultaneously determine its physical address.
//virtAddr and physAddr are essentially passed by-reference.
//this allows for:
//void *virt, *phys;
//makeVirtPhysPage(&virt, &phys)
//now, virt[N] exists for 0 <= N < PAGE_SIZE,
//  and phys+N is the physical address for virt[N]
//based on http://www.raspians.com/turning-the-raspberry-pi-into-an-fm-transmitter/
void makeVirtPhysPage(void** virtAddr, void** physAddr) {
    *virtAddr = valloc(PAGE_SIZE); //allocate one page of RAM

    //force page into RAM and then lock it ther:
    ((int*)*virtAddr)[0] = 1;
    mlock(*virtAddr, PAGE_SIZE);
    ((int*)*virtAddr)[0] = 0; //undo the change we made above. This way the page should be zero-filled

    //Magic to determine the physical address for this page:
    uint64_t pageInfo;
    int file = open("/proc/self/pagemap", 'r');
    lseek(file, ((uint32_t)*virtAddr)/PAGE_SIZE*8, SEEK_SET);
    read(file, &pageInfo, 8);

    *physAddr = (void*)(uint32_t)(pageInfo*PAGE_SIZE);
    printf("makeVirtPhysPage virtual to phys: %p -> %p\n", *virtAddr, *physAddr);
}

//call with virtual address to deallocate a page allocated with makeVirtPhysPage
void freeVirtPhysPage(void* virtAddr) {
    munlock(virtAddr, PAGE_SIZE);
    free(virtAddr);
}

//map a physical address into our virtual address space. memfd is the file descriptor for /dev/mem
volatile uint32_t* mapPeripheral(int memfd, int addr) {
    ///dev/mem behaves as a file. We need to map that file into memory:
    void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, addr);
    //now, *mapped = memory at physical address of addr.
    if (mapped == MAP_FAILED) {
        printf("failed to map memory (did you remember to run as root?)\n");
        exit(1);
    } else {
        printf("mapped: %p\n", mapped);
    }
    return (volatile uint32_t*)mapped;
}

uint64_t readSysTime(volatile uint32_t *timerBaseMem) {
    return ((uint64_t)*(timerBaseMem + TIMER_CHI/4) << 32) + (uint64_t)(*(timerBaseMem + TIMER_CLO/4));
}


void printMem(volatile void *begin, int numChars) {
    volatile uint32_t *addr = (volatile uint32_t*)begin;
    volatile uint32_t *end = addr + numChars/4;
    while (addr < end) {
        printf("%08x ", *addr);
        ++addr;
    }
    printf("\n");
}

int main() {
    //First, we need to obtain the virtual base-address of our program:
    //void *virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
	//		MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
	//		-1, 0);
    //First, open the linux device, /dev/mem
    //dev/mem provides access to the physical memory of the entire processor+ram
    //This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        printf("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    //now map /dev/mem into memory, but only map specific peripheral sections:
    volatile uint32_t *gpioBaseMem = mapPeripheral(memfd, GPIO_BASE);
    volatile uint32_t *dmaBaseMem = mapPeripheral(memfd, DMA_BASE);
    volatile uint32_t *pwmBaseMem = mapPeripheral(memfd, PWM_BASE);
    volatile uint32_t *timerBaseMem = mapPeripheral(memfd, TIMER_BASE);
    
    //now set our pin (#4) as an output:
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0);
    uint32_t fselMask = 0x7 << (3*4); //bitmask for the 3 bits that control pin 4
    uint32_t fselValue = 0x1 << (3*4); //value that we want to give the above bitmask (0b001 = set mode to output)
    *fselAddr = ((*fselAddr) & ~fselMask) | fselValue; //set pin 4 to be an output.
    
    //configure DMA...
    //First, allocate 1 page for the source:
    void *virtSrcPage, *physSrcPage;
    makeVirtPhysPage(&virtSrcPage, &physSrcPage);
    
    //write a few bytes to the source page:
    uint32_t *srcArray = (uint32_t*)virtSrcPage;
    srcArray[0]  = (1 << 4); //set pin 4 ON
    srcArray[1]  = 0; //GPSET1
    srcArray[2]  = 0; //padding
    //srcArray[3]  = (1 << 4); //set pin 4 OFF
    srcArray[3]  = 0; //GPCLR0
    srcArray[4]  = 0; //GPCLR1
    srcArray[5]  = 0; //padding
    
    srcArray[64*6+0]  = 0; //GPSET0
    srcArray[64*6+1]  = 0; //GPSET1
    srcArray[64*6+2]  = 0; //padding
    srcArray[64*6+3]  = (1 << 4); //GPCLR0
    srcArray[64*6+4] = 0; //GPCLR1
    srcArray[64*6+5] = 0; //padding
    
    //allocate 1 page for the control blocks
    void *virtCbPage, *physCbPage;
    makeVirtPhysPage(&virtCbPage, &physCbPage);
    
    //dedicate the first 8 bytes of this page to holding the cb.
    //struct DmaControlBlock *cb1 = (struct DmaControlBlock*)virtCbPage;
    //struct DmaControlBlock *cb2 = (struct DmaControlBlock*)(virtCbPage+1*DMA_CONTROL_BLOCK_ALIGNMENT);
    //struct DmaControlBlock *cb3 = (struct DmaControlBlock*)(virtCbPage+2*DMA_CONTROL_BLOCK_ALIGNMENT);
    struct PwmHeader *pwmHeader = (struct PwmHeader*)(pwmBaseMem);
    
    pwmHeader->STA = PWM_STA_ERRS; //clear PWM errors
    pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(7) | PWM_DMAC_PANIC(7);
    pwmHeader->RNG1 = 32; //32-bit output periods (used only for timing purposes)
    pwmHeader->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
    
    //fill the control block:
    //after each 4-byte copy, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address:
    struct DmaControlBlock *cbArr = (struct DmaControlBlock*)virtCbPage;
    int maxIdx = PAGE_SIZE/DMA_CONTROL_BLOCK_ALIGNMENT;
    for (int i=0; i<maxIdx; i += 2) {
        cbArr[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS;
        cbArr[i].SOURCE_AD = (uint32_t)(physSrcPage + i/2*24);
        cbArr[i].DEST_AD = GPIO_BASE_BUS + GPSET0;
        cbArr[i].TXFR_LEN = 24;
        cbArr[i].STRIDE = 0;
        cbArr[i].NEXTCONBK = (uint32_t)physCbPage + ((void*)&cbArr[i+1] - virtCbPage);
        
        cbArr[i+1].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS;
        cbArr[i+1].SOURCE_AD = (uint32_t)physSrcPage;
        cbArr[i+1].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
        cbArr[i+1].TXFR_LEN = 4;
        cbArr[i+1].STRIDE = 0;
        //cbArr[i+1].NEXTCONBK = &(cbArr[(i+2)%maxIdx]);
        cbArr[i+1].NEXTCONBK = (uint32_t)physCbPage + ((void*)&cbArr[(i+2)%maxIdx] - virtCbPage);
    }
    /*cb1->TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS; 
    cb1->SOURCE_AD = (uint32_t)physSrcPage; //set source and destination DMA address
    cb1->DEST_AD = GPIO_BASE_BUS + GPSET0;
    cb1->TXFR_LEN = 24; //number of bytes to transfer
    cb1->STRIDE = 0; //no 2D stride
    //cb1->NEXTCONBK = (uint32_t)physCbPage; //loop back to this block.
    //cb1->NEXTCONBK = 0; //end block.
    cb1->NEXTCONBK = (uint32_t)(physCbPage + ((void*)cb2-virtCbPage)); //next block is control-block #2
    
    cb2->TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS;
    cb2->SOURCE_AD = (uint32_t)physSrcPage; //can write junk into PWM, so just use an address that's likely to be cached already
    cb2->DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
    cb2->TXFR_LEN = 4; //just one sample
    cb2->STRIDE = 0; //no 2D stride
    cb2->NEXTCONBK = 0; //no next block.
    //cb2->NEXTCONBK = (uint32_t)physCbPage + ((void*)cb1 - virtCbPage); //loop back to first block.
    cb2->NEXTCONBK = (uint32_t)physCbPage + ((void*)cb3 - virtCbPage);
    
    cb3->TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS; 
    cb3->SOURCE_AD = (uint32_t)(physSrcPage+24); //set source and destination DMA address
    cb3->DEST_AD = GPIO_BASE_BUS + GPSET0;
    cb3->TXFR_LEN = 24; //number of bytes to transfer
    cb3->STRIDE = 0; //no 2D stride
    cb3->NEXTCONBK = 0; //end block.*/
    
    //enable DMA channel (it's probably already enabled, but we want to be sure):
    writeBitmasked(dmaBaseMem + DMAENABLE, 1 << 3, 1 << 3);
    
    //configure the DMA header to point to our control block:
    volatile struct DmaChannelHeader *dmaHeader = (volatile struct DmaChannelHeader*)(dmaBaseMem + DMACH3);
    dmaHeader->CS = DMA_CS_RESET; //make sure to disable dma first.
    sleep(1); //give time for the reset command to be handled.
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    dmaHeader->CONBLK_AD = (uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
    //uint64_t t1 = readSysTime(timerBaseMem);
    dmaHeader->CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.
    
    //sleep(1); //give time for copy to happen
    //while (1) { pause(); }
    while (dmaHeader->CS & DMA_CS_ACTIVE) {} //wait for DMA transfer to complete.
    //uint64_t t2 = readSysTime(timerBaseMem);
    //cleanup
    freeVirtPhysPage(virtCbPage);
    freeVirtPhysPage(virtSrcPage);
    //printf("system time: %llu\n", t1);
    //printf("system time: %llu\n", t2);
    return 0;
}
