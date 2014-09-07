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
 * Addendum is http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
 *
 * A few annotations for GPIO/DMA/PWM are available here: https://github.com/626Pilot/RaspberryPi-NeoPixel-WS2812/blob/master/ws2812-RPi.c
 *   https://github.com/metachris/raspberrypi-pwm/blob/master/rpio-pwm/rpio_pwm.c
 *   https://github.com/richardghirst/PiBits/blob/master/ServoBlaster/user/servod.c
 *
 * Cache info can be found here: http://www.freelists.org/post/raspi-internals/caches,18
 *   0x00000000 - L1 & L2 cache
 *   0x40000000 - L2 cache coherent (ie L1 writes are propagated to L2?)
 *   0x80000000 - L2 cache only
 *   0xc0000000 - direct uncached
 *
 * Useful DMA timings, etc: http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=7696&start=50
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
 *   PWM clock works as so: 500MHz / clock_div = PWM_BITRATE (note: bitrate!)
 *     PWM_BITRATE / PWM_RNG1 = #of FIFO writes/sec
 *     Max PWM_BITRATE = 25MHz
 *   Also, dest_addr = 0x7e20b000 // the testbus interface which is a dump peripheral that goes nowhere (http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=7696&start=25 )
 *
 * DMA Control Block layout:
 *   repeat #srcBlock times:
 *     1.copy srcBlock to gpios
 *     2.zero srcBlock
 *     3.move byte to PWM (paced via DREQ)
 *   These are largely redundant; it may be possible to use less memory (each cb uses 32 bytes of memory)
 *
 * Problem: each "frame" is currently 6 words (but the last word is padding), and 1 PAGE_SIZE is not an integer multiple of 6*4
 *   Thus, the very last frame on each page cannot be used with DMA. Because of this, too, the virtual addressing of each frame is messed up - we must skip one frame per page.
 *   One solution is to append 2 more pad words to each frame (so that it is 8 words in length). This fixes all issues, but increases ram usage and potentially cache problems (L2 is 128KB). However, since data reads are sequential, even if all data doesn't fit in cache, it will be prefetched.
 *   Another solution is to decrease frame size to 4 words, and use 2 control blocks for each frame (thus eliminating the 1-byte padding in the center). This will have an even LARGER impact on ram usage - effectively using 20 words/frame vs current 14 words/frame & alternative 16words/frame
 *   Another solution is to directly mix src data with CB data. Each CB has 2 words of padding, and a data frame is 5 words, and each CB must be aligned to 8 words. Therefore, the following is possible, assuming each frame requires 3 CBs:
 *     CB1.1(padded) | CB1.2(padded) | CB1.3(padded) | CB2.1(padded) | CB2.2(padded) | CB2.3(unpadded) | SRC(5) | SRC(5) <- uses 56 words per 2 frames 
 *     HOWEVER, PAGE_SIZE is not an integral multiple of 56 words
 *     Although, 1 of those CBs (the one which zeros the previous source) could be shared amongst multiple frames - that is, only zero every, say, 4 frames. The effect is:
 *      *32 words for 1 frame  grouped (5  src words - 2 means pad to 8 words for src)
 *       48 words for 2 frames grouped (10 src words - 2 means pad to 8 words for src)
 *       72 words for 3 frames grouped (15 src words - 2 means pad to 16 words for src)
 *       96 words for 4 frames grouped (20 src words - 2 means pad to 24 words for src)
 *       112 words for 5 frames grouped(25 src words - 2 means pad to 24 words for src)
 *       136 words for 6 frames grouped(30 src words - 2 means pad to 32 words for src)
 *       160 words for 7 frames grouped(35 src words - 2 means pad to 40 words for src)
 *       176 words for 8 frames grouped(40 src words - 2 means pad to 40 words for src)
 *       200 words for 9 frames grouped(45 src words - 2 means pad to 48 words for src)
 *       216 words for 10frames grouped(50 src words - 2 means pad to 48 words for src)
 *       240 words for 11frames grouped(55 src words - 2 means pad to 56 words for src)
 *       264 words for 12frames grouped(60 src words - 2 means pad to 64 words for src)
 *    ...432 words for 20frames grouped(100src words - 2 means pad to 104 words for src)
 *   ...*512 words for 24frames grouped(120src words - 2 means pad to 120 words for src)
 *     As can be seen, this still requires extra padding. Could do 128 words for 5 frames, or 256 words for 11 frames (23.3 words/frame), and that requires funky math.
 *     The 24 frame option would work OK. 24 is a relatively easy number to work with, and 21.3 words/frame (limit is 21 words/frame)
 *    Another solution is to use the 2D stride functionality. The source frame is really 4 words and the destination is really 2 words, a 1 word gap, and then the other 2 words. Thus 2d stride can be used to skip over that one word gap.
 *
 *  How to determine the current source word being processed?
 *    dma header points to the physical CONBLOCK_AD. This can be linked to the virtual source address via a map.
 *    OR: STRIDE register is unused in 1D mode. Could write the src index that this block is linked to in that register. But then we can't use stride feature.
 *      Rather, we can't use the stride feature on ONE cb per frame. So, use stride on the buffer->GPIO cb, and use the stride register to indicate index on the zeros-copy and the PWM cb. Can tell which CB we're looking at based on the 2DEN flag. If we're looking at the buffer->GPIO cb, then instead look at NEXTCON_BK
 *    NOTE: if 2d stride is disabled, it appears that the DMA engine doesn't even load the STRIDE register (it's read as garbage). It may PERHAPS display the last loaded word.
 *    Note: unused fields are read as "Don't care", meaning we can't use them to store user-data.
 *
 * http://www.raspberrypi.org/forums/viewtopic.php?f=44&t=26907
 *   Says gpu halts all DMA for 16us every 500ms. To bypass, add 'disable_pvt=1' to /boot/cmdline.txt
 * http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=7696&start=25
 *   Says it's possible to get access to a 250MHz clock.
 * How to make DMA more consistent (ie reduce bus contention?):
 *   disable interrupts 1 uS before any 'real' transaction, enable them afterwards
 *   Make sure dummy writes DON'T READ FROM RAM (ie, use src_ignore = 1)
 *   boot with disable_pvt=1 (prevents gpu from halting everything to adjust ram refresh rate twice per second) in /boot/cmdline.txt. Does this affect system stability?
 */
 
#include <sys/mman.h> //for mmap
#include <sys/time.h> //for timespec
#include <time.h> //for timespec / nanosleep (need -std=gnu99)
#include <signal.h> //for sigaction
#include <unistd.h> //for NULL
#include <stdio.h> //for printf
#include <stdlib.h> //for exit, valloc
//#include <malloc.h> //some implementations declare valloc inside malloc.h
#include <fcntl.h> //for file opening
#include <stdint.h> //for uint32_t
#include <string.h> //for memset
#include <errno.h> //for errno
#include <pthread.h> //for pthread_setschedparam

//config settings:
#define PWM_FIFO_SIZE 1 //The DMA transaction is paced through the PWM FIFO. The PWM FIFO consumes 1 word every N uS (set in clock settings). Once the fifo has fewer than PWM_FIFO_SIZE words available, it will request more data from DMA. Thus, a high buffer length will be more resistant to clock drift, but may occasionally request multiple frames in a short succession (faster than FRAME_PER_SEC) in the presence of bus contention, whereas a low buffer length will always space frames AT LEAST 1/FRAMES_PER_SEC seconds apart, but may experience clock drift.
#define SOURCE_BUFFER_FRAMES 8192 //number of gpio timeslices to buffer. These are processed at ~1 million/sec. So 1000 framse is 1 ms. Using a power-of-two is a good idea as it simplifies some of the arithmetic (modulus operations)
#define FRAMES_PER_SEC 1000000 //Note that this number is currently hard-coded in the form of clock settings. Changing this without changing the clock settings will cause problems
#define SCHED_PRIORITY 30 //Linux scheduler priority. Higher = more realtime


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
#define DMA_CS_PRIORITY_MAX DMA_CS_PRIORITY(7)
#define DMA_CS_PANIC_PRIORITY(x) ((x)&0xf << 20)
#define DMA_CS_PANIC_PRIORITY_MAX DMA_CS_PANIC_PRIORITY(7)
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


//https://dev.openwrt.org/browser/trunk/target/linux/brcm2708/patches-3.10/0070-bcm2708_fb-DMA-acceleration-for-fb_copyarea.patch?rev=39770 says that YLENGTH should actually be written as # of copies *MINUS ONE*
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
void logDmaChannelHeader(struct DmaChannelHeader *h) {
    printf("Dma Ch Header:\n CS: 0x%08x\n CONBLK_AD: 0x%08x\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: %u\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n DEBUG: 0x%08x\n", h->CS, h->CONBLK_AD, h->TI, h->SOURCE_AD, h->DEST_AD, h->TXFR_LEN, h->STRIDE, h->NEXTCONBK, h->DEBUG);
}

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

void logDmaControlBlock(struct DmaControlBlock *b) {
    printf("Dma Control Block:\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: 0x%08x\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n unused: 0x%08x %08x\n", b->TI, b->SOURCE_AD, b->DEST_AD, b->TXFR_LEN, b->STRIDE, b->NEXTCONBK, b->_reserved[0], b->_reserved[1]);
}

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

struct GpioBufferFrame {
    //custom structure used for storing the GPIO buffer.
    //These BufferFrame's are DMA'd into the GPIO memory, potentially using the DmaEngine's Stride facility
    uint32_t gpset[2];
    uint32_t gpclr[2];
};

struct DmaChannelHeader *dmaHeader; //must be global for cleanup()

void setSchedPriority(int priority) {
    //In order to get the best timing at a decent queue size, we want the kernel to avoid interrupting us for long durations.
    //This is done by giving our process a high priority. Note, must run as super-user for this to work.
    struct sched_param sp; 
	sp.sched_priority=priority; 
	int ret;
	if (ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
		printf("Warning: pthread_setschedparam (increase thread priority) returned non-zero: %i\n", ret);
	}
}

void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
    //set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
    //eg if x = 0b11001100
    //  writeBitmasked(&x, 0b00000110, 0b11110011),
    //  then x now = 0b11001110
    uint32_t cur = *dest;
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //best to be safe when crossing memory boundaries
}


uint64_t readSysTime(volatile uint32_t *timerBaseMem) {
    return ((uint64_t)*(timerBaseMem + TIMER_CHI/4) << 32) + (uint64_t)(*(timerBaseMem + TIMER_CLO/4));
}

size_t ceilToPage(size_t size) {
    //round up to nearest page-size multiple
    if (size & (PAGE_SIZE-1)) {
        size += PAGE_SIZE - (size & (PAGE_SIZE-1));
    }
    return size;
}

uintptr_t virtToPhys(void* virt, int pagemapfd) {
    uintptr_t pgNum = (uintptr_t)(virt)/PAGE_SIZE;
    int byteOffsetFromPage = (uintptr_t)(virt)%PAGE_SIZE;
    uint64_t physPage;
    ///proc/self/pagemap is a uint64_t array where the index represents the virtual page number and the value at that index represents the physical page number.
    //So if virtual address is 0x1000000, read the value at *array* index 0x1000000/PAGE_SIZE and multiply that by PAGE_SIZE to get the physical address.
    //because files are bytestreams, one must explicitly multiply each byte index by 8 to treat it as a uint64_t array.
    int err = lseek(pagemapfd, pgNum*8, SEEK_SET);
    if (err != pgNum*8) {
        printf("WARNING: virtToPhys %p failed to seek (expected %i got %i. errno: %i)\n", virt, pgNum*8, err, errno);
    }
    read(pagemapfd, &physPage, 8);
    if (!physPage & (1ull<<63)) { //bit 63 is set to 1 if the page is present in ram
        printf("WARNING: virtToPhys %p has no physical address\n", virt);
    }
    physPage = physPage & ~(0x1ffull << 55); //bits 55-63 are flags.
    uintptr_t mapped = (uintptr_t)(physPage*PAGE_SIZE + byteOffsetFromPage);
    return mapped;
}

uintptr_t virtToUncachedPhys(void *virt, int pagemapfd) {
    return virtToPhys(virt, pagemapfd) | 0x40000000; //bus address of the ram is 0x40000000. With this binary-or, writes to the returned address will bypass the CPU (L1) cache, but not the L2 cache. 0xc0000000 should be the base address if L2 must also be bypassed. However, the DMA engine is aware of L2 cache - just not the L1 cache (source: http://en.wikibooks.org/wiki/Aros/Platforms/Arm_Raspberry_Pi_support#Framebuffer )
}


//allocate some memory and lock it so that its physical address will never change
void* makeLockedMem(size_t size) {
    //void* mem = valloc(size); //memory returned by valloc is not zero'd
    size = ceilToPage(size);
    void *mem = mmap(
        NULL,   //let kernel place memory where it wants
        size,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1,	// File descriptor
    0); //no offset into file (file doesn't exist).
    if (mem == MAP_FAILED) {
        printf("makeLockedMem failed\n");
        exit(1);
    }
    memset(mem, 0, size); //simultaneously zero the pages and force them into memory
    mlock(mem, size);
    return mem;
}

//free memory allocated with makeLockedMem
void freeLockedMem(void* mem, size_t size) {
    size = ceilToPage(size);
    munlock(mem, size);
    munmap(mem, size);
}

void* makeUncachedMemView(void* virtaddr, size_t bytes, int memfd, int pagemapfd) {
    //by default, writing to any virtual address will go through the CPU cache.
    //this function will return a pointer that behaves the same as virtaddr, but bypasses the CPU L1 cache (note that because of this, the returned pointer and original pointer should not be used in conjunction, else cache-related inconsistencies will arise)
    //Note: The original memory should not be unmapped during the lifetime of the uncached version, as then the OS won't know that our process still owns the physical memory.
    bytes = ceilToPage(bytes);
    //first, just allocate enough *virtual* memory for the operation. This is done so that we can do the later mapping to a contiguous range of virtual memory:
    void *mem = mmap(
        NULL,   //let kernel place memory where it wants
        bytes,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1,	// File descriptor
    0); //no offset into file (file doesn't exist).
    //now, free the virtual memory and immediately remap it to the physical addresses used in virtaddr
    munmap(mem, bytes); //Might not be necessary; MAP_FIXED indicates it can map an already-used page
    for (int offset=0; offset<bytes; offset += PAGE_SIZE) {
        void *mappedPage = mmap(mem+offset, PAGE_SIZE, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED, memfd, virtToUncachedPhys(virtaddr+offset, pagemapfd));
        if (mappedPage != mem+offset) { //We need these mappings to be contiguous over virtual memory (in order to replicate the virtaddr array), so we must ensure that the address we requested from mmap was actually used.
            printf("Failed to create an uncached view of memory at addr %p+0x%08x\n", virtaddr, offset);
            exit(1);
        }
    }
    memset(mem, 0, bytes); //Although the cached version might have been reset, those writes might not have made it through.
    return mem;
}

//free memory allocated with makeLockedMem
void freeUncachedMemView(void* mem, size_t size) {
    size = ceilToPage(size);
    munmap(mem, size);
}

//map a physical address into our virtual address space. memfd is the file descriptor for /dev/mem
volatile uint32_t* mapPeripheral(int memfd, int addr) {
    ///dev/mem behaves as a file. We need to map that file into memory:
    //NULL = virtual address of mapping is chosen by kernel.
    //PAGE_SIZE = map 1 page.
    //PROT_READ|PROT_WRITE means give us read and write priveliges to the memory
    //MAP_SHARED means updates to the mapped memory should be written back to the file & shared with other processes
    //memfd = /dev/mem file descriptor
    //addr = offset in file to map
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


void cleanup() {
    printf("Cleanup\n");
    //disable DMA. Otherwise, it will continue to run in the background, potentially overwriting future user data.
    if(dmaHeader) {
        writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
        usleep(100);
        writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
    }
    //could also disable PWM, but that's not imperative.
}

void cleanupAndExit(int sig) {
    cleanup();
    printf("Exiting with error; caught signal: %i\n", sig);
    exit(1);
}

void sleepUntilMicros(uint64_t micros, volatile uint32_t* timerBaseMem) {
    //Note: cannot use clock_nanosleep with an absolute time, as the process clock may differ from the RPi clock.
    //this function doesn't need to be super precise, so we can tolerate interrupts.
    //Therefore, we can use a relative sleep:
    uint64_t cur = readSysTime(timerBaseMem);
    if (micros > cur) { //avoid overflow caused by unsigned arithmetic
        uint64_t dur = micros - cur;
        //usleep(dur); //nope, causes problems!
        struct timespec t;
        t.tv_sec = dur/1000000;
        t.tv_nsec = (dur - t.tv_sec*1000000)*1000;
        nanosleep(&t, NULL);
    }
}

void queue(int pin, int mode, uint64_t micros, struct GpioBufferFrame* srcArray, volatile uint32_t* timerBaseMem, struct DmaChannelHeader* dmaHeader) {
    //This function takes a pin, a mode (0=off, 1=on) and a time. It then manipulates the GpioBufferFrame array in order to ensure that the pin switches to the desired level at the desired time. It will sleep if necessary.
    //Sleep until we are on the right iteration of the circular buffer (otherwise we cannot queue the command)
    uint64_t callTime = readSysTime(timerBaseMem); //only used for debugging
    uint64_t desiredTime = micros-((uint64_t)SOURCE_BUFFER_FRAMES)*1000000/FRAMES_PER_SEC;
    sleepUntilMicros(desiredTime, timerBaseMem);
    uint64_t awakeTime = readSysTime(timerBaseMem); //only used for debugging
    
    //get the current source index at the current time:
    //must ensure we aren't interrupted during this calculation, hence the two timers instead of 1. 
    //Note: getting the curTime & srcIdx don't have to be done for every call to queue - it could be done eg just once per buffer.
    //  It should be calculated regularly though, to counter clock drift & PWM FIFO underflows
    //  It is done in this function only for simplicity
    int srcIdx;
    uint64_t curTime1, curTime2;
    int tries=0;
    do {
        curTime1 = readSysTime(timerBaseMem);
        srcIdx = dmaHeader->STRIDE; //the source index is stored in the otherwise-unused STRIDE register, for efficiency
        curTime2 = readSysTime(timerBaseMem);
        ++tries;
    } while (curTime2-curTime1 > 1 || (srcIdx & DMA_CB_TXFR_YLENGTH_MASK)); //allow 1 uS variability.
    //calculate the frame# at which to place the event:
    int usecFromNow = micros - curTime2;
    int framesFromNow = usecFromNow*FRAMES_PER_SEC/1000000; //Note: may cause overflow if FRAMES_PER_SECOND is not a multiple of 1000000 or if optimizations are COMPLETELY disabled.
    if (framesFromNow < 10) { //Not safe to schedule less than ~10uS into the future (note: should be operating on usecFromNow, not framesFromNow)
        printf("Warning: behind schedule: %i (%i) (tries: %i) (sleep %llu -> %llu (wanted %llu))\n", framesFromNow, usecFromNow, tries, callTime, awakeTime, desiredTime);
        framesFromNow = 10;
    }
    int newIdx = (srcIdx + framesFromNow)%SOURCE_BUFFER_FRAMES;
    //Now queue the command:
    if (mode == 0) { //turn output off
        srcArray[newIdx].gpclr[pin>31] |= 1 << (pin%32);
    } else { //turn output on
        srcArray[newIdx].gpset[pin>31] |= 1 << (pin%32);
    }
}

int main() {
    volatile uint32_t *gpioBaseMem, *dmaBaseMem, *pwmBaseMem, *timerBaseMem, *clockBaseMem;
    //emergency clean-up:
    for (int i = 0; i < 64; i++) { //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = cleanupAndExit;
        sigaction(i, &sa, NULL);
    }
    setSchedPriority(SCHED_PRIORITY);
    //First, open the linux device, /dev/mem
    //dev/mem provides access to the physical memory of the entire processor+ram
    //This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        printf("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    int pagemapfd = open("/proc/self/pagemap", O_RDONLY);
    //now map /dev/mem into memory, but only map specific peripheral sections:
    gpioBaseMem = mapPeripheral(memfd, GPIO_BASE);
    dmaBaseMem = mapPeripheral(memfd, DMA_BASE);
    pwmBaseMem = mapPeripheral(memfd, PWM_BASE);
    timerBaseMem = mapPeripheral(memfd, TIMER_BASE);
    clockBaseMem = mapPeripheral(memfd, CLOCK_BASE);
    
    int outPin = 10;
    //now set our pin as an output:
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0/4 + outPin/10);
    writeBitmasked(fselAddr, 0x7 << (3*(outPin%10)), 0x1 << (3*(outPin%10)));
    //Note: PWM pacing still works, even with no physical outputs, so we don't need to set gpio pin 18 to its alternate function.
    
    //Often need to copy zeros with DMA. This array can be the source. Needs to all lie on one page
    void *zerosPageCached = makeLockedMem(PAGE_SIZE);
    void *zerosPage = makeUncachedMemView(zerosPageCached, PAGE_SIZE, memfd, pagemapfd);
    
    //configure DMA...
    //First, allocate memory for the source:
    size_t numSrcBlocks = SOURCE_BUFFER_FRAMES; //We want apx 1M blocks/sec.
    size_t srcPageBytes = numSrcBlocks*sizeof(struct GpioBufferFrame);
    void *virtSrcPageCached = makeLockedMem(srcPageBytes);
    void *virtSrcPage = makeUncachedMemView(virtSrcPageCached, srcPageBytes, memfd, pagemapfd);
    printf("mappedPhysSrcPage: %p\n", virtToPhys(virtSrcPage, pagemapfd));
    
    //cast virtSrcPage to a GpioBufferFrame array:
    struct GpioBufferFrame *srcArray = (struct GpioBufferFrame*)virtSrcPage; //Note: calling virtToPhys on srcArray will return NULL. Use srcArrayCached for that.
    struct GpioBufferFrame *srcArrayCached = (struct GpioBufferFrame*)virtSrcPageCached;
    //srcArray[0].gpset[0] = (1 << outPin); //set pin ON
    //srcArray[numSrcBlocks/2].gpclr[0] = (1 << outPin); //set pin OFF;
    
    //configure PWM clock:
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL/4))&(~CM_PWMCTL_ENAB)); //disable clock
    do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
    *(clockBaseMem + CM_PWMDIV/4) = CM_PWMDIV_PASSWD | CM_PWMDIV_DIVI(50); //configure clock divider (running at 500MHz undivided)
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD; //source 500MHz base clock, no MASH.
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD | CM_PWMCTL_ENAB; //enable clock
    do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY == 0); //wait for clock to activate
    
    //configure rest of PWM:
    struct PwmHeader *pwmHeader = (struct PwmHeader*)(pwmBaseMem);
    
    pwmHeader->DMAC = 0; //disable DMA
    pwmHeader->CTL |= PWM_CTL_CLRFIFO; //clear pwm
    usleep(100);
    
    pwmHeader->STA = PWM_STA_ERRS; //clear PWM errors
    usleep(100);
    
    pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(PWM_FIFO_SIZE) | PWM_DMAC_PANIC(PWM_FIFO_SIZE); //DREQ is activated at queue < PWM_FIFO_SIZE
    pwmHeader->RNG1 = 10; //used only for timing purposes; #writes to PWM FIFO/sec = PWM CLOCK / RNG1
    pwmHeader->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
    
    //allocate memory for the control blocks
    size_t cbPageBytes = numSrcBlocks * sizeof(struct DmaControlBlock) * 3; //3 cbs for each source block
    void *virtCbPageCached = makeLockedMem(cbPageBytes);
    void *virtCbPage = makeUncachedMemView(virtCbPageCached, cbPageBytes, memfd, pagemapfd);
    //fill the control blocks:
    struct DmaControlBlock *cbArrCached = (struct DmaControlBlock*)virtCbPageCached;
    struct DmaControlBlock *cbArr = (struct DmaControlBlock*)virtCbPage;
    printf("#dma blocks: %i, #src blocks: %i\n", numSrcBlocks*3, numSrcBlocks);
    for (int i=0; i<numSrcBlocks*3; i += 3) {
        //pace DMA through PWM
        cbArr[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
        cbArr[i].SOURCE_AD = virtToUncachedPhys(srcArrayCached + i/3, pagemapfd); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
        cbArr[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
        cbArr[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(1) | DMA_CB_TXFR_LEN_XLENGTH(4);
        cbArr[i].STRIDE = i/3;
        cbArr[i].NEXTCONBK = virtToUncachedPhys(cbArrCached+i+1, pagemapfd); //have to use the cached version because the uncached version isn't listed in pagemap(?)
        //copy buffer to GPIOs
        cbArr[i+1].TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
        cbArr[i+1].SOURCE_AD = virtToUncachedPhys(srcArrayCached + i/3, pagemapfd);
        cbArr[i+1].DEST_AD = GPIO_BASE_BUS + GPSET0;
        cbArr[i+1].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
        cbArr[i+1].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
        cbArr[i+1].NEXTCONBK = virtToUncachedPhys(cbArrCached+i+2, pagemapfd);
        //clear buffer (TODO: investigate using a 4-word copy ("burst") )
        cbArr[i+2].TI = DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
        cbArr[i+2].SOURCE_AD = virtToUncachedPhys(zerosPageCached, pagemapfd);
        cbArr[i+2].DEST_AD = virtToUncachedPhys(srcArrayCached + i/3, pagemapfd);
        cbArr[i+2].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(1) | DMA_CB_TXFR_LEN_XLENGTH(sizeof(struct GpioBufferFrame));
        cbArr[i+2].STRIDE = i/3; //might be better to use the NEXT index
        int nextIdx = i+3 < numSrcBlocks*3 ? i+3 : 0; //last block should loop back to the first block
        cbArr[i+2].NEXTCONBK = virtToUncachedPhys(cbArrCached + nextIdx, pagemapfd); //(uint32_t)physCbPage + ((void*)&cbArr[(i+2)%maxIdx] - virtCbPage);
    }
    for (int i=0; i<cbPageBytes; i+=PAGE_SIZE) {
        printf("virt cb[%i] -> phys: 0x%08x (0x%08x)\n", i, virtToPhys(i+(void*)cbArrCached, pagemapfd), virtToUncachedPhys(i+(void*)cbArrCached, pagemapfd));
    }
    //source: http://virtualfloppy.blogspot.com/2014/01/dma-support-at-last.html
    //cat /sys/module/dma/parameters/dmachans gives a bitmask of DMA channels that are not used by GPU. Results: ch 1, 3, 6, 7 are reserved.
    //dmesg | grep "DMA"; results: Ch 2 is used by SDHC host
    //ch 0 is known to be used for graphics acceleration
    //Thus, applications can use ch 4, 5, or the LITE channels @ 8 and beyond.
    //If using LITE channels, then we can't use the STRIDE feature, so that narrows it down to ch 4 and ch 5.
    int dmaCh = 5; 
    //enable DMA channel (it's probably already enabled, but we want to be sure):
    writeBitmasked(dmaBaseMem + DMAENABLE, 1 << dmaCh, 1 << dmaCh);
    
    //configure the DMA header to point to our control block:
    dmaHeader = (struct DmaChannelHeader*)(dmaBaseMem + DMACH(dmaCh)/4); //must divide by 4, as dmaBaseMem is uint32_t*
    printf("Previous DMA header:\n");
    logDmaChannelHeader(dmaHeader);
    //abort any previous DMA:
    //dmaHeader->NEXTCONBK = 0; //NEXTCONBK is read-only.
    dmaHeader->CS |= DMA_CS_ABORT; //make sure to disable dma first.
    usleep(100); //give time for the abort command to be handled.
    
    dmaHeader->CS = DMA_CS_RESET;
    usleep(100);
    
    writeBitmasked(&dmaHeader->CS, DMA_CS_END, DMA_CS_END); //clear the end flag
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    uint32_t firstAddr = virtToUncachedPhys(cbArrCached, pagemapfd);
    printf("starting DMA @ CONBLK_AD=0x%08x\n", firstAddr);
    dmaHeader->CONBLK_AD = firstAddr; //(uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
    dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG; //high priority (max is 7)
    dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG | DMA_CS_ACTIVE; //activate DMA. 
    
    uint64_t startTime = readSysTime(timerBaseMem);
    printf("DMA Active @ %llu uSec\n", startTime);
    /*while (dmaHeader->CS & DMA_CS_ACTIVE) {
        logDmaChannelHeader(dmaHeader);
    } //wait for DMA transfer to complete.*/
    for (int i=1; ; ++i) { //generate the output sequence:
        //logDmaChannelHeader(dmaHeader);
        //this just toggles outPin every few us:
        queue(outPin, i%2, startTime + 1000*i, srcArray, timerBaseMem, dmaHeader);
    }
    //Exit routine:
    cleanup();
    printf("Exiting cleanly:\n");
    freeUncachedMemView(virtCbPage, cbPageBytes);
    freeLockedMem(virtCbPageCached, cbPageBytes);
    freeUncachedMemView(virtSrcPage, srcPageBytes);
    freeLockedMem(virtSrcPageCached, srcPageBytes);
    freeUncachedMemView(zerosPage, PAGE_SIZE);
    freeLockedMem(zerosPageCached, PAGE_SIZE);
    close(pagemapfd);
    close(memfd);
    return 0;
}
