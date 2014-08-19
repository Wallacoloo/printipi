/* (C) 2014 Colin Wallace
 * MIT License
 */
/*
 * processor documentation is at: http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * pg 38 for DMA
 * pg 89 for gpio
 *
 * The general idea is to have a buffer of N blocks, where each block is the same size as the gpio registers, 
 *   and have the DMA module continually copying the data in this buffer into those registers.
 * In this way, we can have (say) 32 blocks, and then be able to buffer the next 32 IO frames.
 */
 
#include <sys/mman.h> //for mmap
#include <unistd.h> //for NULL
#include <stdio.h> //for printf
#include <stdlib.h> //for exit
#include <fcntl.h> //for file opening
#include <stdint.h> //for uint32_t
 

#define GPIO_BASE 0x20200000 //base address of the GPIO control registers.
#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size
#define GPFSEL0   0x20200000 //gpio function select. There are 6 of these (32 bit registers)
#define GPFSEL1   0x20200004
#define GPFSEL2   0x20200008
#define GPFSEL3   0x2020000c
#define GPFSEL4   0x20200010
#define GPFSEL5   0x20200014
//bits 2-0 of GPFSEL0: set to 000 to make Pin 0 an output. 001 is an input. Other combinations represent alternate functions
//bits 3-5 are for pin 1.
//...
//bits 27-29 are for pin 9.
//GPFSEL1 repeats, but bits 2-0 are Pin 10, 27-29 are pin 19.
//...
#define GPSET0    0x2020001C //GPIO Pin Output Set. There are 2 of these (32 bit registers)
//writing a '1' to bit N of GPSET0 makes that pin HIGH.
//writing a '0' has no effect.
//GPSET0[0-31] maps to pins 0-31
//GPSET1[0-21] maps to pins 32-53
#define GPCLR0    0x20200028 //GPIO Pin Output Clear. There are 2 of these (32 bits each)
//GPCLR acts the same way as GPSET, but clears the pin instead.
#define GPLEV0    0x20200034 //GPIO Pin Level. There are 2 of these (32 bits each)

#define DMA_BASE 0x20207000
#define DMACH0   0x20207000
#define DMACH1   0x20207100
#define DMACH2   0x20207200
#define DMACH3   0x20207300
//...
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used.

//DMA Control Block
struct DmaCb {
    uint32_t TI; //transfer information
        //31:27 unused
        //26    NO_WIDE_BURSTS
        //21:25 WAITS; number of cycles to wait between each DMA read/write operation
        //16:20 PERMAP; peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
        //12:15 BURST_LENGTH
        //11    SRC_IGNORE; set to 1 to not perform reads. Used to manually fill caches
        //10    SRC_DREQ; set to 1 to have the DREQ from PERMAP gate requests.
        //9     SRC_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //8     SRC_INC;   set to 1 to automatically increment the source address after each read (TODO: isn't this kind of pertinent?)
        //7     DEST_IGNORE; set to 1 to not perform writes.
        //6     DEST_DREG; set to 1 to have the DREQ from PERMAP gate *writes*
        //5     DEST_WIDTH
        //4     DEST_INC
        //3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
        //2     unused (0)
        //1     TDMODE; set to 1 to enable 2D mode
        //0     INTEN;  set to 1 to generate an interrupt upon completion
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Dest address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
};


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

int main() {
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
    
    //now set our pin (#4) as an output:
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0 - GPIO_BASE);
    uint32_t fselMask = 0x7 << (3*4); //bitmask for the 3 bits that control pin 4
    uint32_t fselValue = 0x1 << (3*4); //value that we want to give the above bitmask (0b001 = set mode to output)
    *fselAddr = ((*fselAddr) & ~fselMask) | fselValue; //set pin 4 to be an output.
    
    //configure DMA:
    struct DmaCb cb1;
    
    return 0;
}
