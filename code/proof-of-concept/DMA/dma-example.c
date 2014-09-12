/* This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

/*
 * processor documentation is at: http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * pg 38 for DMA
 */
 
#include <sys/mman.h> //for mmap
#include <unistd.h> //for NULL
#include <stdio.h> //for printf
#include <stdlib.h> //for exit
#include <fcntl.h> //for file opening
#include <stdint.h> //for uint32_t

#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size

//physical addresses for the DMA peripherals, as found in the processor documentation:
#define DMA_BASE 0x20007000
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
#define DMACH0   0x20007000
#define DMACH1   0x20007100
#define DMACH2   0x20007200
#define DMACH3   0x20007300
//...
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x20007ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC (1<<4)
#define DMA_CB_TI_SRC_INC (1<<8)

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
    uint32_t CS; //Control and Status
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
    uint32_t CONBLK_AD; //Control Block Address
    uint32_t TI; //transfer information; see DmaControlBlock.TI for description
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t DEBUG; //controls debug settings
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
        //6     DEST_DREG; set to 1 to have the DREQ from PERMAP gate *writes*
        //5     DEST_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //4     DEST_INC;   set to 1 to automatically increment the destination address after each read (Tyou'll want this if you're copying a range of memory)
        //3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
        //2     unused (0)
        //1     TDMODE; set to 1 to enable 2D mode
        //0     INTEN;  set to 1 to generate an interrupt upon completion
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
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
    volatile uint32_t *dmaBaseMem = mapPeripheral(memfd, DMA_BASE);
    
    //configure DMA:
    //allocate 1 page for the source and 1 page for the destination:
    void *virtSrcPage, *physSrcPage;
    makeVirtPhysPage(&virtSrcPage, &physSrcPage);
    void *virtDestPage, *physDestPage;
    makeVirtPhysPage(&virtDestPage, &physDestPage);
    
    //write a few bytes to the source page:
    char *srcArray = (char*)virtSrcPage;
    srcArray[0]  = 'h';
    srcArray[1]  = 'e';
    srcArray[2]  = 'l';
    srcArray[3]  = 'l';
    srcArray[4]  = 'o';
    srcArray[5]  = ' ';
    srcArray[6]  = 'w';
    srcArray[7]  = 'o';
    srcArray[8]  = 'r';
    srcArray[9]  = 'l';
    srcArray[10] = 'd';
    srcArray[11] =  0; //null terminator used for printf call.
    
    //allocate 1 page for the control blocks
    void *virtCbPage, *physCbPage;
    makeVirtPhysPage(&virtCbPage, &physCbPage);
    
    //dedicate the first 8 bytes of this page to holding the cb.
    struct DmaControlBlock *cb1 = (struct DmaControlBlock*)virtCbPage;
    
    //fill the control block:
    cb1->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each 4-byte copy, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
    cb1->SOURCE_AD = (uint32_t)physSrcPage; //set source and destination DMA address
    cb1->DEST_AD = (uint32_t)physDestPage;
    cb1->TXFR_LEN = 12; //transfer 12 bytes
    cb1->STRIDE = 0; //no 2D stride
    cb1->NEXTCONBK = 0; //no next control block
    
    printf("destination was initially: '%s'\n", (char*)virtDestPage);
    
    //enable DMA channel (it's probably already enabled, but we want to be sure):
    writeBitmasked(dmaBaseMem + DMAENABLE - DMA_BASE, 1 << 3, 1 << 3);
    
    //configure the DMA header to point to our control block:
    volatile struct DmaChannelHeader *dmaHeader = (volatile struct DmaChannelHeader*)(dmaBaseMem + DMACH3 - DMA_BASE);
    dmaHeader->CS = DMA_CS_RESET; //make sure to disable dma first.
    sleep(1); //give time for the reset command to be handled.
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    dmaHeader->CONBLK_AD = (uint32_t)physCbPage; //we have to point it to the PHYSICAL address of the control block (cb1)
    dmaHeader->CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.
    
    sleep(1); //give time for copy to happen
    
    printf("destination reads: '%s'\n", (char*)virtDestPage);
    
    //cleanup
    freeVirtPhysPage(virtCbPage);
    freeVirtPhysPage(virtDestPage);
    freeVirtPhysPage(virtSrcPage);
    return 0;
}
