#include "mitpi.h"

#define GPIO_BASE 0x20200000 //base address of the GPIO control registers.
#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size
#define GPFSEL0   0x00000000 //gpio function select. There are 6 of these (32 bit registers)
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


#include <sys/mman.h> //for mmap
#include <sys/time.h> //for timespec
#include <time.h> //for timespec / nanosleep / usleep (need -std=gnu99)
#include <signal.h> //for sigaction
#include <unistd.h> //for NULL
//#include <stdio.h> //for printf
#include <stdlib.h> //for exit
#include <cassert>
#include <fcntl.h> //for file opening
#include <errno.h> //for errno
#include <pthread.h> //for pthread_setschedparam
#include <chrono>
#include "common/logging.h"


namespace mitpi {

volatile uint32_t *gpioBaseMem = NULL;

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

volatile uint32_t* mapPeripheral(int memfd, int addr) {
    ///dev/mem behaves as a file. We need to map that file into memory:
    //NULL = virtual address of mapping is chosen by kernel.
    //PAGE_SIZE = map 1 page.
    //PROT_READ|PROT_WRITE means give us read and write priveliges to the memory
    //MAP_SHARED means updates to the mapped memory should be written back to the file & shared with other processes
    //addr = offset in file to map
    void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, addr);
    //now, *mapped = memory at physical address of addr.
    if (mapped == MAP_FAILED) {
        LOGE("MitPi::mapPeripheral failed to map memory (did you remember to run as root?)\n");
        exit(1);
    } else {
        LOGV("MitPi::mapPeripheral mapped: %p\n", mapped);
    }
    return (volatile uint32_t*)mapped;
}

void init() {
    if (gpioBaseMem) {
        return; //already initialized
    }
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        LOGE("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    gpioBaseMem = mapPeripheral(memfd, GPIO_BASE);
}

void makeOutput(int pin) {
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0/4 + pin/10);
    writeBitmasked(fselAddr, 0x7 << (3*(pin%10)), 0x1 << (3*(pin%10))); //0b111 is bitmask to select just our pin, 0b001 is mode to make pin an output
}
void makeInput(int pin) {
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0/4 + pin/10);
    writeBitmasked(fselAddr, 0x7 << (3*(pin%10)), 0x0 << (3*(pin%10))); //0b111 is bitmask to select just our pin, 0b000 is mode to make pin an input
}
void setPinHigh(int pin) {
    //first, select the appropriate register. The first register controls pins 0-31, the second pins 32-63.
    volatile uint32_t *gpSetAddr = (volatile uint32_t*)(gpioBaseMem + GPSET0/4 + (pin/32));
    //now write a 1 ONLY to our pin. The act of writing a 1 to the address triggers it to be set high.
    *gpSetAddr = 1 << (pin & 31);
}
void setPinLow(int pin) {
    //first, select the appropriate register. The first register controls pins 0-31, the second pins 32-63.
    volatile uint32_t *gpClrAddr = (volatile uint32_t*)(gpioBaseMem + GPCLR0/4 + (pin/32));
    //now write a 1 ONLY to our pin. The act of writing a 1 to the address triggers it to be set high.
    *gpClrAddr = 1 << (pin & 31);
}
void setPinState(int pin, bool state) {
    state ? setPinHigh(pin) : setPinLow(pin);
}
bool readPinState(int pin) {
    volatile uint32_t* gpLevAddr = (volatile uint32_t*)(gpioBaseMem + GPLEV0/4 + (pin/32));
    uint32_t value = *gpLevAddr;
    return (value & (1 << (pin & 31))) ? 1 : 0;
}

}
