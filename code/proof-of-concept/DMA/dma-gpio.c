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



int main() {
    //First, open the linux device, /dev/mem
    //dev/mem provides access to the physical memory of the entire processor+ram
    //This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        printf("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    ///dev/mem behaves as a file. We need to map that file into memory:
    void *gpioBaseMem = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE,
        MAP_SHARED,
        memfd, GPIO_BASE);
    //now, *gpioBaseMem = memory at physical address of GPIO_BASE.
    if (gpioBaseMem == MAP_FAILED) {
        printf("failed to map memory (did you remember to run as root?)\n");
        exit(1);
    }
    //now set our pin (#4) as an output:
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0 - GPIO_BASE);
    *fselAddr = (*fselAddr) | (1 << (3*4));
    //*(volatile uint32_t *)(gpioBaseMem + GPFSEL0 - GPIO_BASE) = 1 << (3*4);
    return 0;
}
