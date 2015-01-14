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
#define GPPUD     0x00000094 //GPIO Pull-up/down register. Write 1 for pd, 2 for pu, and then write the PUDCLK
#define GPPUDCLK0 0x00000098 //GPIO Pull-up/down clock. Have to send a clock signal to the pull-up/down resistors to activate them.
#define GPPUDCLK1 0x000000a0 //second register for GPPUDCLK (first is for pins 0-31, 2nd for the rest)

//GPIO_PADS documentation from http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
#define GPIO_PADS_BASE    0x20100000
#define GPIO_PAD0         0x0000002c //address for hysteresis / slew rate / drive strength settings
#define GPIO_PADS_PASSWRD 0x5A //any write to GPIO_PADS must be or'd with the password to prevent accidental writes

#define TIMER_BASE 0x20003000
#define TIMER_CLO 0x00000004 //lower 32-bits of 1 MHz timer
#define TIMER_CHI 0x00000008 //upper 32-bits

#include <sys/mman.h> //for mmap
#include <sys/time.h> 
#include <time.h> //for nanosleep / usleep (if have -std=gnu99)
#include <unistd.h> //for usleep
#include <stdlib.h> //for exit
#include <cassert> //for assert
#include <fcntl.h> //for file opening
#include "common/logging.h"


namespace mitpi {

static volatile uint32_t *gpioBaseMem    = nullptr;
static volatile uint32_t *timerBaseMem   = nullptr;
static volatile uint32_t *gpioPadBaseMem = nullptr;

static void assertValidPin(int pin) {
    (void)pin; //unused when assertions are disabled.
    assert(pin >= 0 && pin < 64);
}

static void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
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
    void *mapped = mmap(nullptr, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, addr);
    //now, *mapped = memory at physical address of addr.
    if (mapped == MAP_FAILED) {
        LOGE("MitPi::mapPeripheral failed to map memory (did you remember to run as root?)\n");
        exit(1);
    } else {
        LOGV("MitPi::mapPeripheral mapped: %p\n", mapped);
    }
    return (volatile uint32_t*)mapped;
}

bool init() {
    if (gpioBaseMem) {
        return 0; //already initialized
    }
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        LOGE("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    gpioBaseMem =    mapPeripheral(memfd, GPIO_BASE);
    timerBaseMem =   mapPeripheral(memfd, TIMER_BASE);
    gpioPadBaseMem = mapPeripheral(memfd, GPIO_PADS_BASE);
    //disable hysteresis & slew limiting for user pins. Enable for 8 mA sink/source capability per-pin
    setPadProperties(PAD_DRIVE_8MA | PAD_HYSTERESIS_DIS | PAD_SLEW_NO_LIMIT, 0);
    setPadProperties(PAD_DRIVE_8MA | PAD_HYSTERESIS_DIS | PAD_SLEW_NO_LIMIT, 1);
    return 0; //init OK
}

void makeOutput(int pin) {
    assertValidPin(pin);
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0/4 + pin/10);
    writeBitmasked(fselAddr, 0x7 << (3*(pin%10)), 0x1 << (3*(pin%10))); //0x7  is bitmask to select just our pin, 0x1 is mode to make pin an output
}
void makeInput(int pin) {
    assertValidPin(pin);
    volatile uint32_t *fselAddr = (volatile uint32_t*)(gpioBaseMem + GPFSEL0/4 + pin/10);
    writeBitmasked(fselAddr, 0x7 << (3*(pin%10)), 0x0 << (3*(pin%10))); //0x7 is bitmask to select just our pin, 0x0 is mode to make pin an input
}
void setPinHigh(int pin) {
    assertValidPin(pin);
    //first, select the appropriate register. The first register controls pins 0-31, the second pins 32-63.
    volatile uint32_t *gpSetAddr = (volatile uint32_t*)(gpioBaseMem + GPSET0/4 + (pin/32));
    //now write a 1 ONLY to our pin. The act of writing a 1 to the address triggers it to be set high.
    *gpSetAddr = 1 << (pin & 31);
}
void setPinLow(int pin) {
    assertValidPin(pin);
    //first, select the appropriate register. The first register controls pins 0-31, the second pins 32-63.
    volatile uint32_t *gpClrAddr = (volatile uint32_t*)(gpioBaseMem + GPCLR0/4 + (pin/32));
    //now write a 1 ONLY to our pin. The act of writing a 1 to the address triggers it to be set high.
    *gpClrAddr = 1 << (pin & 31);
}
void setPinState(int pin, bool state) {
    state ? setPinHigh(pin) : setPinLow(pin);
}
bool readPinState(int pin) {
    assertValidPin(pin);
    volatile uint32_t* gpLevAddr = (volatile uint32_t*)(gpioBaseMem + GPLEV0/4 + (pin/32));
    uint32_t value = *gpLevAddr;
    return (value & (1 << (pin & 31))) ? 1 : 0;
}

void setPinPull(int pin, GpioPull pull) {
    assertValidPin(pin);
    volatile uint32_t *pudAddr = (volatile uint32_t*)(gpioBaseMem + GPPUD/4);
    volatile uint32_t *pudClkAddr = (volatile uint32_t*)(gpioBaseMem + GPPUDCLK0/4 + pin/32);
    *pudAddr = pull;
    usleep(10);
    *pudClkAddr = 1 << (pin & 31);
    usleep(10);
    *pudAddr = GPIOPULL_NONE;
    *pudClkAddr = 0;
}

void setPadProperties(uint32_t flags, int bank) {
    assert(0 <= bank && bank <= 2);
    volatile uint32_t *padAddr = (volatile uint32_t*)(gpioPadBaseMem + GPIO_PAD0/4 + bank);
    *padAddr = (flags | GPIO_PADS_PASSWRD);
}

void usleep(unsigned int us) {
    //explicitly exposed to allow users of the library access to a usleep function
    ::usleep(us);
}

uint64_t readSysTime() {
    return ((uint64_t)*(timerBaseMem + TIMER_CHI/4) << 32) + (uint64_t)(*(timerBaseMem + TIMER_CLO/4));
}

}
