#ifndef DRIVERS_RPI_MITPI_H
#define DRIVERS_RPI_MITPI_H

#include <stdint.h> //for uint32_t

namespace mitpi {

enum GpioPin {
    //There are 2 board revisions (and now the model B+), which have slightly different I/O wiring.
    //P1 header (2x13 pins) for version 1:
    //looking down at the board with the P1 header in the upper right,
    //The upper-left pin is P1_01,
    //The upper-right pin is P1_02 (so, odds are left, evens are right)
    //numbering increases as you go vertically down 
    GPIO_P1_03     =  0, //(i.e., physical pin #3 is addressed in software as #0)
    GPIO_P1_05     =  1,
    GPIO_P1_07     =  4,
    GPIO_P1_08     = 14,
    GPIO_P1_10     = 15,
    GPIO_P1_11     = 17,
    GPIO_P1_12     = 18,
    GPIO_P1_13     = 21,
    GPIO_P1_15     = 22,
    GPIO_P1_16     = 23,
    GPIO_P1_18     = 24,
    GPIO_P1_19     = 10,
    GPIO_P1_21     =  9,
    GPIO_P1_22     = 25,
    GPIO_P1_23     = 11,
    GPIO_P1_24     =  8,
    GPIO_P1_26     =  7,

    //The P1 header for revision 2:
    //same physical numbering as before
    V2_GPIO_P1_03  =  2,
    V2_GPIO_P1_05  =  3,
    V2_GPIO_P1_07  =  4,
    V2_GPIO_P1_08  = 14,
    V2_GPIO_P1_10  = 15,
    V2_GPIO_P1_11  = 17,
    V2_GPIO_P1_12  = 18,
    V2_GPIO_P1_13  = 27,
    V2_GPIO_P1_15  = 22,
    V2_GPIO_P1_16  = 23,
    V2_GPIO_P1_18  = 24,
    V2_GPIO_P1_19  = 10,
    V2_GPIO_P1_21  =  9,
    V2_GPIO_P1_22  = 25,
    V2_GPIO_P1_23  = 11,
    V2_GPIO_P1_24  =  8,
    V2_GPIO_P1_26  =  7,

    //Revision 2 has another 2x4 header.
    //This doesn't have any pins soldered on by default, so (trivial) physical modification is required to use them
    //Physical numbering is ordered ... how?
    V2_GPIO_P5_03  = 28,
    V2_GPIO_P5_04  = 29,
    V2_GPIO_P5_05  = 30,
    V2_GPIO_P5_06  = 31,
};

enum GpioPull {
    //Internal pull-up / down resistors
    GPIOPULL_NONE = 0,
    GPIOPULL_DOWN = 1,
    GPIOPULL_UP   = 2,
};

volatile uint32_t* mapPeripheral(int memfd, int addr);

void init();

void makeOutput(int pin);
void makeInput(int pin);
void setPinHigh(int pin);
void setPinLow(int pin);
void setPinState(int pin, bool state);
bool readPinState(int pin);
void setPinPull(int pin, GpioPull pull);
void usleep(unsigned int us);

}


#endif
