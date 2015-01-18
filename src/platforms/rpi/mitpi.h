/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Note: BCM2835 documentation available: http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * Additional Docs (GPIO Pads - hysteresis control, etc): http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
 */
 

#ifndef PLATFORMS_RPI_MITPI_H
#define PLATFORMS_RPI_MITPI_H

//for uint32_t
#include <stdint.h>

/*
 * Mitpi is a small library used for interfacing with the Raspberry Pi's peripherals, including GPIO and timers,
 *   licensed under the extremely permissive MIT license.
 *
 * Please note that most of its features require superuser priviledges, so you may have to launch any executable which uses this library with `sudo'
 */
namespace mitpi {

//Map physical pin locations to logical pin ids used by the processor
enum GpioPin {
    //There are 2 board revisions (and now the model A+/B+), which have slightly different I/O wiring.
    //P1 header (2x13 pins) for version 1 (i.e. before Sept 2012:
    //looking down at the board with the P1 header in the upper right,
    //The upper-left pin is P1_01,
    //The upper-right pin is P1_02 (so, odds are left, evens are right)
    //numbering increases as you go vertically down 
    V1_GPIO_P1_03     =  0, //(i.e., physical pin #3 is addressed in software as #0)
    V1_GPIO_P1_05     =  1,
    V1_GPIO_P1_07     =  4,
    V1_GPIO_P1_08     = 14,
    V1_GPIO_P1_10     = 15,
    V1_GPIO_P1_11     = 17,
    V1_GPIO_P1_12     = 18,
    V1_GPIO_P1_13     = 21,
    V1_GPIO_P1_15     = 22,
    V1_GPIO_P1_16     = 23,
    V1_GPIO_P1_18     = 24,
    V1_GPIO_P1_19     = 10,
    V1_GPIO_P1_21     =  9,
    V1_GPIO_P1_22     = 25,
    V1_GPIO_P1_23     = 11,
    V1_GPIO_P1_24     =  8,
    V1_GPIO_P1_26     =  7,

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

    //Revision 2 has another 2x4 header called 'P5'.
    //This doesn't have any pins soldered on by default, so (trivial) physical modification is required to use them
    //Physical numbering is ordered ... how?
    V2_GPIO_P5_03  = 28,
    V2_GPIO_P5_04  = 29,
    V2_GPIO_P5_05  = 30,
    V2_GPIO_P5_06  = 31,

    //The addition of the model A+ and B+ brought a 40-pin GPIO header.
    //The first 26 pins are identical to the model A/B Revision 2, with the latter 14 exposing additional GPIOs
    PLUS_GPIO_P1_03  =  2,
    PLUS_GPIO_P1_05  =  3,
    PLUS_GPIO_P1_07  =  4,
    PLUS_GPIO_P1_08  = 14,
    PLUS_GPIO_P1_10  = 15,
    PLUS_GPIO_P1_11  = 17,
    PLUS_GPIO_P1_12  = 18,
    PLUS_GPIO_P1_13  = 27,
    PLUS_GPIO_P1_15  = 22,
    PLUS_GPIO_P1_16  = 23,
    PLUS_GPIO_P1_18  = 24,
    PLUS_GPIO_P1_19  = 10,
    PLUS_GPIO_P1_21  =  9,
    PLUS_GPIO_P1_22  = 25,
    PLUS_GPIO_P1_23  = 11,
    PLUS_GPIO_P1_24  =  8,
    PLUS_GPIO_P1_26  =  7,
    PLUS_GPIO_P1_29  =  5,
    PLUS_GPIO_P1_31  =  6,
    PLUS_GPIO_P1_32  = 12,
    PLUS_GPIO_P1_33  = 13,
    PLUS_GPIO_P1_35  = 19,
    PLUS_GPIO_P1_36  = 16,
    PLUS_GPIO_P1_37  = 26,
    PLUS_GPIO_P1_38  = 20,
    PLUS_GPIO_P1_40  = 21,

    NULL_GPIO_PIN = 127,
};

//Internal pull-up / down resistors
enum GpioPull {
    GPIOPULL_NONE = 0,
    GPIOPULL_DOWN = 1,
    GPIOPULL_UP   = 2,
};

//Each bank of pins can be set to have different drive strengths, hysteresis and slew.
//pass a bitwise-or'd collection of flags to <setPadProperties>
enum GpioPadProperties {
    //Per-pin drive strength. Defaults to 8 mA at boot.
    //Note: avoid drawing a total of > 50 mA across ALL gpio pins / 3v3 supply
    PAD_DRIVE_2MA  = 0,
    PAD_DRIVE_4MA  = 1,
    PAD_DRIVE_6MA  = 2,
    PAD_DRIVE_8MA  = 3,
    PAD_DRIVE_10MA = 4,
    PAD_DRIVE_12MA = 5,
    PAD_DRIVE_14MA = 6,
    PAD_DRIVE_16MA = 7,
    //Hysteresis settings:
    PAD_HYSTERESIS_DIS = 0,
    PAD_HYSTERESIS_EN  = (1 << 3),
    //Slew rate limit
    //From Wikipedia: Slew rate is defined as the maximum rate of change of output voltage per unit of time and is expressed as volt per second.
    PAD_SLEW_LIMIT =   0,
    PAD_SLEW_NO_LIMIT = (1 << 4),
};

typedef unsigned int PinIntType;

volatile uint32_t* mapPeripheral(int memfd, int addr);

bool init();

void makeOutput(PinIntType pin);
void makeInput(PinIntType pin);
void setPinHigh(PinIntType pin);
void setPinLow(PinIntType pin);
void setPinState(PinIntType pin, bool state);
bool readPinState(PinIntType pin);
void setPinPull(PinIntType pin, GpioPull pull);
//Set the drive strength, hysteresis and slew limits for a set of pins.
//@flags bitwise-or of GpioPadProperties flags
//@bank 0 for pins 0-27, 1 -> pins 28-45, 2 -> pins 46-53
void setPadProperties(uint32_t flags, int bank);
void usleep(unsigned int us);
uint64_t readSysTime();


//convenience type used to manage lifetime of mitpi.
//Can be used as a member object of a type that depends on Mitpi functionality.
//Example:
//class X {
//    InitMitpiType mitpiDependency;
//    void doStuff() {
//        mitpi::makeOutput(11);
//        //Note the lack of explicit mitpi initialization. This is done with the InitMitpiType above
//    }
//};
struct InitMitpiType {
    inline InitMitpiType() {
        init();
    }
};

}


#endif
