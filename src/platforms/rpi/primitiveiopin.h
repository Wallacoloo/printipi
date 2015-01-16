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

#ifndef PLATFORMS_RPI_PRIMITIVEIOPIN_H
#define PLATFORMS_RPI_PRIMITIVEIOPIN_H

#include <cassert>

#include "mitpi.h" //for GpioPin
#include "hardwarescheduler.h" //for ability to schedule PWM
//for MAX_RPI_PIN_ID
#include "compileflags.h"

namespace plat {
namespace rpi {

class PrimitiveIoPin {
	//logical index of this pin (0-53, I believe)
	mitpi::GpioPin pinIdx;
	public:
		//@return a pin with an invalid output
		//
		//Note that it is undefined to call any function on a null pin except for <isNull> and <id>.
		inline static PrimitiveIoPin null() {
			return PrimitiveIoPin(mitpi::NULL_GPIO_PIN);
		}
		//@return true if the pin is equal to the one constructed by the <null> function.
		inline bool isNull() const {
			return pinIdx == mitpi::NULL_GPIO_PIN;
		}
		//@pinIdx *logical* index of the pin
		//@pullUpDown direct this pin to either pull up to 3.3v, down to gnd, or no pull at all.
		//Note that this pull direction will be applied **even when operating as an output pin**.
		// The rpi's pull resistors preserve state across reboots.
		// This makes it so that when the Pi resets, its pins will still be in a defined state (no runaway heater while the Pi is still booting)
		// Of course, it's still a VERY GOOD IDEA to use hardware pull resistors as well.
		//**Also note:** @pullUpDown acts the same regardless of the <IoPin>'s read/write inversions.
		inline PrimitiveIoPin(mitpi::GpioPin pinIdx, mitpi::GpioPull pullUpDown=mitpi::GPIOPULL_NONE)
		  : pinIdx(pinIdx) {
		  	assert((pinIdx == mitpi::NULL_GPIO_PIN || pinIdx <= MAX_RPI_PIN_ID) && "Make sure to appropriately set MAX_RPI_PIN_ID (see compileflags.h) or else some pins might not behave correctly");
			if (pinIdx != mitpi::NULL_GPIO_PIN) {
				mitpi::init();
				//we CANNOT set the pin pull of a null gpio pin
				mitpi::setPinPull(pinIdx, pullUpDown);
			}
		}
		//@return the logical index of the pin (for use with other platform-specific functions)
		inline mitpi::GpioPin id() const {
            return pinIdx;
        }
        //configure the pin as an output, and set its output state
		inline void makeDigitalOutput(IoLevel lev) {
            mitpi::makeOutput(pinIdx);
            digitalWrite(lev);
        }
        //configure the pin as a PWM output & set its duty cycle and period (if applicable)
        inline void makePwmOutput(float duty, float desiredPeriod) {
        	mitpi::makeOutput(pinIdx);
        	pwmWrite(duty, desiredPeriod);
        }
	    //configure the pin to be an input
	    inline void makeDigitalInput() {
            mitpi::makeInput(pinIdx);
        }
	    //read the pin's input value (must call makeDigitalInput beforehand)
	    inline IoLevel digitalRead() const {
            return mitpi::readPinState(pinIdx);
        }
        //write a digital value to the pin. IoHigh = 3.3v, IoLow = 0v. Must call makeDigitalOutput beforehand
        inline void digitalWrite(IoLevel lev) {
            mitpi::setPinState(pinIdx, lev);
        }
        //set pwm duty cycle & period (if applicable). Must call makePwmOutput beforehand.
        inline void pwmWrite(float duty, float desiredPeriod) {
        	HardwareScheduler().queuePwm(*this, duty, desiredPeriod);
        }
};

}
}


#endif