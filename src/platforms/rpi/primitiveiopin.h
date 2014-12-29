#ifndef PLATFORMS_RPI_PRIMITIVEIOPIN_H
#define PLATFORMS_RPI_PRIMITIVEIOPIN_H

#include "mitpi.h" //for GpioPin

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
			mitpi::init();
			mitpi::setPinPull(pinIdx, pullUpDown);
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
};

}
}


#endif