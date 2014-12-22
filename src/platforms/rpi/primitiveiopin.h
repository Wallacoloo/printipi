#ifndef PLATFORMS_RPI_PRIMITIVEIOPIN_H
#define PLATFORMS_RPI_PRIMITIVEIOPIN_H

#include "mitpi.h" //for GpioPin

namespace plat {
namespace rpi {

class PrimitiveIoPin {
	mitpi::GpioPin PinIdx;
    mitpi::GpioPull PullUpDown;
	public:
		inline static PrimitiveIoPin null() {
			return PrimitiveIoPin(mitpi::NULL_GPIO_PIN);
		}
		inline bool isNull() const {
			//cannot implicitly compare types, so we need an explicit `isNull` function
			return PinIdx == mitpi::NULL_GPIO_PIN;
		}
		inline PrimitiveIoPin(mitpi::GpioPin pinIdx, mitpi::GpioPull pullUpDown=mitpi::GPIOPULL_NONE)
		  : PinIdx(pinIdx), PullUpDown(pullUpDown) {
			mitpi::init();
		}
		inline mitpi::GpioPin id() const {
            return PinIdx;
        }
        //configure the pin as an output, and set its output state
		inline void makeDigitalOutput(IoLevel lev) {
            mitpi::makeOutput(PinIdx);
            digitalWrite(lev);
        }
	    //configure the pin to be an input
	    inline void makeDigitalInput() {
            mitpi::makeInput(PinIdx);
            mitpi::setPinPull(PinIdx, PullUpDown);
        }
	    //read the pin's input value (assumes pin is configured as digital)
	    inline IoLevel digitalRead() const {
            return mitpi::readPinState(PinIdx);
        }
        inline void digitalWrite(IoLevel lev) {
            mitpi::setPinState(PinIdx, lev);
        }
};

}
}


#endif