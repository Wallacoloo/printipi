#ifndef DRIVERS_RPI_PRIMITIVEIOPIN_H
#define DRIVERS_RPI_PRIMITIVEIOPIN_H

#include "mitpi.h" //for GpioPin

namespace drv {
namespace rpi {

class PrimitiveIoPin {
	mitpi::GpioPin PinIdx;
    mitpi::GpioPull PullUpDown;
	public:
		static PrimitiveIoPin null() {
			return PrimitiveIoPin(mitpi::NULL_GPIO_PIN);
		}
		PrimitiveIoPin(mitpi::GpioPin pinIdx, mitpi::GpioPull pullUpDown=mitpi::GPIOPULL_NONE)
		  : PinIdx(pinIdx), PullUpDown(pullUpDown) {
			mitpi::init();
		}
		GpioPinIdType id() const {
            return PinIdx;
        }
        //configure the pin as an output, and set its output state
		inline void makeDigitalOutput(IoLevel lev) {
            mitpi::makeOutput(PinIdx);
            digitalWrite(lev);
        }
	    //configure the pin to be an input
	    void makeDigitalInput() {
            mitpi::makeInput(PinIdx);
            mitpi::setPinPull(PinIdx, PullUpDown);
        }
	    //read the pin's input value (assumes pin is configured as digital)
	    IoLevel digitalRead() const {
            return mitpi::readPinState(PinIdx) ? IoHigh : IoLow;
        }
        void digitalWrite(IoLevel lev) {
            mitpi::setPinState(PinIdx, lev == IoHigh ? 1 : 0);
        }
};

}
}


#endif