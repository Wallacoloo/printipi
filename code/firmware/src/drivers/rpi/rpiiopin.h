#ifndef DRIVERS_RPI_RPIIOPIN_H
#define DRIVERS_RPI_RPIIOPIN_H

#include "drivers/iopin.h"
#include "drivers/rpi/rpi.h"

namespace drv {
namespace rpi {

template <uint8_t PinIdx> class RpiIoPin : public IoPin {
	public:
		RpiIoPin() {
			initIO();
		}
		void makeDigitalOutput(IoLevel lev) {
			bcm2835_gpio_fsel(PinIdx, BCM2835_GPIO_FSEL_OUTP); //configure this pin as output
			setValue(lev);
		}
		void makeDigitalInput() {
			bcm2835_gpio_fsel(PinIdx, BCM2835_GPIO_FSEL_INPT); //configure this pin as input
		}
		IoLevel digitalRead() const {
			return bcm2835_gpio_lev(PinIdx) ? IoHigh : IoLow;
		}
		void digitalWrite(IoLevel lev) {
			bcm2835_gpio_write(PinIdx, lev == IoHigh ? HIGH : LOW);
		}

};


}
}
#endif
