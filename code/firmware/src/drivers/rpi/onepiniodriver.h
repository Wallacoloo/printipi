#ifndef DRIVERS_RPI_ONEPINIODRIVER_H
#define DRIVERS_RPI_ONEPINIODRIVER_H

//#include "drivers/enabledisabledriver.h"
#include "drivers/iodriver.h"
#include "drivers/rpi/rpi.h"

namespace drv {
namespace rpi {

template <uint8_t Pin, bool ValueHigh, bool ValueLow=!ValueHigh, bool ValueDis=!ValueHigh> class OnePinIODriver : public IODriver {
	public:
		OnePinIODriver() : IODriver(this) {
			initIO();
			bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_OUTP); //configure this pins as output
			bcm2835_gpio_write(Pin, ValueDis);
		}
		static void deactivate() {
			bcm2835_gpio_write(Pin, ValueDis);
		}
		void stepForward() {
			bcm2835_gpio_write(Pin, ValueHigh);
		}
		void stepBackward() {
			bcm2835_gpio_write(Pin, ValueLow);
		}
		/*static void deactivate() { //called on shutdown.
			disable();
		}*/
};

}
}


#endif
