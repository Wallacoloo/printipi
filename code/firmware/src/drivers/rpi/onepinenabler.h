#ifndef DRIVERS_RPI_ONEPINENABLER_H
#define DRIVERS_RPI_ONEPINENABLER_H

#include "drivers/enabledisabledriver.h"

namespace drv {
namespace rpi {

template <uint8_t Pin, bool ValueEn, bool ValueDis=!ValueEn> class OnePinEnabler : public EnableDisableDriver {
	public:
		OnePinEnabler() : EnableDisableDriver(this) {
			initIO();
		}
		static void enable() {
			EnableDisableDriver::enable();
			bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_OUTP); //configure this pins as output
			bcm2835_gpio_write(Pin, ValueEn);
		}
		static void disable() {
			EnableDisableDriver::disable();
			bcm2835_gpio_write(Pin, ValueDis);
		}
		/*static void deactivate() { //called on shutdown.
			disable();
		}*/
};

}
}


#endif
