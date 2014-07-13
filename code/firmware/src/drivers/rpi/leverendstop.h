#ifndef DRIVERS_RPI_LEVERENDSTOP_H
#define DRIVERS_RPI_LEVERENDSTOP_H

#include "drivers/endstop.h"
#include "drivers/rpi/rpi.h"
#include "logging.h"

namespace drv {
namespace rpi {


template <uint8_t Pin, bool ValueTriggered, bcm2835PUDControl PullUpDown=BCM2835_GPIO_PUD_OFF> class LeverEndstop : public Endstop {
	public:
		LeverEndstop() : Endstop(this) {
			bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_INPT);
			bcm2835_gpio_set_pud(Pin, PullUpDown);
		}
		static bool isTriggered() {
			bool t = bcm2835_gpio_lev(Pin) == ValueTriggered;
			LOGV("LeverEndstop: %i is %i\n", Pin, t);
			return t;
		}
};


}
}
#endif
