#ifndef DRIVERS_RPI_LEVERENDSTOP_H
#define DRIVERS_RPI_LEVERENDSTOP_H

#include "drivers/endstop.h"
//#include "drivers/rpi/rpi.h"
//#include "common/logging.h"

namespace drv {
namespace rpi {


template <typename Pin> class LeverEndstop : public Endstop {
	static Pin pin;
	public:
		LeverEndstop() : Endstop(this) {
			//initIO();
			//bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_INPT);
			//bcm2835_gpio_set_pud(Pin, PullUpDown);
			pin.makeDigitalInput();
		}
		static bool isTriggered() {
			//uint8_t raw = bcm2835_gpio_lev(Pin);
			//bool t = raw == ValueTriggered;
			bool t = pin.digitalRead() == IoHigh;
			//LOGV("LeverEndstop: %i is %i (bool: %i)\n", Pin, raw, t);
			/*raw = bcm2835_gpio_lev(Pin);
			t = raw == ValueTriggered;
			LOGV("LeverEndstop: %i is %i (bool: %i)\n", Pin, raw, t);*/
			return t;
		}
};

template <typename Pin> Pin LeverEndstop<Pin>::pin;

}
}
#endif
