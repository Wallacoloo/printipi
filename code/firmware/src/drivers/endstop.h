#ifndef DRIVERS_ENDSTOP_H
#define DRIVERS_ENDSTOP_H

/* 
 * Printipi/drivers/axisstepper.h
 * (c) 2014 Colin Wallace
 *
 * Endstops are queriable switches placed at the axis limits.
 * They typically represent a "known" point to which the device can be homed upon initiailization,
 *   or a point beyond which the device should not be pushed.
 */

#include "iodriver.h"
#include "iopin.h"

namespace drv {


template <typename Pin> class Endstop : public IODriver {
	Pin pin;
	public:
		Endstop() : IODriver(this) {
			//initIO();
			//bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_INPT);
			//bcm2835_gpio_set_pud(Pin, PullUpDown);
			pin.makeDigitalInput();
		}
		bool isTriggered() {
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
//template <typename Pin> Pin Endstop<Pin>::pin;

//default Endstop implementation which always acts as if untriggered:
typedef Endstop<NoPin> EndstopNoExist;

}
#endif
