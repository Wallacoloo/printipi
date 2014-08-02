#ifndef DRIVERS_FAN_H
#define DRIVERS_FAN_H

/* 
 * Printipi/drivers/axisstepper.h
 * (c) 2014 Colin Wallace
 *
 * The Fan class serves to control a physical Fan, often used to cool cpu components.
 * This class essentially wraps another IO driver (like /src/drivers/rpi/onepiniodriver.h) so that it can be commanded as a fan
 */
 
#include "iodriver.h"

namespace drv {

template <typename Driver> class Fan : public IODriver {
	Driver driver;
	public:
		Fan() : IODriver(this) {}
		constexpr bool isFan() { return true; }
		//forward output control:
		void stepForward() {
			driver.stepForward();
		}
		void stepBackward() {
			driver.stepBackward();
		}
};

}
#endif
