#ifndef DRIVERS_FAN_H
#define DRIVERS_FAN_H

/* 
 * Printipi/drivers/fan.h
 * (c) 2014 Colin Wallace
 *
 * The Fan class serves to control a physical Fan, often used to cool cpu components.
 * This class essentially wraps an IoPin so that it can be commanded as a fan
 */
 
#include "iodriver.h"

namespace drv {

template <typename Driver> class Fan : public IODriver {
	Driver driver;
	public:
		Fan() : IODriver() {
			driver.makeDigitalOutput(IoLow);
		}
		constexpr bool isFan() { return true; }
		//constexpr float fanPwmPeriod() { return 0.1; }
		//forward output control:
		void stepForward() {
			driver.digitalWrite(IoHigh);
		}
		void stepBackward() {
			driver.digitalWrite(IoLow);
		}
};

}
#endif
