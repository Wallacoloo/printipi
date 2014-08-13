#ifndef DRIVERS_ENABLEDISABLEDRIVER_H
#define DRIVERS_ENABLEDISABLEDRIVER_H

/* 
 * Printipi/drivers/enabledisabledriver.h
 * (c) 2014 Colin Wallace
 * 
 * Provides an interface for controlling an ENABLE pin on an IC.
 * Possible use-case: disabling stepper drivers when inactive, and re-enabling them when active.
 */

#include "common/logging.h"

namespace drv {

template <typename Pin> class EnableDisableDriver : public IODriver {
	Pin pin;
	public:
		EnableDisableDriver() : IODriver() {
			pin.makeDigitalOutput(IoLow);
		}
		inline void enable() {
			pin.digitalWrite(IoHigh);
		}
		inline void disable() {
			pin.digitalWrite(IoLow);
		}
};
//template <typename Pin> Pin EnableDisableDriver<Pin>::pin;

//Default implementation for a device that cannot be enabled/disabled.
typedef EnableDisableDriver<NoPin> NullEnabler;


}
#endif
