#ifndef DRIVERS_ENABLEDISABLEDRIVER_H
#define DRIVERS_ENABLEDISABLEDRIVER_H

/* 
 * Printipi/drivers/enabledisabledriver.h
 * (c) 2014 Colin Wallace
 * 
 * Provides an interface for controlling an ENABLE pin on an IC.
 * Possible use-case: disabling stepper drivers when inactive, and re-enabling them when active.
 *
 * Note: EnableDisableDriver is an interface, and not an implementation.
 * An implementation is needed for each device to be enabled - eg the fan.
 * These implementations must provide the functions outlined further down in the header.
 */

#include "common/logging.h"

namespace drv {

class EnableDisableDriver : public IODriver {
	public:
		template <typename ThisT> EnableDisableDriver(ThisT *_this) : IODriver(_this) {}
		inline static void enable() {
			//LOGD("EnableDisablerDriver::enable()\n");
		}
		inline static void disable() {
			//LOGD("EnableDisablerDriver::disable()\n");
		}
};

//Default implementation for a device that cannot be enabled/disabled.
struct NullEnabler : public EnableDisableDriver {
	NullEnabler() : EnableDisableDriver(this) {}
};


}

#endif
