#ifndef DRIVERS_ENABLEDISABLEDRIVER_H
#define DRIVERS_ENABLEDISABLEDRIVER_H

/* EnableDisableDriver:
 * Provides an interface for controlling an ENABLE pin on an IC.
 * Possible use-case: disabling stepper drivers when inactive, and re-enabling them when active.
*/

#include "logging.h"

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

struct NullEnabler : public EnableDisableDriver {
	NullEnabler() : EnableDisableDriver(this) {}
	/*static void deactivate() {
		LOGV("NullEnabler::deactivate()\n");
	}*/
};


}

#endif
