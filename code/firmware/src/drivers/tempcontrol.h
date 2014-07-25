#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

#include "drivers/iodriver.h"

namespace drv {

template <typename IOFace, typename Thermistor> class TempControl : public IODriver {
	public:
		TempControl() : IODriver(this) {
		}
		float getLastTemp() const {
			return 0;
		}
};

}
#endif
