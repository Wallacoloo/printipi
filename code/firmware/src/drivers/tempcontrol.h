#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

#include "drivers/iodriver.h"

namespace drv {

template <typename IOFace, typename Thermistor> class TempControl : public IODriver {
	Thermistor _therm;
	float _lastTemp;
	public:
		TempControl() : IODriver(this), _lastTemp(0) {
		}
		void onIdleCpu() const {
			LOGV("TempControl::onIdleCpu()\n");
			//_therm.onIdleCp
		}
		float getLastTemp() const {
			return _lastTemp;
		}
};

}
#endif
