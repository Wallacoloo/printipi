#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

#include "drivers/iodriver.h"
#include "timeutil.h"

namespace drv {

template <typename IOFace, typename Thermistor> class TempControl : public IODriver {
	Thermistor _therm;
	float _lastTemp;
	bool _isReading;
	struct timespec _nextReadTime;
	struct timespec _interval;
	public:
		TempControl() : IODriver(this), _lastTemp(0), _isReading(false), _nextReadTime(timespecNow()), _interval{1, 0} {
		}
		bool onIdleCpu() {
			//LOGV("TempControl::onIdleCpu()\n");
			if (_isReading) {
				if (_therm.isReady()) {
					_lastTemp = _therm.value();
					return false; //no more cpu needed.
				} else {
					return true; //need more cpu time.
				}
			} else {
				struct timespec now = timespecNow();
				if (timespecLt(_nextReadTime, now)) { //time for another read
					_nextReadTime = timespecAdd(now, _interval);
					_therm.startRead();
					return true; //more cpu time needed.
				} else {
					return false;
				}
			}
		}
		float getLastTemp() const {
			return _lastTemp;
		}
};

}
#endif
