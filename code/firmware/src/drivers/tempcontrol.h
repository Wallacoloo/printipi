#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

#include "drivers/iodriver.h"
#include "timeutil.h"
#include "filters/nofilter.h"

namespace drv {

template <AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter=NoFilter> class TempControl : public IODriver {
	Heater _heater;
	Thermistor _therm;
	PID _pid;
	Filter _filter;
	float _destTemp;
	float _lastTemp;
	bool _isReading;
	struct timespec _nextReadTime;
	struct timespec _interval;
	public:
		TempControl() : IODriver(this), _destTemp(0), _lastTemp(0), _isReading(false), _nextReadTime(timespecNow()), _interval{1, 0} {
		}
		//route output commands to the heater:
		void stepForward() {
			_heater.stepForward();
		}
		void stepBackward() {
			_heater.stepBackward();
		}
		void setTemp(CelciusType t) {
			_destTemp = t;
		}
		bool onIdleCpu(Scheduler &sched) {
			//LOGV("TempControl::onIdleCpu()\n");
			if (_isReading) {
				if (_therm.isReady()) {
					_lastTemp = _therm.value();
					_isReading = false;
					updatePwm(sched);
					return false; //no more cpu needed.
				} else {
					return true; //need more cpu time.
				}
			} else {
				struct timespec now = timespecNow();
				if (timespecLt(_nextReadTime, now)) { //time for another read
					_nextReadTime = timespecAdd(now, _interval);
					_therm.startRead();
					_isReading = true;
					return true; //more cpu time needed.
				} else { //wait until it's time for another read.
					return false;
				}
			}
		}
		float getLastTemp() const {
			return _lastTemp;
		}
	private:
		void updatePwm(Scheduler &sched) {
			float error = _destTemp - _lastTemp;
			error = _filter.feed(error);
			float pwm = _pid.feed(error);
			//float P = 0.01*error;
			LOG("tempcontrol: pwm=%f\n", pwm);
			sched.schedPwm(DeviceIdx, PwmInfo(pwm, 0.1));
		}
};

}
#endif
