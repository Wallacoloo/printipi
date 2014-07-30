#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

#include "drivers/iodriver.h"
#include "timeutil.h"
#include "filters/nofilter.h"
#include "intervaltimer.h"

namespace drv {

template <AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter=NoFilter> class TempControl : public IODriver {
	static const struct timespec _intervalThresh; //maximum error amount in read before temperature is dropped.
	static const struct timespec _readInterval; //how often to read the thermistor
	IntervalTimer _intervalTimer;
	Heater _heater;
	Thermistor _therm;
	PID _pid;
	Filter _filter;
	float _destTemp;
	float _lastTemp;
	bool _isReading;
	struct timespec _nextReadTime;
	public:
		TempControl() : IODriver(this), _destTemp(0), _lastTemp(0), _isReading(false), _nextReadTime(timespecNow()) {
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
		template <typename Sched> bool onIdleCpu(Sched &sched) {
			//LOGV("TempControl::onIdleCpu()\n");
			if (_isReading) {
				if (_therm.isReady()) {
					_isReading = false;
					if (_intervalTimer.clockCmp(_intervalThresh) <= 0) {
						_lastTemp = _therm.value();
						updatePwm(sched);
					} //else: drop sample.
					return false; //no more cpu needed.
				} else {
					_intervalTimer.clock();
					return true; //need more cpu time.
				}
			} else {
				const struct timespec& now = _intervalTimer.clockGet();
				if (timespecLt(_nextReadTime, now)) { //time for another read
					_nextReadTime = timespecAdd(now, _readInterval);
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
		template <typename Sched> void updatePwm(Sched &sched) {
			float error = _destTemp - _lastTemp;
			error = _filter.feed(error);
			float pwm = _pid.feed(error);
			//float P = 0.01*error;
			LOG("tempcontrol: pwm=%f\n", pwm);
			sched.schedPwm(DeviceIdx, PwmInfo(pwm, 0.1));
		}
};

template <AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter> const struct timespec TempControl<DeviceIdx, Heater, Thermistor, PID, Filter>::_intervalThresh{0, 8000};

template <AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter> const struct timespec TempControl<DeviceIdx, Heater, Thermistor, PID, Filter>::_readInterval{1, 0};

}
#endif
