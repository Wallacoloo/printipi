#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

/* 
 * Printipi/drivers/tempcontrol.h
 * (c) 2014 Colin Wallace
 * 
 * TempControl provides a way to coordinate thermistor readings with the PWM control of a hotend OR heated bed.
 * It used a PID controller to determine the ideal PWM for a given thermistor reading and temperature target.
 * Additionally, it accepts an (optional) filter applied BEFORE the PID controller, which can be used to weed out some variability in readings (use a low-pass filter for this).
 */

#include "drivers/iodriver.h"
#include "common/timeutil.h"
#include "filters/nofilter.h"
#include "intervaltimer.h"
#include "common/typesettings.h"

namespace drv {

//enum passed as template parameter to define the TempControl instance as either controlling a Hotend or a Heated Bed.
//Functionally, they work the same, but each type responds to different G-codes.
enum TempControlType {
	HotendType,
	HeatedBedType
};

template <TempControlType HotType, AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter=NoFilter> class TempControl : public IODriver {
	static const struct timespec _intervalThresh; //drop thermistor read if the IOs aren't serviced regularly enough.
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
		TempControl() : IODriver(this), _destTemp(-300), _lastTemp(-300), _isReading(false), _nextReadTime(timespecNow()) {
		}
		//register as the correct device type:
		bool isHotend() const {
			return HotType == HotendType;
		}
		bool isHeatedBed() const {
			return HotType == HeatedBedType;
		}
		//route output commands to the heater:
		void stepForward() {
			_heater.stepForward();
		}
		void stepBackward() {
			_heater.stepBackward();
		}
		void setTargetTemperature(CelciusType t) {
			_destTemp = t;
		}
		CelciusType getMeasuredTemperature() const {
			return _lastTemp;
		}
		template <typename Sched> bool onIdleCpu(Sched &sched) {
			//LOGV("TempControl::onIdleCpu()\n");
			if (_isReading) {
				if (_therm.isReady()) {
					_isReading = false;
					if (_intervalTimer.clockCmp(timespecToTimepoint<EventClockT::time_point>(_intervalThresh)) > 0) { //too much latency in reading sample; restart.
						LOGV("Thermistor sample dropped\n");
						return true; //restart read.
					} else {
						_lastTemp = _therm.value();
						updatePwm(sched);
						return false; //no more cpu needed.
					}
				} else {
					_intervalTimer.clock();
					return true; //need more cpu time.
				}
			} else {
				const struct timespec& now = timepointToTimespec(_intervalTimer.clock());
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
	private:
		template <typename Sched> void updatePwm(Sched &sched) {
			float error = _destTemp - _lastTemp;
			error = _filter.feed(error);
			float pwm = _pid.feed(error);
			LOG("tempcontrol: pwm=%f\n", pwm);
			sched.schedPwm(DeviceIdx, PwmInfo(pwm, 0.1));
		}
};

#if RUNNING_IN_VM
	template <TempControlType HotType, AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter> const struct timespec TempControl<HotType, DeviceIdx, Heater, Thermistor, PID, Filter>::_intervalThresh{0, 2000000}; //high latency for valgrind
#else
	template <TempControlType HotType, AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter> const struct timespec TempControl<HotType, DeviceIdx, Heater, Thermistor, PID, Filter>::_intervalThresh{0, 40000}; //use 40000 for debug, 2000000 for valgrind.
#endif

template <TempControlType HotType, AxisIdType DeviceIdx, typename Heater, typename Thermistor, typename PID, typename Filter> const struct timespec TempControl<HotType, DeviceIdx, Heater, Thermistor, PID, Filter>::_readInterval{3, 0};

}
#endif
