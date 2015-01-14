/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 

#ifndef DRIVERS_TEMPCONTROL_H
#define DRIVERS_TEMPCONTROL_H

#include <utility> //for std::move

#include "iodriver.h"
#include "common/filters/nofilter.h"
#include "common/intervaltimer.h"
#include "platforms/auto/chronoclock.h" //for EventClockT
#include "iodrivers/iopin.h"
#include "compileflags.h" //for CelciusType
#include "pid.h" //for default template parameter
#include "common/logging.h"

namespace iodrv {

//enum passed to instructor to define the TempControl instance as either controlling a Hotend or a Heated Bed.
//Functionally, they work the same, but each type responds to different G-codes.
enum TempControlType {
    HotendType,
    HeatedBedType
};

/* 
 * TempControl provides a way to coordinate thermistor readings with the PWM control of a hotend OR heated bed.
 * It used a PID controller to determine the ideal PWM for a given thermistor reading and temperature target.
 * Additionally, it accepts an (optional) filter applied BEFORE the PID controller, which can be used to weed out some variability in readings (use a low-pass filter for this).
 * Currently, it assumes an RC-based thermistor, but in the future it may be expanded to work with any analog IoPin too.
 */
template <typename Thermistor, typename PID=PID, typename Filter=NoFilter> class TempControl : public IODriver {
    static const std::chrono::microseconds _intervalThresh;
    static const std::chrono::microseconds _readInterval;
    static const std::chrono::microseconds _maxRead;
    const TempControlType _hotType;
    IntervalTimer _intervalTimer;
    IoPin _heater;
    Thermistor _therm;
    PID _pid;
    Filter _filter;
    float _pwmPeriod;
    float _destTemp;
    float _lastTemp;
    bool _isReading;
    EventClockT::time_point _nextReadTime;
    public:
        inline TempControl(const TempControlType hotType, IoPin &&heater, Thermistor &&therm, const PID &pid, const Filter &filter, float pwmPeriod=1./25000) 
         : IODriver(), _hotType(hotType), _heater(std::move(heater)), _therm(std::move(therm)), _pid(pid), _filter(filter), _pwmPeriod(pwmPeriod), _destTemp(-300), _lastTemp(-300), _isReading(false),
         _nextReadTime(EventClockT::now()) {
            _heater.setDefaultState(IO_DEFAULT_LOW);
            _heater.makeDigitalOutput(IoLow);
        }
        //register as the correct device type:
        inline bool isHotend() const {
            return _hotType == HotendType;
        }
        inline bool isHeatedBed() const {
            return _hotType == HeatedBedType;
        }
        inline void setTargetTemperature(CelciusType t) {
            _destTemp = t;
        }
        inline CelciusType getMeasuredTemperature() const {
            return _lastTemp;
        }
        inline CelciusType getTargetTemperature() const {
            return _destTemp;
        }
        template <typename CallbackInterface> bool onIdleCpu(CallbackInterface &cbInterface) {
            //LOGV("TempControl::onIdleCpu()\n");
            if (_isReading) {
                if (_therm.isReady()) {
                    _isReading = false;
                    if (_intervalTimer.clockCmp(_intervalThresh) > 0) { //too much latency in reading sample; restart.
                        LOGV("Thermistor sample dropped\n");
                        return true; //restart read.
                    } else {
                        _lastTemp = _therm.value();
                        updatePwm(cbInterface);
                        return false; //no more cpu needed.
                    }
                } else {
                    _intervalTimer.clock();
                    if (_therm.timeSinceStartRead() > _maxRead) {
                        LOG("Thermistor read error\n");
                        _isReading = false;
                        return false;
                    } else {
                        return true; //still waiting for result; need more cpu time.
                    }
                }
            } else {
                auto now = _intervalTimer.clock();
                if (_nextReadTime < now) {
                    _nextReadTime += _readInterval;
                    _therm.startRead();
                    _isReading = true;
                    return true; //just started read; more cpu time needed.
                } else { //wait until it's time for another read.
                    return false;
                }
            }
        }
    private:
        template <typename CallbackInterface> void updatePwm(CallbackInterface &cbInterface) {
	        // Make this actually pass both the setpoint and process value
	        // into the the controller
            float filtered = _filter.feed(_lastTemp);
            float pwm = _pid.feed(_destTemp, filtered);
            LOG("tempcontrol: drive-strength=%f, temp=%f *C\n", pwm, filtered);
            cbInterface.schedPwm(_heater, pwm, _pwmPeriod);
        }
};

#if RUNNING_IN_VM
    template <typename Thermistor, typename PID, typename Filter> const std::chrono::microseconds TempControl<Thermistor, PID, Filter>::_intervalThresh(2000000); //high latency for valgrind
#else
    template <typename Thermistor, typename PID, typename Filter> const std::chrono::microseconds TempControl<Thermistor, PID, Filter>::_intervalThresh(40000);
#endif

template <typename Thermistor, typename PID, typename Filter> const std::chrono::microseconds TempControl<Thermistor, PID, Filter>::_readInterval(3000000);
template <typename Thermistor, typename PID, typename Filter> const std::chrono::microseconds TempControl<Thermistor, PID, Filter>::_maxRead(1000000);

}
#endif
