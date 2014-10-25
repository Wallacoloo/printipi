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

/* 
 * Printipi/drivers/iodriver.h
 *
 * IODrivers control the electrical interface to each component.
 * One IODriver is needed for each stepper motor, fan, hotend, etc.
 * Note that the stepForward and stepBackward methods may have different meanings for non-stepper motors.
 *   for fans or hotends, this would mean turn on, or turn off
 *
 * Note: IODriver is an interface, and not an implementation.
 * An implementation is needed for each electrical component - the fan, hotend, and 1 for each stepper motor, etc.
 * These implementations must provide the functions outlined further down in the header.
 */
 

#ifndef DRIVERS_IODRIVER_H
#define DRIVERS_IODRIVER_H

#include <cassert> //for assert
#include "schedulerbase.h" //for OnIdleCpuIntervalT
#include "typesettings/primitives.h" //for CelciusType
#include "common/tupleutil.h"
#include "event.h"
#include "drivers/iopin.h" //for NoPin

namespace drv {

class IODriver {
    public:
        inline IODriver() {}
        //for a (stepper) motor, advance +/- 1 step:
        inline bool isEventOutputSequenceable(const Event &) { return false; } //OVERRIDE THIS
        std::vector<OutputEvent> getEventOutputSequence(const Event &) { assert(false); } //OVERRIDE THIS if isEventOutputSequenceable returns true.
        NoPin getPwmPin() const { return NoPin(); } //OVERRIDE THIS if device is pwm-able.
        inline void stepForward() {} //OVERRIDE THIS
        inline void stepBackward() {} //OVERRIDE THIS
        /*deactivate: called at program exit.
        safely deactivate any IOs, including motors, heaters, etc.*/
        //inline static void deactivate() {} //OVERRIDE THIS
        /* called by M17; Enable/power all stepper motors */
        inline void lockAxis() {} //OVERRIDE THIS (stepper motor drivers only)
        /* called by M18; Disable all stepper motors. Intention is to let them move 'freely', eg, for manual adjustment or to disable idle noise. */
        inline void unlockAxis() {} //OVERRIDE THIS (stepper motor drivers only)
        inline bool isFan() const { return false; } //OVERRIDE THIS (fans only: return true)
        //inline float fanPwmPeriod() const { return 0.2; }
        //inline float heaterPwmPeriod() const { return 0.1; }
        inline float fanPwmPeriod() const { return 0; }
        inline float heaterPwmPeriod() const { return 1.0/25000; }
        inline bool isHotend() const { return false; } //OVERRIDE THIS (hotends only: return true)
        inline bool isHeatedBed() const { return false; } //OVERRIDE THIS (beds only: return true. No need to define a bed if it isn't heated).
        inline void setTargetTemperature(CelciusType) { assert(false && "IoDriver::setTargetTemperature() must be overriden by subclass."); }
        inline CelciusType getMeasuredTemperature() const { return -300; } //OVERRIDE THIS (hotends / beds only)
        /* called when the scheduler has extra time,
        Can be used to check the status of inputs, etc.
        Return true if object needs to continue to be serviced, false otherwise. */
        template <typename Sched> inline bool onIdleCpu(Sched & /*sched*/) { return false; } //OVERRIDE THIS
        //selectAndStep...: used internally
        template <typename TupleT> static void selectAndStepForward(TupleT &drivers, AxisIdType axis);
        template <typename TupleT> static void selectAndStepBackward(TupleT &drivers, AxisIdType axis);
        template <typename TupleT> static bool isEventOutputSequenceable(TupleT &drivers, const Event &evt);
        template <typename TupleT, typename ...Args > static bool callIdleCpuHandlers(TupleT &drivers, Args... args);
        template <typename TupleT> static void lockAllAxis(TupleT &drivers);
        template <typename TupleT> static void unlockAllAxis(TupleT &drivers);
        template <typename TupleT> static void setHotendTemp(TupleT &drivers, CelciusType temp);
        template <typename TupleT> static void setBedTemp(TupleT &drivers, CelciusType temp);
        template <typename TupleT> static CelciusType getHotendTemp(TupleT &drivers);
        template <typename TupleT> static CelciusType getBedTemp(TupleT &drivers);
};

//IODriver::selectAndStepForward helper functions:
struct IODriver__stepForward {
    template <typename T> void operator()(std::size_t index, T &driver, AxisIdType desiredIndex) {
        if (index == desiredIndex) {
            driver.stepForward();
        }
    }
};
template <typename TupleT> void IODriver::selectAndStepForward(TupleT &drivers, AxisIdType axis) {
    callOnAll(drivers, IODriver__stepForward(), axis);
}


//IODriver::selectAndStepBackward helper functions:
struct IODriver__stepBackward {
    template <typename T> void operator()(std::size_t index, T &driver, AxisIdType desiredIndex) {
        if (index == desiredIndex) {
            driver.stepBackward();
        }
    }
};
template <typename TupleT> void IODriver::selectAndStepBackward(TupleT &drivers, AxisIdType axis) {
    callOnAll(drivers, IODriver__stepBackward(), axis);
}

//IODriver::isEventOutputSequenceable helper functions:
struct IODriver__isEventOutputSequenceable {
    template <typename T> bool operator()(std::size_t index, T &driver, const Event &evt) {
        if (index == evt.stepperId()) {
            return driver.isEventOutputSequenceable(evt);
        } else {
            return false;
        }
    }
};

template <typename TupleT> bool IODriver::isEventOutputSequenceable(TupleT &drivers, const Event &evt) {
    return tupleReduceLogicalOr(drivers, IODriver__isEventOutputSequenceable(), evt);
}

//IODriver::callIdleCpuHandlers helper functions:

struct IODriver__onIdleCpu {
    template <typename T, typename ...Args> bool operator()(std::size_t /*index*/, T &driver, Args... args) {
        return driver.onIdleCpu(args...);
    }
};
template <typename TupleT, typename ...Args> bool IODriver::callIdleCpuHandlers(TupleT &drivers, Args... args) {
    return tupleReduceLogicalOr(drivers, IODriver__onIdleCpu(), args...);
}

//IODriver::lockAllAxis helper functions:
struct IODriver__lockAllAxis {
    template <typename T> void operator()(std::size_t /*index*/, T &driver) {
        driver.lockAxis();
    }
};
template <typename TupleT> void IODriver::lockAllAxis(TupleT &drivers) {
    callOnAll(drivers, IODriver__lockAllAxis());
}

//IODriver::unlockAllAxis helper functions:
struct IODriver__unlockAllAxis {
    template <typename T> void operator()(std::size_t /*index*/, T &driver) {
        driver.unlockAxis();
    }
};
template <typename TupleT> void IODriver::unlockAllAxis(TupleT &drivers) {
    callOnAll(drivers, IODriver__unlockAllAxis());
}

//IODriver::setHotendTemp helper functions:
struct IODriver__setHotendTemp {
    template <typename T> void operator()(std::size_t /*index*/, T &driver, CelciusType temp) {
        if (driver.isHotend()) {
            driver.setTargetTemperature(temp);
        }
    }
};
template <typename TupleT> void IODriver::setHotendTemp(TupleT &drivers, CelciusType temp) {
    callOnAll(drivers, IODriver__setHotendTemp(), temp);
}
//IODriver::setBedTemp helper functions:
struct IODriver__setBedTemp {
    template <typename T> void operator()(std::size_t /*index*/, T &driver, CelciusType temp) {
        if (driver.isHeatedBed()) {
            driver.setTargetTemperature(temp);
        }
    }
};
template <typename TupleT> void IODriver::setBedTemp(TupleT &drivers, CelciusType temp) {
    callOnAll(drivers, IODriver__setBedTemp(), temp);
}

//IODriver::getHotendTemp helper functions:
struct IODriver__getHotendTemp {
    CelciusType value;
    IODriver__getHotendTemp() : value(-300) {}
    template <typename T> void operator()(std::size_t /*index*/, T &driver) {
        if (driver.isHotend()) {
            value = driver.getMeasuredTemperature();
        }
    }
};
template <typename TupleT> CelciusType IODriver::getHotendTemp(TupleT &drivers) {
    IODriver__getHotendTemp t;
    callOnAll(drivers, &t);
    return t.value;
}
//IODriver::getBedTemp helper functions:
struct IODriver__getBedTemp {
    CelciusType value;
    IODriver__getBedTemp() : value(-300) {}
    template <typename T> void operator()(std::size_t /*index*/, T &driver) {
        if (driver.isHeatedBed()) {
            value = driver.getMeasuredTemperature();
        }
    }
};
template <typename TupleT> CelciusType IODriver::getBedTemp(TupleT &drivers) {
    IODriver__getBedTemp t;
    callOnAll(drivers, &t);
    return t.value;
}



}
#endif
