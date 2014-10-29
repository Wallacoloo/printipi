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
#include "compileflags.h" //for CelciusType
#include "common/tupleutil.h"
#include "event.h"
#include "drivers/iopin.h" //for NoPin

namespace drv {

class IODriver {
    public:
        inline IODriver() {}
        //for a (stepper) motor, advance +/- 1 step:
        std::vector<OutputEvent> getEventOutputSequence(const Event &) { assert(false); } //OVERRIDE THIS (stepper motor drivers only)
        NoPin getPwmPin() const { return NoPin(); } //OVERRIDE THIS if device is pwm-able.
        /* called by M17; Enable/power all stepper motors */
        inline void lockAxis() {} //OVERRIDE THIS (stepper motor drivers only)
        /* called by M18; Disable all stepper motors. Intention is to let them move 'freely', eg, for manual adjustment or to disable idle noise. */
        inline void unlockAxis() {} //OVERRIDE THIS (stepper motor drivers only)
        inline bool isFan() const { return false; } //OVERRIDE THIS (fans only: return true)
        inline float fanPwmPeriod() const { return 0; }
        inline float heaterPwmPeriod() const { return 1.0/25000; }
        inline bool isHotend() const { return false; } //OVERRIDE THIS (hotends only: return true)
        inline bool isHeatedBed() const { return false; } //OVERRIDE THIS (beds only: return true. No need to define a bed if it isn't heated).
        inline void setTargetTemperature(CelciusType) { assert(false && "IoDriver::setTargetTemperature() must be overriden by subclass."); }
        inline CelciusType getTargetTemperature() const { assert(false && "IoDriver::getTargetTemperature() must be overriden by subclass."); }
        inline CelciusType getMeasuredTemperature() const { return -300; } //OVERRIDE THIS (hotends / beds only)
        /* called when the scheduler has extra time,
        Can be used to check the status of inputs, etc.
        Return true if object needs to continue to be serviced, false otherwise. */
        template <typename Sched> inline bool onIdleCpu(Sched & /*sched*/) { return false; } //OVERRIDE THIS
        template <typename TupleT, typename ...Args > static bool callIdleCpuHandlers(TupleT &drivers, Args... args);
        template <typename TupleT> static void lockAllAxis(TupleT &drivers);
        template <typename TupleT> static void unlockAllAxis(TupleT &drivers);
        template <typename TupleT> static void setHotendTemp(TupleT &drivers, CelciusType temp);
        template <typename TupleT> static void setBedTemp(TupleT &drivers, CelciusType temp);
        template <typename TupleT> static CelciusType getHotendTemp(TupleT &drivers);
        template <typename TupleT> static CelciusType getHotendTargetTemp(TupleT &drivers);
        template <typename TupleT> static CelciusType getBedTemp(TupleT &drivers);
};

//IODriver::callIdleCpuHandlers helper functions:

struct IODriver__onIdleCpu {
    template <typename T, typename ...Args> bool operator()(std::size_t index, T &driver, Args... args) {
        (void)index; //unused;
        return driver.onIdleCpu(args...);
    }
};
template <typename TupleT, typename ...Args> bool IODriver::callIdleCpuHandlers(TupleT &drivers, Args... args) {
    return tupleReduceLogicalOr(drivers, IODriver__onIdleCpu(), args...);
}

//IODriver::lockAllAxis helper functions:
struct IODriver__lockAllAxis {
    template <typename T> void operator()(std::size_t index, T &driver) {
        (void)index; //unused;
        driver.lockAxis();
    }
};
template <typename TupleT> void IODriver::lockAllAxis(TupleT &drivers) {
    callOnAll(drivers, IODriver__lockAllAxis());
}

//IODriver::unlockAllAxis helper functions:
struct IODriver__unlockAllAxis {
    template <typename T> void operator()(std::size_t index, T &driver) {
        (void)index; //unused;
        driver.unlockAxis();
    }
};
template <typename TupleT> void IODriver::unlockAllAxis(TupleT &drivers) {
    callOnAll(drivers, IODriver__unlockAllAxis());
}

//IODriver::setHotendTemp helper functions:
struct IODriver__setHotendTemp {
    template <typename T> void operator()(std::size_t index, T &driver, CelciusType temp) {
        (void)index; //unused;
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
    template <typename T> void operator()(std::size_t index, T &driver, CelciusType temp) {
        (void)index; //unused;
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
    template <typename T> void operator()(std::size_t index, T &driver) {
        (void)index; //unused;
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
//IODriver::getHotendTargetTemp helper functions:
struct IODriver__getHotendTargetTemp {
    CelciusType value;
    IODriver__getHotendTargetTemp() : value(-300) {}
    template <typename T> void operator()(std::size_t index, T &driver) {
        (void)index; //unused;
        if (driver.isHotend()) {
            value = driver.getTargetTemperature();
        }
    }
};
template <typename TupleT> CelciusType IODriver::getHotendTargetTemp(TupleT &drivers) {
    IODriver__getHotendTargetTemp t;
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
