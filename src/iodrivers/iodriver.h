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
 * Printipi/iodrivers/iodriver.h
 *
 * IODrivers control the electrical interface to each component.
 * One IODriver is needed for each stepper motor, fan, hotend, etc.
 *
 * Note: IODriver is an interface, and not an implementation.
 * An implementation is needed for each electrical component - the fan, hotend, and 1 for each stepper motor, etc.
 * These implementations must provide the functions outlined further down in the header.
 */
 

#ifndef DRIVERS_IODRIVER_H
#define DRIVERS_IODRIVER_H

#include <cassert> //for assert
#include <tuple> 
#include "schedulerbase.h" //for OnIdleCpuIntervalT
#include "compileflags.h" //for CelciusType
#include "common/tupleutil.h"
#include "common/mathutil.h" //for mathutil::ABSOLUTE_ZERO_CELCIUS
#include "iopin.h"

namespace iodrv {

class IODriver {
    public:
        inline IODriver() {}
        //called by M17; Enable/power all stepper motors
        //OVERRIDE THIS (stepper motor drivers only)
        inline void lockAxis() {}
        // called by M18; Disable all stepper motors. Intention is to let them move 'freely', eg, for manual adjustment or to disable idle noise.
        //OVERRIDE THIS (stepper motor drivers only)
        inline void unlockAxis() {} 
        //OVERRIDE THIS (fans only: return true)
        inline bool isFan() const { return false; } 
        //OVERRIDE THIS (hotends only: return true)
        inline bool isHotend() const { return false; } 
        //OVERRIDE THIS (beds only: return true. No need to define a bed if it isn't heated).
        inline bool isHeatedBed() const { return false; } 
        //OVERRIDE THIS (hotends / beds only)
        inline void setTargetTemperature(CelciusType) { assert(false && "IoDriver::setTargetTemperature() must be overriden by subclass."); }
        //OVERRIDE THIS (hotends / beds only)
        inline CelciusType getTargetTemperature() const { 
            assert(false && "IoDriver::getTargetTemperature() must be overriden by subclass."); 
            return mathutil::ABSOLUTE_ZERO_CELCIUS; //for when assertions are disabled.
        }
        //OVERRIDE THIS (hotends / beds only)
        inline CelciusType getMeasuredTemperature() const { 
            assert(false && "IoDriver::getMeasuredTemperature() must be overriden by subclass."); 
            return mathutil::ABSOLUTE_ZERO_CELCIUS; //for when assertions are disabled.
        } 
        //called when the scheduler has extra time,
        //Can be used to check the status of inputs, etc.
        //Return true if object needs to continue to be serviced, false otherwise. 
        template <typename CallbackInterface> inline bool onIdleCpu(const CallbackInterface &) { return false; }
        template <typename CallbackInterface> inline void setFanDutyCycle(const CallbackInterface &interface, float dutyCycle) {
            (void)interface;
            (void)dutyCycle;
            assert(false && "IoDriver::setFanDutyCycle() must be overriden by subclass");
        }
        template <typename TupleT> static void lockAllAxis(TupleT &drivers);
        template <typename TupleT> static void unlockAllAxis(TupleT &drivers);
        template <typename TupleT> static void setHotendTemp(TupleT &drivers, CelciusType temp);
        template <typename TupleT> static void setBedTemp(TupleT &drivers, CelciusType temp);
        template <typename TupleT> static CelciusType getHotendTemp(TupleT &drivers);
        template <typename TupleT> static CelciusType getHotendTargetTemp(TupleT &drivers);
        template <typename TupleT> static CelciusType getBedTemp(TupleT &drivers);
};

namespace {
    //place helper functions in an unnamed namespace to limit visibility and hint the documentation generator

    //IODriver::lockAllAxis helper functions:
    struct IODriver__lockAllAxis {
        template <typename T> void operator()(std::size_t index, T &driver) {
            (void)index; //unused;
            driver.lockAxis();
        }
    };

    //IODriver::unlockAllAxis helper functions:
    struct IODriver__unlockAllAxis {
        template <typename T> void operator()(std::size_t index, T &driver) {
            (void)index; //unused;
            driver.unlockAxis();
        }
    };

    //IODriver::setHotendTemp helper functions:
    struct IODriver__setHotendTemp {
        template <typename T> void operator()(std::size_t index, T &driver, CelciusType temp) {
            (void)index; //unused;
            if (driver.isHotend()) {
                driver.setTargetTemperature(temp);
            }
        }
    };

    //IODriver::setBedTemp helper functions:
    struct IODriver__setBedTemp {
        template <typename T> void operator()(std::size_t index, T &driver, CelciusType temp) {
            (void)index; //unused;
            if (driver.isHeatedBed()) {
                driver.setTargetTemperature(temp);
            }
        }
    };

    //IODriver::getHotendTemp helper functions:
    struct IODriver__getHotendTemp {
        CelciusType value;
        IODriver__getHotendTemp() : value(mathutil::ABSOLUTE_ZERO_CELCIUS) {}
        template <typename T> void operator()(std::size_t index, T &driver) {
            (void)index; //unused;
            if (driver.isHotend()) {
                value = driver.getMeasuredTemperature();
            }
        }
    };

    //IODriver::getHotendTargetTemp helper functions:
    struct IODriver__getHotendTargetTemp {
        CelciusType value;
        IODriver__getHotendTargetTemp() : value(mathutil::ABSOLUTE_ZERO_CELCIUS) {}
        template <typename T> void operator()(std::size_t index, T &driver) {
            (void)index; //unused;
            if (driver.isHotend()) {
                value = driver.getTargetTemperature();
            }
        }
    };

    //IODriver::getBedTemp helper functions:
    struct IODriver__getBedTemp {
        CelciusType value;
        IODriver__getBedTemp() : value(mathutil::ABSOLUTE_ZERO_CELCIUS) {}
        template <typename T> void operator()(std::size_t /*index*/, T &driver) {
            if (driver.isHeatedBed()) {
                value = driver.getMeasuredTemperature();
            }
        }
    };
}



template <typename TupleT> void IODriver::lockAllAxis(TupleT &drivers) {
    callOnAll(drivers, IODriver__lockAllAxis());
}
template <typename TupleT> void IODriver::unlockAllAxis(TupleT &drivers) {
    callOnAll(drivers, IODriver__unlockAllAxis());
}
template <typename TupleT> void IODriver::setHotendTemp(TupleT &drivers, CelciusType temp) {
    callOnAll(drivers, IODriver__setHotendTemp(), temp);
}
template <typename TupleT> void IODriver::setBedTemp(TupleT &drivers, CelciusType temp) {
    callOnAll(drivers, IODriver__setBedTemp(), temp);
}
template <typename TupleT> CelciusType IODriver::getHotendTemp(TupleT &drivers) {
    IODriver__getHotendTemp t;
    callOnAll(drivers, &t);
    return t.value;
}
template <typename TupleT> CelciusType IODriver::getHotendTargetTemp(TupleT &drivers) {
    IODriver__getHotendTargetTemp t;
    callOnAll(drivers, &t);
    return t.value;
}
template <typename TupleT> CelciusType IODriver::getBedTemp(TupleT &drivers) {
    IODriver__getBedTemp t;
    callOnAll(drivers, &t);
    return t.value;
}



}
#endif
