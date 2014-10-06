#ifndef DRIVERS_IODRIVER_H
#define DRIVERS_IODRIVER_H

/* 
 * Printipi/drivers/iodriver.h
 * (c) 2014 Colin Wallace
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

#include <cassert> //for assert
#include "common/typesettings.h"
#include "common/tupleutil.h"
#include "scheduler.h"
#include "event.h"
#include "drivers/iopin.h" //for NoPin

namespace drv {

class IODriver {
    public:
        inline IODriver() {}
        //for a (stepper) motor, advance +/- 1 step:
        inline bool isEventOutputSequenceable(const Event &) { return false; } //OVERRIDE THIS
        std::vector<OutputEvent> getEventOutputSequence(const Event &) { assert(false); } //OVERRIDE THIS if isEventOutputSequenceable returns true.
        inline bool canDoPwm() const { return false; } //OVERRIDE THIS
        NoPin getPwmPin() const { return NoPin(); } //OVERRIDE THIS if canDoPwm returns true.
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
        inline float fanPwmPeriod() const { return 0.2; }
        inline float heaterPwmPeriod() const { return 0.1; }
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
        template <typename TupleT> static bool canDoPwm(TupleT &drivers, AxisIdType axis);
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

//IODriver::canDoPwm helper functions:
struct IODriver__canDoPwm {
    template <typename T> bool operator()(std::size_t index, T &driver, AxisIdType axis) {
        if (index == axis) {
            return driver.canDoPwm();
        } else {
            return false;
        }
    }
};

template <typename TupleT> bool IODriver::canDoPwm(TupleT &drivers, AxisIdType axis) {
    return tupleReduceLogicalOr(drivers, IODriver__canDoPwm(), axis);
}

//IODriver::callIdleCpuHandlers helper functions:

/*template <typename TupleT, std::size_t myIdxPlusOne, typename ...Args> struct IODriver__onIdleCpu {
    bool operator()(TupleT &drivers, Args... args) {
        bool prev = IODriver__onIdleCpu<TupleT, myIdxPlusOne-1, Args...>()(drivers, args...);
        bool cur = std::get<myIdxPlusOne-1>(drivers).onIdleCpu(args...); //EXPLICITLY CALCULATE THIS SEPARATELY TO PREVENT SHORT-CIRCUIT OPERATIONS
        return prev || cur; //return true if ANY objects need future servicing.
    }
};

template <typename TupleT, typename ...Args> struct IODriver__onIdleCpu<TupleT, 0, Args...> {
    bool operator()(TupleT &, Args...) {
        //return std::get<0>(drivers).onIdleCpu(args...);
        return false; //no more objects need cpu time.
    }
};

template <typename TupleT, typename ...Args> bool IODriver::callIdleCpuHandlers(TupleT &drivers, Args... args) {
    return IODriver__onIdleCpu<TupleT, std::tuple_size<TupleT>::value, Args...>()(drivers, args...);
}*/
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
