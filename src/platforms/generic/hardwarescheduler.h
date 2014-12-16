#ifndef PLATFORMS_GENERIC_HARDWARESCHEDULER_H
#define PLATFORMS_GENERIC_HARDWARESCHEDULER_H

#include <cassert> //for assert

#include "platforms/auto/chronoclock.h" //for EventClockT
#include "schedulerbase.h" //for OnIdleCpuIntervalT (cannot forward-declare an enum)

class OutputEvent; //forward declare for class defined in outputevent.h


namespace plat {
namespace generic {

struct HardwareScheduler {
    inline void queue(const OutputEvent &) {
        //add this event to the hardware queue, waiting until schedTime(evt.time()) if necessary
        //assert(false); //DefaultSchedulerInterface::HardwareScheduler cannot queue!
        LOGW("Warning: default platforms/generic/hardwarescheduler.h cannot queue()\n");
    }
    inline void queuePwm(int pin, float ratio, float idealPeriod) {
        //Set the given pin to a pwm duty-cycle of `ratio` using a maximum period of maxPeriod (irrelevant if using PCM algorithm). Eg queuePwm(5, 0.4) sets pin #5 to a 40% duty cycle.
        (void)pin; (void)ratio; (void)idealPeriod; //unused
        //assert(false); //DefaultSchedulerInterface::HardwareScheduler cannot queuePwm!
        LOGW("Warning: default platforms/generic/hardwarescheduler.h cannot queuePwm()\n");
    }
    inline EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
        //If an event needs to occur at evtTime, this function should return the earliest time at which it can be scheduled.
        //This function is only templated to prevent importing typesettings.h (circular import), required for the real EventClockT. An implementation only needs to support EventClockT::time_point
        return evtTime;
    }
    inline bool onIdleCpu(OnIdleCpuIntervalT interval) {
        (void)interval; //unused
        return false; //no more cpu needed
    }
};
}
}


#endif