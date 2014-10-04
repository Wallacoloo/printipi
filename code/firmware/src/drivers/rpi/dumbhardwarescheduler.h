#ifndef DRIVERS_RPI_DUMBHARDWARESCHEDULER_H
#define DRIVERS_RPI_DUMBHARDWARESCHEDULER_H

#include "outputevent.h"
#include "mitpi.h"

namespace rpi {

class DumbHardwareScheduler {
    public:
        DumbHardwareScheduler() {
            mitpi::init();
        }
        inline void queue(const OutputEvent &e) {
            //add this event to the hardware queue, waiting until schedTime(evt.time()) if necessary
            //assert(false);
            mitpi::setPinState(e.pinId(), e.state());
        }
        inline void queuePwm(int /*pin*/, float /*ratio*/, float /*maxPeriod*/) {
            //Set the given pin to a pwm duty-cycle of `ratio` using a maximum period of maxPeriod (irrelevant if using PCM algorithm). Eg queuePwm(5, 0.4) sets pin #5 to a 40% duty cycle.
            //assert(false); //DefaultSchedulerInterface::HardwareScheduler cannot queuePwm!
            //TODO: implement
        }
        template <typename EventClockT_time_point> EventClockT_time_point schedTime(EventClockT_time_point evtTime) const {
            //If an event needs to occur at evtTime, this function should return the earliest time at which it can be scheduled.
            //This function is only templated to prevent importing typesettings.h (circular import), required for the real EventClockT. An implementation only needs to support the EventClockT::time_point defined in common/typesettings.h
            return evtTime;
        }
};

}

#endif
