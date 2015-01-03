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

#ifndef PLATFORMS_GENERIC_HARDWARESCHEDULER_H
#define PLATFORMS_GENERIC_HARDWARESCHEDULER_H

#include <cassert> //for assert

#include "platforms/auto/chronoclock.h" //for EventClockT
#include "platforms/auto/primitiveiopin.h"
#include "schedulerbase.h" //for OnIdleCpuIntervalT (cannot forward-declare an enum)
#include "common/logging.h"

//forward declare for class defined in outputevent.h
class OutputEvent; 


namespace plat {
namespace generic {

struct HardwareScheduler {
    //add this event to the hardware queue, waiting until schedTime(evt.time()) if necessary
    inline void queue(OutputEvent evt) {
        evt.primitiveIoPin().digitalWrite(evt.state());
    }
    //Set the given pin to a pwm duty-cycle of `ratio` using a maximum period of maxPeriod (irrelevant if using PCM algorithm). 
    //E.g. queuePwm(5, 0.4) sets pin #5 to a 40% duty cycle.
    inline void queuePwm(const PrimitiveIoPin &pin, float ratio, float idealPeriod) {
        (void)pin; (void)ratio; (void)idealPeriod; //unused
        LOGW("Warning: default platforms/generic/hardwarescheduler.h cannot queuePwm()\n");
    }
    //If an event needs to occur at evtTime, this function should return the earliest time at which it can be scheduled.
    inline EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
        return evtTime;
    }
    //Can be used to perform routine resource management when there's free cpu.
    //Avoid spending more than a few hundred microseconds in this function, or event scheduling might be impacted
    //@return true if we request more cpu time.
    inline bool onIdleCpu(OnIdleCpuIntervalT interval) {
        (void)interval; //unused
        return false; //no more cpu needed
    }
};
}
}


#endif