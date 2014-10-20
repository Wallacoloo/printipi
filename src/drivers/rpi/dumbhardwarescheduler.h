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
 * Printipi/drivers/rpi/dumbhardwarescheduler.h
 *
 * DumbHardwareScheduler implements the HardwareScheduler interface declared in schedulerbase.h as simplistically as is possible.
 *
 * It just toggles whichever pin immediately when requested (no buffering). Thus it is very susceptible to OS task-switching, but can also serve as a reference when doing a speedy port of Printipi to another platform.
 *
 * For a much better implementation of HardwareScheduler, see DmaScheduler (which is the default HardwareScheduler used for Raspberry Pi builds)
 */

#ifndef DRIVERS_RPI_DUMBHARDWARESCHEDULER_H
#define DRIVERS_RPI_DUMBHARDWARESCHEDULER_H

#include "outputevent.h"
#include "mitpi.h"
#include "common/typsettings/clocks.h"

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
        inline EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
            //If an event needs to occur at evtTime, this function should return the earliest time at which it can be scheduled.
            return evtTime;
        }
};

}

#endif
