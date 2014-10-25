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
 * Printipi/schedulerbase.h
 *
 * SchedulerBase allows any file to insert exit handlers, without adding the entire Scheduler as a dependency.
 * It also defines the interfaces used within the Scheduler (SchedulerInterface and HardwareScheduler) which can be inherited from.
 */

#ifndef SCHEDULERBASE_H
#define SCHEDULERBASE_H

#include <algorithm> //for std::min
#include <array>
#include <vector>
#include <cassert> //for assert

#include "outputevent.h"

#include "drivers/auto/chronoclock.h" //for EventClockT

#ifndef SCHED_PRIORITY
    #define SCHED_PRIORITY 30
#endif
#ifndef SCHED_NUM_EXIT_HANDLER_LEVELS
    #define SCHED_NUM_EXIT_HANDLER_LEVELS 2
#endif
#define SCHED_IO_EXIT_LEVEL 0
#define SCHED_MEM_EXIT_LEVEL 1

//Scheduler::Interface::onIdleCpu can be called with a flag indicating (roughly) how long it's been since it was last called.
enum OnIdleCpuIntervalT {
    OnIdleCpuIntervalShort,
    OnIdleCpuIntervalWide
};

class Event; //forward declaration to avoid inclusion of event.h.

/* Base class from which all templated schedulers derive.
Defines things such as exit handlers */
class SchedulerBase {
    static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
    //static std::atomic<bool> isExiting; //typical implementations of exit() call the exit handlers from within the thread that called exit. Therefore, if the exiting thread causes another thread to call exit(), this value must be atomic. But we only have one thread.
    static bool isExiting;
    private:
        static void callExitHandlers();
    public:
        static void configureExitHandlers();
        static void registerExitHandler(void (*handler)(), unsigned level);
};

/* A Scheduler is useless without an interface. In order to use a scheduler, implement a class with the following methods
and pass it as a template argument to the Scheduler<Interface> type. */
struct NullSchedulerInterface {
    public:
        struct HardwareScheduler {
            inline void queue(const OutputEvent &) {
                //add this event to the hardware queue, waiting until schedTime(evt.time()) if necessary
                assert(false); //DefaultSchedulerInterface::HardwareScheduler cannot queue!
            }
            inline void queuePwm(int /*pin*/, float /*ratio*/, float /*maxPeriod*/) {
                //Set the given pin to a pwm duty-cycle of `ratio` using a maximum period of maxPeriod (irrelevant if using PCM algorithm). Eg queuePwm(5, 0.4) sets pin #5 to a 40% duty cycle.
                assert(false); //DefaultSchedulerInterface::HardwareScheduler cannot queuePwm!
            }
            EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
                //If an event needs to occur at evtTime, this function should return the earliest time at which it can be scheduled.
                //This function is only templated to prevent importing typesettings.h (circular import), required for the real EventClockT. An implementation only needs to support EventClockT::time_point
                return evtTime;
            }
            bool onIdleCpu(OnIdleCpuIntervalT interval) {
                (void)interval; //unused
                return false; //no more cpu needed
            }
        };
    private:
        HardwareScheduler _hardwareScheduler;
    public:
        inline bool onIdleCpu(OnIdleCpuIntervalT interval) {
            (void)interval; //unused
            return false; //no more cpu needed
        }
        inline static constexpr std::size_t numIoDrivers() {
            return 0; //no IoDrivers;
        }
        template <typename Func> void iterEventOutputSequence(const Event &evt, Func f) {
            assert(false); //DefaultSchedulerInterface cannot iterEventOutputSequence
        }
        inline void queue(const OutputEvent &evt) {
            _hardwareScheduler.queue(evt);
        }
        inline void queuePwm(int pin, float duty, float maxPeriod) {
            _hardwareScheduler.queuePwm(pin, duty, maxPeriod);
        }
        template <typename EventClockT_time_point> EventClockT_time_point schedTime(EventClockT_time_point evtTime) const {
            return _hardwareScheduler.schedTime(evtTime);
        }
};

#endif
