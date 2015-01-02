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

#ifndef SCHEDULER_H
#define SCHEDULER_H


#include <cassert> //for assert
#include <array>
#include "outputevent.h"
#include "common/logging.h"
#include "common/intervaltimer.h"
#include "compileflags.h"
#include "platforms/auto/thisthreadsleep.h" //for SleepT
#include "platforms/auto/primitiveiopin.h"
#include "iodrivers/iopin.h"

#if USE_PTHREAD
    #include <pthread.h> //for pthread_setschedparam
#endif
#include "schedulerbase.h"


/* 
 * The Scheduler controls program flow between tending communications and executing events at precise times.
 * It is designed to run in a single-threaded environment so it can have maximum control.
 * Scheduler.eventLoop should be called after any program setup is completed.
 * The eventLoop function will frequently yield control *briefly* to Interface.onIdleCpu.
 * This gives the onIdleCpu function the possibility to schedule events using Scheduler.queue.
 */
template <typename Interface> class Scheduler : public SchedulerBase {
    EventClockT::duration MAX_SLEEP; //need to call onIdleCpu handlers every so often, even if no events are ready.
    Interface interface;
    bool hasActiveEvent;
    bool _doExit;
    public:
        void queue(const OutputEvent &evt);
        void schedPwm(const iodrv::IoPin &idx, float duty, float maxPeriod);
        template <typename T> void setMaxSleep(T duration) {
            MAX_SLEEP = std::chrono::duration_cast<EventClockT::duration>(duration);
        }
        inline void setDefaultMaxSleep() {
            setMaxSleep(std::chrono::milliseconds(40));
        }
        Scheduler(Interface interface);
        void initSchedThread() const; //call this from whatever threads call nextEvent to optimize that thread's priority.
        bool isRoomInBuffer() const;
        void eventLoop();
        void yield(const OutputEvent *evt);
        void exitEventLoop();
    private:
        void sleepUntilEvent(const OutputEvent *evt) const;
        bool isEventNear(const OutputEvent &evt) const;
        bool isEventTime(const OutputEvent &evt) const;
};

template <typename Interface> Scheduler<Interface>::Scheduler(Interface interface) 
    : interface(interface)
    ,hasActiveEvent(false)
    ,_doExit(false)
    {
    setDefaultMaxSleep();
}


template <typename Interface> void Scheduler<Interface>::queue(const OutputEvent &evt) {
    hasActiveEvent = true;
    this->yield(&evt);
    hasActiveEvent = false;
}

template <typename Interface> void Scheduler<Interface>::schedPwm(const iodrv::IoPin &pin, float duty, float maxPeriod) {
    duty = std::min(1.f, std::max(0.f, duty)); //clamp pwm between [0, 1]
    this->interface.queuePwm(pin.primitiveIoPin(), pin.translateDutyCycleToPrimitive(duty), maxPeriod);
}

template <typename Interface> void Scheduler<Interface>::initSchedThread() const {
    #if USE_PTHREAD
        struct sched_param sp; 
        sp.sched_priority=SCHED_PRIORITY; 
        if (int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
            LOGW("Warning: pthread_setschedparam (increase thread priority) at scheduler.cpp returned non-zero: %i\n", ret);
        } else {
            LOG("Set pthread sched_priority\n");
        }
    #endif
}

template <typename Interface> bool Scheduler<Interface>::isRoomInBuffer() const {
    return !hasActiveEvent;
}


template <typename Interface> void Scheduler<Interface>::eventLoop() {
    OnIdleCpuIntervalT intervalT = OnIdleCpuIntervalWide;
    int numShortIntervals = 0; //need to track the number of short cpu intervals, because if we just execute short intervals constantly for, say, 1 second, then certain services that only run at long intervals won't occur. So make every, say, 10000th short interval transform into a wide interval.
    while (!_doExit) {
        if (interface.onIdleCpu(intervalT)) {
            intervalT = (++numShortIntervals % 2048) ? OnIdleCpuIntervalShort : OnIdleCpuIntervalWide;
        } else if (!_doExit) { //must recheck _doExit flag, since it could have been modified in the call to onIdleCpu
            intervalT = OnIdleCpuIntervalWide; //no cpu is needed; wide delay
            sleepUntilEvent(nullptr);
        }
    }
    _doExit = false;
}

template <typename Interface> void Scheduler<Interface>::yield(const OutputEvent *evt) {
    OnIdleCpuIntervalT intervalT = OnIdleCpuIntervalWide;
    //need to track the number of short cpu intervals, because if we just execute short intervals constantly for, say, 1 second, 
    //  then certain services that only run at long intervals won't occur. 
    //  So make every, say, 10000th short interval transform into a wide interval.
    int numShortIntervals = 0; 
    while (!isEventTime(*evt)) {
        if (!interface.onIdleCpu(intervalT)) {
            //if we don't need any onIdleCpu, then sleep until the event.
            //sleepUntilEvent won't always do the full sleep; it has a time limit.
            this->sleepUntilEvent(evt);
            //We just slept for a while, which translates to a wide interval. Note that it may not actually be the event time yet.
            intervalT = OnIdleCpuIntervalWide;
            //numShortIntervals = 0;
        } else {
            //after 2048 (just a nice binary number) short intervals, insert a wide interval instead:
            intervalT = (++numShortIntervals % 2048) ? OnIdleCpuIntervalShort : OnIdleCpuIntervalWide;
        }
    }
    interface.queue(*evt);
}

template <typename Interface> void Scheduler<Interface>::exitEventLoop() {
    _doExit = true;
}

template <typename Interface> void Scheduler<Interface>::sleepUntilEvent(const OutputEvent *evt) const {
    //need to call onIdleCpu handlers occasionally - avoid sleeping for long periods of time.
    auto sleepUntil = EventClockT::now() + MAX_SLEEP;
    if (evt) { //allow calling with NULL to sleep for a configured period of time (MAX_SLEEP)
        auto evtTime = interface.schedTime(evt->time());
        if (evtTime < sleepUntil) {
            sleepUntil = evtTime;
        }
    }
    SleepT::sleep_until(sleepUntil);
}

template <typename Interface> bool Scheduler<Interface>::isEventNear(const OutputEvent &evt) const {
    auto thresh = EventClockT::now() + std::chrono::microseconds(20);
    return interface.schedTime(evt.time()) <= thresh;
}

template <typename Interface> bool Scheduler<Interface>::isEventTime(const OutputEvent &evt) const {
    return interface.schedTime(evt.time()) <= EventClockT::now();
}

#endif
