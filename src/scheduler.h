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
 * Printipi/scheduler.h
 *
 * The Scheduler controls program flow between tending communications and executing events at precise times.
 * It also allows for software PWM of any output.
 * It is designed to run in a single-threaded environment so it can have maximum control.
 * As such, the program should call Scheduler.yield() periodically if doing any long-running task.
 * Events can be queued with Scheduler.queue, and Scheduler.eventLoop should be called after any program setup is completed.
 */
#ifndef SCHEDULER_H
#define SCHEDULER_H


#include <cassert> //for assert
#include <array>
#include "event.h"
#include "outputevent.h"
#include "common/logging.h"
#include "common/intervaltimer.h"
#include "common/typesettings/compileflags.h"

#if USE_PTHREAD
    #include <pthread.h> //for pthread_setschedparam
#endif
#include "schedulerbase.h"

struct NullSchedAdjuster {
    //void reset() {}
    EventClockT::time_point adjust(EventClockT::time_point tp) const {
        return tp;
    }
    //void update(EventClockT::time_point) {}
};

template <typename Interface=NullSchedulerInterface> class Scheduler : public SchedulerBase {
    typedef NullSchedAdjuster SchedAdjuster;
    EventClockT::duration MAX_SLEEP; //need to call onIdleCpu handlers every so often, even if no events are ready.
    Interface interface;
    SchedAdjuster schedAdjuster;
    bool hasActiveEvent;
    public:
        void queue(const OutputEvent &evt);
        void schedPwm(AxisIdType idx, float duty, float maxPeriod);
        template <typename T> void setMaxSleep(T duration) {
            MAX_SLEEP = std::chrono::duration_cast<EventClockT::duration>(duration);
        }
        inline void setDefaultMaxSleep() {
            setMaxSleep(std::chrono::milliseconds(40));
        }
        Scheduler(Interface interface);
        //Event nextEvent(bool doSleep=true, std::chrono::microseconds timeout=std::chrono::microseconds(1000000));
        void initSchedThread() const; //call this from whatever threads call nextEvent to optimize that thread's priority.
        //EventClockT::time_point lastSchedTime() const; //get the time at which the last event is scheduled, or the current time if no events queued.
        bool isRoomInBuffer() const;
        void eventLoop();
        void yield(const OutputEvent *evt);
    private:
        void sleepUntilEvent(const OutputEvent *evt) const;
        bool isEventNear(const OutputEvent &evt) const;
        bool isEventTime(const OutputEvent &evt) const;
};

template <typename Interface> Scheduler<Interface>::Scheduler(Interface interface) 
    : interface(interface)
    ,hasActiveEvent(false)
    {
    setDefaultMaxSleep();
}


template <typename Interface> void Scheduler<Interface>::queue(const OutputEvent &evt) {
    hasActiveEvent = true;
    this->yield(&evt);
    hasActiveEvent = false;
}

template <typename Interface> void Scheduler<Interface>::schedPwm(AxisIdType idx, float duty, float maxPeriod) {
    duty = std::min(1.f, std::max(0.f, duty)); //clamp pwm between [0, 1]
    interface.iterPwmPins(idx, duty, [this, maxPeriod](int pin_, float duty_) {this->interface.queuePwm(pin_, duty_, maxPeriod); }); //note: some physical pins may be inverted, indicating duty must be switched, hence why it occurs as a parameter to the lambda
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
    while (1) {
        if (interface.onIdleCpu(intervalT)) {
            //intervalT = OnIdleCpuIntervalShort; //more cpu is needed; no delay
            intervalT = (++numShortIntervals % 2048) ? OnIdleCpuIntervalShort : OnIdleCpuIntervalWide;
        } else {
            intervalT = OnIdleCpuIntervalWide; //no cpu is needed; wide delay
            sleepUntilEvent(NULL);
        }
    }
}

template <typename Interface> void Scheduler<Interface>::yield(const OutputEvent *evt) {
    OnIdleCpuIntervalT intervalT = OnIdleCpuIntervalWide;
    int numShortIntervals = 0; //need to track the number of short cpu intervals, because if we just execute short intervals constantly for, say, 1 second, then certain services that only run at long intervals won't occur. So make every, say, 10000th short interval transform into a wide interval.
    while (!isEventTime(*evt)) {
        if (!interface.onIdleCpu(intervalT)) { //if we don't need any onIdleCpu, then either sleep for event or yield to rest of program:
            this->sleepUntilEvent(evt);
            //break; //don't break because sleepUntilEvent won't always do the full sleep
            intervalT = OnIdleCpuIntervalWide;
            //numShortIntervals = 0;
        } else {
            //after 2048 (just a nice binary number) short intervals, insert a wide interval instead:
            intervalT = (++numShortIntervals % 2048) ? OnIdleCpuIntervalShort : OnIdleCpuIntervalWide;
        }
    }
    interface.queue(*evt);
}

template <typename Interface> void Scheduler<Interface>::sleepUntilEvent(const OutputEvent *evt) const {
    //need to call onIdleCpu handlers occasionally - avoid sleeping for long periods of time.
    auto sleepUntil = EventClockT::now() + MAX_SLEEP;
    if (evt) { //allow calling with NULL to sleep for a configured period of time (MAX_SLEEP)
        //auto evtTime = schedAdjuster.adjust(evt->time());
        auto evtTime = interface.schedTime(schedAdjuster.adjust(evt->time()));
        if (evtTime < sleepUntil) {
            sleepUntil = evtTime;
        }
    }
    //LOGV("Scheduler::sleepUntilEvent: %ld.%08lu\n", sleepUntil.tv_sec, sleepUntil.tv_nsec);
    SleepT::sleep_until(sleepUntil);
}

template <typename Interface> bool Scheduler<Interface>::isEventNear(const OutputEvent &evt) const {
    auto thresh = EventClockT::now() + std::chrono::microseconds(20);
    return interface.schedTime(schedAdjuster.adjust(evt.time())) <= thresh;
}

template <typename Interface> bool Scheduler<Interface>::isEventTime(const OutputEvent &evt) const {
    return interface.schedTime(schedAdjuster.adjust(evt.time())) <= EventClockT::now();
}

#endif
