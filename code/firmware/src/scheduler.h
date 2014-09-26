#ifndef SCHEDULER_H
#define SCHEDULER_H

/* 
 * Printipi/scheduler.h
 * (c) 2014 Colin Wallace
 *
 * The Scheduler controls program flow between tending communications and executing events at precise times.
 * It also allows for software PWM of any output.
 * It is designed to run in a single-threaded environment so it can have maximum control.
 * As such, the program should call Scheduler.yield() periodically if doing any long-running task.
 * Events can be queued with Scheduler.queue, and Scheduler.eventLoop should be called after any program setup is completed.
 */


#include <cassert> //for assert
#include <set>
//#include <thread> //for this_thread::sleep_until
//#include <time.h> //for timespec
//#include <chrono> 
#include <array>
#include <vector>
//#include <atomic>
#include <tuple>
#include "event.h"
#include "outputevent.h"
#include "common/logging.h"
//#include "common/timeutil.h"
#include "common/intervaltimer.h"
#include "common/suresleep.h"

#include <pthread.h> //for pthread_setschedparam
#include "schedulerbase.h"

struct NullSchedAdjuster {
    void reset() {}
    EventClockT::time_point adjust(EventClockT::time_point tp) const {
        return tp;
    }
    void update(EventClockT::time_point) {}
};

struct SchedAdjusterAccel {
    /* The logic is a bit odd here, but the idea is to compensate for missed events.
      If the scheduler isn't serviced on time, we don't want 10 backed-up events all happening at the same time. Instead, we offset them and pick them up then. We can never execute events with intervals smaller than they would register - this would indicate real movement of, say, 70mm/sec when the user only asked for 60mm/sec. Thus the scheduler can never be made "on track" again, unless there is a gap in scheduled events.
      If, because of the stall, actual velocity was decreased to 10mm/sec, we cannot jump instantly back to 60mm/sec (this would certainly cause MORE missed steps)! Instead, we accelerate back up to it.
      The tricky bit is - how do we estimate what the actual velocity is? We don't want to overcompensate. Unfortunately for now, some of the logic might :P */
    //static constexpr float a = -5.0;
    static constexpr float a = -12.0;
    IntervalTimer lastRealTime;
    EventClockT::time_point lastSchedTime;
    float lastSlope;
    SchedAdjusterAccel() : lastSchedTime(), lastSlope(1) {}
    /* Reset() should destroy any offset, so that adjust(time.now()) gives time.now() */
    void reset() {
        lastRealTime.reset();
        lastSchedTime = EventClockT::time_point();
        lastSlope = 1;
    }
    EventClockT::time_point adjust(EventClockT::time_point tp) const {
        //SHOULD work precisely with x0, y0 = (0, 0)
        float s_s0 = std::chrono::duration_cast<std::chrono::duration<float> >(tp - lastSchedTime).count();
        float offset;
        if (s_s0 < (1.-lastSlope)/2./a) { //accelerating:
            offset = a*s_s0*s_s0 + lastSlope*s_s0;
        } else { //stabilized:
            offset = (1.-lastSlope)*(1.-lastSlope)/-4/a + s_s0;
        }
        EventClockT::time_point ret(lastRealTime.get() + std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(offset)));
        if (ret < tp) {
            LOGV("SchedAdjuster::adjust adjusted into the past!\n");
        }
        LOGV("SchedAdjuster::adjust, (a, lastSlope), s_s0[b-a], offset[R, m]: (%f, %f) %f[%" PRId64 "-%" PRId64 "], %f[%" PRId64 "->%" PRId64 "]\n", a, lastSlope, s_s0, tp.time_since_epoch().count(), lastSchedTime.time_since_epoch().count(), offset, lastRealTime.get().time_since_epoch().count(), ret.time_since_epoch().count());
        return ret;
    }
    //call this when the event scheduled at time t is actually run.
    void update(EventClockT::time_point tp) {
        //SHOULD work reasonably with x0, y0 = (0, 0)
        auto y0 = lastRealTime.get();
        if (EventClockT::now()-y0 > std::chrono::milliseconds(50)) {
            //only sample every few ms, to mitigate Events scheduled on top of eachother.
            auto y1 = lastRealTime.clock();
            //the +X.XXX is to prevent a division-by-zero, and to minimize the effect that small sched errors have on the timeline:
            auto avgSlope = (std::chrono::duration_cast<std::chrono::duration<float> >(y1 - y0).count()+0.030) / (0.030+std::chrono::duration_cast<std::chrono::duration<float> >(tp-lastSchedTime).count());
            lastSlope = std::max(1., std::min(RUNNING_IN_VM ? 1. : 20., 2.*avgSlope - lastSlope)); //set a minimum for the speed that can be run at. The max(1,...) is because avgSlope can be smaller than the actual value due to the +0.030 on top and bottom.
            lastSchedTime = tp;
        }
    }
};

template <typename Interface=DefaultSchedulerInterface> class Scheduler : public SchedulerBase {
    typedef NullSchedAdjuster SchedAdjuster;
    EventClockT::duration MAX_SLEEP; //need to call onIdleCpu handlers every so often, even if no events are ready.
    Interface interface;
    SchedAdjuster schedAdjuster;
    bool hasActiveEvent;
    EventClockT::time_point _lastSchedTime;
    public:
        void queue(const Event &evt);
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
        EventClockT::time_point lastSchedTime() const; //get the time at which the last event is scheduled, or the current time if no events queued.
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
    ,_lastSchedTime(EventClockT::now())
    {
    //clock_gettime(CLOCK_MONOTONIC, &(this->lastEventHandledTime)); //initialize to current time.
    setDefaultMaxSleep();
}


template <typename Interface> void Scheduler<Interface>::queue(const Event& evt) {
    //assert(isRoomInBuffer());
    //this->orderedInsert(evt, INSERT_BACK);
    
    //Turn the event into an output event sequence, and then queue those individual events:
    //assert(interface.isEventOutputSequenceable(evt));
    hasActiveEvent = true;
    interface.iterEventOutputSequence(evt, [this](const OutputEvent &out) {this->queue(out); });
    hasActiveEvent = false;
}

template <typename Interface> void Scheduler<Interface>::queue(const OutputEvent &evt) {
    _lastSchedTime = evt.time();
    this->yield(&evt);
}

template <typename Interface> void Scheduler<Interface>::schedPwm(AxisIdType idx, float duty, float maxPeriod) {
    interface.iterPwmPins(idx, duty, [this](int pin_, float duty_) {this->interface.queuePwm(pin_, duty_); });
}

template <typename Interface> void Scheduler<Interface>::initSchedThread() const {
    struct sched_param sp; 
    sp.sched_priority=SCHED_PRIORITY; 
    if (int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
        LOGW("Warning: pthread_setschedparam (increase thread priority) at scheduler.cpp returned non-zero: %i\n", ret);
    }
}

template <typename Interface> EventClockT::time_point Scheduler<Interface>::lastSchedTime() const {
    //TODO: Note, this method, as-is, is const!
    /*if (this->outputEventQueue.empty()) {
        const_cast<Scheduler<Interface>*>(this)->schedAdjuster.reset(); //we have no events; no need to preserve *their* reference times, so reset for simplicity.
        return EventClockT::now(); //an alternative is to not use ::now(), but instead a time set in the past that is scheduled to happen now. Minimum intervals are conserved, so there's that would actually work decently. In actuality, the eventQueue will NEVER be empty except at initialization, because it handles pwm too.
    } else {
        return this->outputEventQueue.rbegin()->time();
    }*/
    //return EventClockT::now();
    auto now = EventClockT::now();
    return _lastSchedTime < now ? now : _lastSchedTime;
}

template <typename Interface> bool Scheduler<Interface>::isRoomInBuffer() const {
    return !hasActiveEvent;
}


template <typename Interface> void Scheduler<Interface>::eventLoop() {
    OnIdleCpuIntervalT intervalT = OnIdleCpuIntervalWide;
    while (1) {
        if (interface.onIdleCpu(intervalT)) {
            intervalT = OnIdleCpuIntervalShort; //more cpu is needed; no delay
        } else {
            intervalT = OnIdleCpuIntervalWide; //no cpu is needed; wide delay
            sleepUntilEvent(NULL);
        }
    }
}

template <typename Interface> void Scheduler<Interface>::yield(const OutputEvent *evt) {
    OnIdleCpuIntervalT intervalT = OnIdleCpuIntervalWide;
    while (!isEventTime(*evt)) {
        if (!interface.onIdleCpu(intervalT)) { //if we don't need any onIdleCpu, then either sleep for event or yield to rest of program:
            this->sleepUntilEvent(evt);
            //break; //don't break because sleepUntilEvent won't always do the full sleep
            intervalT = OnIdleCpuIntervalWide;
        } else {
            intervalT = OnIdleCpuIntervalShort;
        }
    }
    /*OutputEventQueueType::const_iterator iter = this->outputEventQueue.cbegin();
    OutputEvent evt = *iter;
    this->outputEventQueue.erase(iter); //iterator unaffected even if other events were inserted OR erased.
    interface.queue(evt);*/
    interface.queue(*evt);
    
    
    //manage PWM events:
    /*const PwmInfo &pwm = pwmInfo[evt.stepperId()];
    if (pwm.isNonNull()) {
        Event nextPwm;
        //         for | back
        // nsLow    0     1
        // nsHigh   1     0 
        //dir = (nsLow ^ for)
        if (evt.direction() == StepForward) {
            //next event will be StepBackward, or refresh this event if there is no off-duty.
            nextPwm = Event(evt.time(), evt.stepperId(), pwm.nsLow ? StepBackward : StepForward);
            //nextPwm.offsetNano(pwm.nsHigh);
            nextPwm.offset(std::chrono::nanoseconds(pwm.nsHigh));
        } else {
            //next event will be StepForward, or refresh this event if there is no on-duty.
            nextPwm = Event(evt.time(), evt.stepperId(), pwm.nsHigh ? StepForward : StepBackward);
            //nextPwm.offsetNano(pwm.nsLow);
            nextPwm.offset(std::chrono::nanoseconds(pwm.nsLow));
        }
        this->orderedInsert(nextPwm, INSERT_FRONT);
    }*/
    //this->eventQueue.pop_front(); //this is OK to put after PWM generation, because the next PWM event will ALWAYS occur after the current pwm event, so the queue front won't change. Furthermore, if interface.onEvent(evt) generates a new event (which it shouldn't), it most probably won't be scheduled for the past.
    //forceWait = false; //avoid draining ALL events - just drain the first.
}

template <typename Interface> void Scheduler<Interface>::sleepUntilEvent(const OutputEvent *evt) const {
    //need to call onIdleCpu handlers occasionally - avoid sleeping for long periods of time.
    bool doSureSleep = false;
    auto sleepUntil = EventClockT::now() + MAX_SLEEP;
    if (evt) { //allow calling with NULL to sleep for a configured period of time (MAX_SLEEP)
        //auto evtTime = schedAdjuster.adjust(evt->time());
        auto evtTime = interface.schedTime(schedAdjuster.adjust(evt->time()));
        if (evtTime < sleepUntil) {
            sleepUntil = evtTime;
            doSureSleep = true;
        }
    }
    //LOGV("Scheduler::sleepUntilEvent: %ld.%08lu\n", sleepUntil.tv_sec, sleepUntil.tv_nsec);
    if (doSureSleep) {
        SureSleep::sleep_until(sleepUntil);
    } else {
        SleepT::sleep_until(sleepUntil);
    }
}

template <typename Interface> bool Scheduler<Interface>::isEventNear(const OutputEvent &evt) const {
    auto thresh = EventClockT::now() + std::chrono::microseconds(20);
    //return schedAdjuster.adjust(evt.time()) <= thresh;
    return interface.schedTime(schedAdjuster.adjust(evt.time())) <= thresh;
}

template <typename Interface> bool Scheduler<Interface>::isEventTime(const OutputEvent &evt) const {
    //return schedAdjuster.adjust(evt.time()) <= EventClockT::now();
    return interface.schedTime(schedAdjuster.adjust(evt.time())) <= EventClockT::now();
}

#endif
