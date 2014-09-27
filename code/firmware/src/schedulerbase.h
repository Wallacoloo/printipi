#ifndef SCHEDULERBASE_H
#define SCHEDULERBASE_H

#include <algorithm> //for std::min
#include <array>
#include <vector>
#include <cassert> //for assert

#include "outputevent.h"

//#include "common/typesettings.h" //for EventClockT (no! circular include!)

#ifndef SCHED_PRIORITY
    #define SCHED_PRIORITY 30
#endif
#ifndef SCHED_NUM_EXIT_HANDLER_LEVELS
    #define SCHED_NUM_EXIT_HANDLER_LEVELS 2
#endif
#define SCHED_IO_EXIT_LEVEL 0
#define SCHED_MEM_EXIT_LEVEL 1

class Event; //forward declaration to avoid inclusion of event.h (as event.h includes typesettings.h, which may include this file)

//Scheduler::Interface::onIdleCpu can be called with a flag indicating (roughly) how long it's been since it was last called.
enum OnIdleCpuIntervalT {
    OnIdleCpuIntervalShort,
    OnIdleCpuIntervalWide
};

struct PwmInfo {
    unsigned nsHigh;
    unsigned nsLow;
    PwmInfo() : nsHigh(0), nsLow(0) {}
    PwmInfo(float duty, float period) : 
        nsHigh(std::min(999999999, std::max(0, (int)(duty*period*1000000000)))), //clamp the times to >= 0 and <= 1
        nsLow(std::min(999999999, std::max(0, (int)((1-duty)*period*1000000000)))) {}
    inline float period() const {
        return nsHigh + nsLow;
    }
    inline float dutyCycle() const {
        return (float)nsHigh / period();
    }
    inline bool isNonNull() const {
        return nsHigh || nsLow;
    }
};

/* Base class from which all templated schedulers derive.
Defines things such as exit handlers */
class SchedulerBase {
    static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
    //static std::atomic<bool> isExiting; //typical implementations of exit() call the exit handlers from within the thread that called exit. Therefore, if the exiting thread causes another thread to call exit(), this value must be atomic.
    static bool isExiting;
    private:
        static void callExitHandlers();
    public:
        static void configureExitHandlers();
        static void registerExitHandler(void (*handler)(), unsigned level);
};

/* A Scheduler is useless without an interface. In order to use a scheduler, implement a class with the following methods
and pass it as a template argument to the Scheduler<Interface> type. */
struct DefaultSchedulerInterface {
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
            template <typename EventClockT_time_point> EventClockT_time_point schedTime(EventClockT_time_point evtTime) const {
                //If an event needs to occur at evtTime, this function should return the earliest time at which it can be scheduled.
                //This function is only templated to prevent importing typesettings.h (circular import), required for the real EventClockT. An implementation only needs to support the EventClockT::time_point defined in common/typesettings.h
                return evtTime;
            }
        };
    private:
        HardwareScheduler _hardwareScheduler;
    public:
        inline bool onIdleCpu() {
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
