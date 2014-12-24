#ifndef BOILERPLATE_THISTHREADSLEEPADAPTER_H
#define BOILERPLATE_THISTHREADSLEEPADAPTER_H

#include <chrono>

template <typename ClockT, typename SleepT> class ThisThreadSleepAdapter {
    //allows for sleeping to an absolute time when the custom clock (EventClockT) has a different offset than the system clock (which is otherwise used for measuring sleep times).
    //ClockT is the clock to which absolute times should be compared.
    //SleepT is a sleep function (like ThisThreadSleepPosix or std::this_thread) which can sleep for a RELATIVE TIME.
    public:
        template<class Clock, class Duration> static void sleep_until(const std::chrono::time_point<Clock, Duration> &sleep_time) {
            auto now = ClockT::now();
            auto rel = sleep_time.time_since_epoch() - now.time_since_epoch();
            sleep_for(rel);
        }
        template <class Rep, class Period> static void sleep_for(const std::chrono::duration<Rep, Period> &dur) {
            SleepT::sleep_for(dur);
        }
};

#endif
