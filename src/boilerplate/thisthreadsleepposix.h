#ifndef BOILERPLATE_THISTHREADSLEEPPOSIX_H
#define BOILERPLATE_THISTHREADSLEEPPOSIX_H

/*
 * std::this_thread::sleep_until may be having issues when using custom clocks. Try this as a workaround if using Posix.
*/

#include <chrono>
#include <time.h>

class ThisThreadSleepPosix {
    public:
        template<class Clock, class Duration> static void sleep_until(const std::chrono::time_point<Clock, Duration> &sleep_time) {
            auto dur = sleep_time.time_since_epoch();
            auto durSec = std::chrono::duration_cast<std::chrono::seconds>(dur);
            auto durNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur) - durSec;
            timespec tsSleepUntil;
            tsSleepUntil.tv_sec = durSec.count();
            tsSleepUntil.tv_nsec = durNsec.count();
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tsSleepUntil, NULL);
        }
        template <class Rep, class Period> void sleep_for(const std::chrono::duration<Rep, Period> &dur) {
            auto durSec = std::chrono::duration_cast<std::chrono::seconds>(dur);
            auto durNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur) - durSec;
            timespec tsSleepUntil;
            tsSleepUntil.tv_sec = durSec.count();
            tsSleepUntil.tv_nsec = durNsec.count();
            clock_nanosleep(CLOCK_MONOTONIC, 0, &tsSleepUntil, NULL); //0 = time given is relative.
        }
};


#endif
