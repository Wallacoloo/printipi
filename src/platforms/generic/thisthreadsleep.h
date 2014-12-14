#ifndef PLATFORMS_GENERIC_THISTHREADSLEEP_H
#define PLATFORMS_GENERIC_THISTHREADSLEEP_H

/*
 * std::this_thread::sleep_until may be having issues when using custom clocks. Try this as a workaround if using Posix.
*/

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))

    #include <chrono>
    #include <time.h>

    namespace plat {
    namespace generic {

    class ThisThreadSleep {
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
            template <class Rep, class Period> static void sleep_for(const std::chrono::duration<Rep, Period> &dur) {
                auto durSec = std::chrono::duration_cast<std::chrono::seconds>(dur);
                auto durNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur) - durSec;
                timespec tsSleepUntil;
                tsSleepUntil.tv_sec = durSec.count();
                tsSleepUntil.tv_nsec = durNsec.count();
                clock_nanosleep(CLOCK_MONOTONIC, 0, &tsSleepUntil, NULL); //0 = time given is relative.
            }
    };

    }
    }
    
#else //default to C++11 thread interface and hope for the best:
    #include <thread>
    namespace plat {
    namespace generic {
        typedef std::this_thead SleepT;
    }
    }

#endif

#endif
