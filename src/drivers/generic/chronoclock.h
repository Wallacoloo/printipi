#ifndef DRIVERS_GENERIC_CHRONOCLOCK
#define DRIVERS_GENERIC_CHRONOCLOCK

/*
 * gcc 4.6 doesn't support std::chrono::clock stuff.
 * gcc 4.7 on arm implements it incorrectly (later versions untested)
 * Therefore, we implement our own clock that has the same interface which we can use within Linux.
*/

namespace drv {
namespace generic {

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    #include <chrono>
    #include <time.h>
    class ChronoClock {
        public:
            typedef std::chrono::nanoseconds duration;
            typedef duration::rep rep;
            typedef duration::period period;
            typedef std::chrono::time_point<ChronoClock> time_point;
            static const bool is_steady = true;
            inline static time_point now() noexcept {
                //struct timespec tnow = timespecNow();
                struct timespec tnow;
                clock_gettime(CLOCK_MONOTONIC, &tnow);
                return time_point(std::chrono::seconds(tnow.tv_sec) + std::chrono::nanoseconds(tnow.tv_nsec));
            }
    };
#else
    #include <chrono>
    typedef std::chrono::steady_clock EventClockT;
    #include <thread>
    typedef std::this_thead SleepT;
#endif

}
}

#endif
