#ifndef PLATFORMS_GENERIC_CHRONOCLOCK
#define PLATFORMS_GENERIC_CHRONOCLOCK

/*
 * gcc 4.6 doesn't support std::chrono::clock stuff.
 * gcc 4.7 on arm implements it incorrectly (later versions untested)
 * Therefore, we implement our own clock that has the same interface which we can use within Linux (if on unix or apple).
 *   Otherwise, we default to std::chrono and hope for the best.
*/

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    #include <chrono>
    #include <time.h>
    namespace plat {
    namespace generic {
        class ChronoClock {
            public:
                typedef std::chrono::nanoseconds duration;
                typedef duration::rep rep;
                typedef duration::period period;
                typedef std::chrono::time_point<ChronoClock> time_point;
                static const bool is_steady = true;
                inline static time_point now() noexcept {
                    struct timespec tnow;
                    clock_gettime(CLOCK_MONOTONIC, &tnow);
                    return time_point(std::chrono::seconds(tnow.tv_sec) + std::chrono::nanoseconds(tnow.tv_nsec));
                }
        };
    }
    }
#else //default to C++11 clocks and hope for the best:
    #include <chrono>
    namespace plat {
    namespace generic {
        typedef std::chrono::steady_clock EventClockT;
    }
    }
#endif

#endif
