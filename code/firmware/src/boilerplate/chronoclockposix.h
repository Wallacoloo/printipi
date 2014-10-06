#ifndef BOILERPLATE_CHRONOCLOCKPOSIX_H
#define BOILERPLATE_CHRONOCLOCKPOSIX_H

/*
 * gcc 4.6 doesn't support std::chrono::clock stuff.
 * gcc 4.7 on arm implements it incorrectly (later versions untested)
 * Therefore, we implement our own clock that has the same interface which we can use within Linux.
*/

#include <chrono>
#include <time.h>
//#include "common/timeutil.h"

class ChronoClockPosix {
    public:
        typedef std::chrono::nanoseconds duration;
        typedef duration::rep rep;
        typedef duration::period period;
        typedef std::chrono::time_point<ChronoClockPosix> time_point;
        static const bool is_steady = true;
        inline static time_point now() noexcept {
            //struct timespec tnow = timespecNow();
            struct timespec tnow;
            clock_gettime(CLOCK_MONOTONIC, &tnow);
            return time_point(std::chrono::seconds(tnow.tv_sec) + std::chrono::nanoseconds(tnow.tv_nsec));
        }
};


#endif
