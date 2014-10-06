#ifndef COMMON_SURESLEEP_H
#define COMMON_SURESLEEP_H

#include "typesettings.h"
#include <chrono>


struct SureSleep {
    template<class Clock, class Duration> static void sleep_until(const std::chrono::time_point<Clock,Duration>& sleep_time) {
        //sleep until (120 uS) before the sleep_time:
        auto adjusted = sleep_time - std::chrono::microseconds(120);
        if (adjusted > EventClockT::now()) {
            SleepT::sleep_until(adjusted);
        }
        //now hard-delay until the right time:
        do {} while (EventClockT::now() < sleep_time);
    }
};



#endif
