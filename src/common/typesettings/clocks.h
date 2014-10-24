#include "drivers/auto/chronoclock.h"
#include "drivers/auto/thisthreadsleep.h"
/*#ifndef COMMON_TYPESETTINGS_CLOCKS_H
#define COMMON_TYPESETTINGS_CLOCKS_H

#include "compileflags.h"

#ifdef TARGET_RPI
    //use a lower-latency clock available on the rpi:
    #ifdef COMPILING_MAIN
        #warning "using drv::rpi::ChronoClockRpi for EventClockT. If your compilation target is not the Raspberry Pi, the program will crash on startup"
    #endif
    //#include "drivers/rpi/chronoclock.h"
    #include PLATFORM_DRIVER_CHRONOCLOCK
    //typedef drv::rpi::ChronoClock EventClockT;
    typedef drv::TARGET_PLATFORM_LOWER::ChronoClock EventClockT;
    //#include "boilerplate/chronoclockposix.h"
    //typedef ChronoClockPosix EventClockT;
    #include "drivers/generic/thisthreadsleep.h"
    //typedef ThisThreadSleepPosix SleepT
    #include "boilerplate/thisthreadsleepadapter.h"
    typedef ThisThreadSleepAdapter<EventClockT, drv::generic::ThisThreadSleep> SleepT;
#else
    #if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
        //use special posix clock
        #ifdef COMPILING_MAIN
            #warning "using ChronoClockPosix for EventClockT. While this does work, you will get better performance if you use a clock specific to your machine (eg make DTARGET_RPI=1 for the Raspberry Pi)"
        #endif
        #include "drivers/generic/chronoclock.h"
        typedef drv::generic::ChronoClock EventClockT;
        #include "drivers/generic/thisthreadsleep.h"
        typedef drv::generic::ThisThreadSleep SleepT;
    #else
        //use C++11 steady_clock. Hope it exists and is functional!
        #ifdef COMPILING_MAIN
            #warning "using std::chrono::steady_clock for EventClockT. steady_clock has been known to have improper implementations in gcc <= 4.6 for x86 and gcc <= 4.7 for arm"
        #endif
        #include <chrono>
        typedef std::chrono::steady_clock EventClockT;
        #include <thread>
        typedef std::this_thead SleepT;
    #endif
#endif

#endif*/
