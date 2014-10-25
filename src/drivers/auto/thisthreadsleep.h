#ifndef DRIVERS_AUTO_THISTHREADSLEEP_H
#define DRIVERS_AUTO_THISTHREADSLEEP_H

#include "common/typesettings/compileflags.h"

#ifdef PLATFORM_DRIVER_THISTHREADSLEEP
    #include PLATFORM_DRIVER_THISTHREADSLEEP
    typedef drv::TARGET_PLATFORM_LOWER::ThisThreadSleep ThisThreadSleep;
#else

    #ifdef PLATFORM_DRIVER_CHRONOCLOCK
        //custom platform clock type. Must make ALL sleeps relative (unless platform also provides ThisThreadSleep
        #include "boilerplate/thisthreadsleepadapter.h"
        #include "drivers/generic/thisthreadsleep.h"
        typedef ThisThreadSleepAdapter<EventClockT, drv::generic::ThisThreadSleep> SleepT;
    #else
        //generic ChronoClock, so can use generic ThisThreadSleep
        #include "drivers/generic/thisthreadsleep.h"
        typedef drv::generic::ThisThreadSleep SleepT;
    #endif
    
#endif

#endif
