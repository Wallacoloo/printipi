#ifndef PLATFORMS_AUTO_THISTHREADSLEEP_H
#define PLATFORMS_AUTO_THISTHREADSLEEP_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_THISTHREADSLEEP
    #include PLATFORM_DRIVER_THISTHREADSLEEP
    typedef plat::TARGET_PLATFORM_LOWER::ThisThreadSleep SleepT;
#else

    #ifdef PLATFORM_DRIVER_CHRONOCLOCK
        //custom platform clock type. Must make ALL sleeps relative (unless platform also provides ThisThreadSleep
        #include "boilerplate/thisthreadsleepadapter.h"
        #include "platforms/generic/thisthreadsleep.h"
        typedef ThisThreadSleepAdapter<EventClockT, plat::generic::ThisThreadSleep> SleepT;
    #else
        //generic ChronoClock, so can use generic ThisThreadSleep
        #include "platforms/generic/thisthreadsleep.h"
        typedef plat::generic::ThisThreadSleep SleepT;
    #endif
    
#endif

#endif
