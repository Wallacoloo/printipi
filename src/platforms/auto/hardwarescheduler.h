#ifndef PLATFORMS_AUTO_HARDWARESCHEDULER_H
#define PLATFORMS_AUTO_HARDWARESCHEDULER_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_HARDWARESCHEDULER
    #include PLATFORM_DRIVER_HARDWARESCHEDULER
    typedef plat::TARGET_PLATFORM_LOWER::HardwareScheduler HardwareScheduler;
#else
    #include "platforms/generic/hardwarescheduler.h"
    typedef plat::generic::HardwareScheduler HardwareScheduler;
#endif

#endif
