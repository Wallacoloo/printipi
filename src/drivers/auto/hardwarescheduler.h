#ifndef DRIVERS_AUTO_HARDWARESCHEDULER_H
#define DRIVERS_AUTO_HARDWARESCHEDULER_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_HARDWARESCHEDULER
    #include PLATFORM_DRIVER_HARDWARESCHEDULER
    typedef drv::TARGET_PLATFORM_LOWER::HardwareScheduler SchedInterfaceHardwareScheduler;
#else
    #include "drivers/generic/hardwarescheduler.h"
    typedef drv::generic::HardwareScheduler SchedInterfaceHardwareScheduler;
#endif

#endif
