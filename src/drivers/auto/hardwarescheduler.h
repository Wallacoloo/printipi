#ifndef DRIVERS_AUTO_HARDWARESCHEDULER_H
#define DRIVERS_AUTO_HARDWARESCHEDULER_H

#include "common/typesettings/compileflags.h"

#ifdef PLATFORM_DRIVER_HARDWARESCHEDULER
    #include PLATFORM_DRIVER_HARDWARESCHEDULER
    typedef drv::TARGET_PLATFORM_LOWER::HardwareScheduler SchedInterfaceHardwareScheduler;
#else
    #include "schedulerbase.h"
    typedef NullSchedulerInterface::HardwareScheduler SchedInterfaceHardwareScheduler;
#endif

#endif
