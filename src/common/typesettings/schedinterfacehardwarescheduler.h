#include "drivers/auto/hardwarescheduler.h"
/*#ifndef COMMON_TYPESETTINGS_SCHEDINTERFACEHARDWARESCHEDULER_H
#define COMMON_TYPESETTINGS_SCHEDINTERFACEHARDWARESCHEDULER_H

#include "compileflags.h"

#ifdef TARGET_RPI
    #ifndef NO_DMA
        //#include "drivers/rpi/hardwarescheduler.h"
        #include PLATFORM_DRIVER_HARDWARESCHEDULER
        typedef drv::TARGET_PLATFORM_LOWER::HardwareScheduler SchedInterfaceHardwareScheduler;
    #else
        #include "drivers/rpi/dumbhardwarescheduler.h"
        typedef drv::rpi::DumbHardwareScheduler SchedInterfaceHardwareScheduler;
    #endif
#else
    #include "schedulerbase.h"
    typedef NullSchedulerInterface::HardwareScheduler SchedInterfaceHardwareScheduler;
#endif

#endif*/
