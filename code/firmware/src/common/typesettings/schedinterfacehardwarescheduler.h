#ifndef COMMON_TYPESETTINGS_SCHEDINTERFACEHARDWARESCHEDULER_H
#define COMMON_TYPESETTINGS_SCHEDINTERFACEHARDWARESCHEDULER_H

#include "compileflags.h"

#ifdef TARGET_RPI
    #ifndef NO_DMA
        #include "drivers/rpi/dmascheduler.h"
        typedef drv::rpi::DmaScheduler SchedInterfaceHardwareScheduler;
    #else
        #include "drivers/rpi/dumbhardwarescheduler.h"
        typedef drv::rpi::DumbHardwareScheduler SchedInterfaceHardwareScheduler;
    #endif
#else
    #include "schedulerbase.h"
    typedef NullSchedulerInterface::HardwareScheduler SchedInterfaceHardwareScheduler;
#endif

#endif
