#ifndef COMMON_TYPESETTINGS_SCHEDINTERFACEHARDWARESCHEDULER_H
#define COMMON_TYPESETTINGS_SCHEDINTERFACEHARDWARESCHEDULER_H

#if DTARGET_RPI == 1
	#define TARGET_RPI //provide a user-usable macro
	#include "drivers/rpi/dmascheduler.h"
	typedef drv::rpi::DmaScheduler SchedInterfaceHardwareScheduler;
#else
	#include "schedulerbase.h"
	typedef DefaultSchedulerInterface::HardwareScheduler SchedInterfaceHardwareScheduler;
#endif

#endif
