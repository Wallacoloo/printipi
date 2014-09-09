#ifndef COMMON_TYPESETTINGS_H
#define COMMON_TYPESETTINGS_H

/* 
 * Printipi/common/typesettings.h
 * (c) 2014 Colin Wallace
 *
 * This file contains typedefs and enums for types used throughout the program.
 * It also processes some command-line based build settings (defines).
 *
 * TODO: it may be beneficial to split this into different files, eg typesettings/eventclockt.h, typesettings/sleept.h.
 *   This way circular includes can be avoided inside types contained in typesettings that depend on other type settings.
 */

#include <cstdint> //for uint8_t
#include <inttypes.h> //for PRId64
//define a clock type
#if DTARGET_RPI == 1
	#define TARGET_RPI //provide a user-usable macro
	//use a lower-latency clock available on the rpi:
	#ifdef COMPILING_MAIN
		#warning "using drv::rpi::ChronoClockRpi for EventClockT. If your compilation target is not the Raspberry Pi, the program will crash on startup"
	#endif
	//#include "drivers/rpi/chronoclockrpi.h"
	//typedef drv::rpi::ChronoClockRpi EventClockT;
	#include "boilerplate/chronoclockposix.h"
	typedef ChronoClockPosix EventClockT;
	#include "boilerplate/thisthreadsleepposix.h"
	typedef ThisThreadSleepPosix SleepT;
	#include "drivers/rpi/dmascheduler.h"
	typedef drv::rpi::DmaScheduler SchedInterfaceHardwareScheduler;
#else
	#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
		//use special posix clock
		#ifdef COMPILING_MAIN
			#warning "using ChronoClockPosix for EventClockT. While this does work, you will get better performance if you use a clock specific to your machine (eg make DTARGET_RPI=1 for the Raspberry Pi)"
		#endif
		#include "boilerplate/chronoclockposix.h"
		typedef ChronoClockPosix EventClockT;
		#include "boilerplate/thisthreadsleepposix.h"
		typedef ThisThreadSleepPosix SleepT;
		#include "schedulerbase.h"
		typedef DefaultSchedulerInterface::HardwareScheduler SchedInterfaceHardwareScheduler;
	#else
		//use C++11 steady_clock. Hope it exists and is functional!
		#ifdef COMPILING_MAIN
			#warning "using std::chrono::steady_clock for EventClockT. steady_clock has been known to have improper implementations in gcc <= 4.6 for x86 and gcc <= 4.7 for arm"
		#endif
		#include <chrono>
		typedef std::chrono::steady_clock EventClockT;
		#include <thread>
		typedef std::this_thead SleepT;
		#include "schedulerbase.h"
		typedef DefaultSchedulerInterface::HardwareScheduler SchedInterfaceHardwareScheduler;
	#endif
#endif
//allow for generation of code that still works in high-latency enviroments, like valgrind
#ifdef DRUNNING_IN_VM
	#define RUNNING_IN_VM 1
#else
	#define RUNNING_IN_VM 0
#endif

#ifndef PRId64
	#define PRId64 "lld"
#endif

typedef uint8_t AxisIdType;
typedef float CelciusType;

enum PositionMode {
	POS_ABSOLUTE,
	POS_RELATIVE
};

enum LengthUnit {
	UNIT_MM,
	UNIT_IN
};

enum StepDirection {
	StepBackward,
	StepForward
};

enum CoordAxis {
	COORD_X,
	COORD_Y,
	COORD_Z,
	COORD_E
};

template <typename T> StepDirection stepDirFromSign(T dir) {
	return dir < 0 ? StepBackward : StepForward;
}
template <typename T> T stepDirToSigned(StepDirection dir) {
	return dir == StepBackward ? -1 : 1;
}

#endif
