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

#include <inttypes.h> //for PRId64

#include "typesettings/clocks.h"
#include "typesettings/schedinterfacehardwarescheduler.h"

//allow for generation of code that still works in high-latency enviroments, like valgrind
#ifdef DRUNNING_IN_VM
	#define RUNNING_IN_VM 1
#else
	#define RUNNING_IN_VM 0
#endif

#ifndef PRId64
	#define PRId64 "lld"
#endif

#include "typesettings/primitives.h"
#include "typesettings/enums.h"

template <typename T> StepDirection stepDirFromSign(T dir) {
	return dir < 0 ? StepBackward : StepForward;
}
template <typename T> T stepDirToSigned(StepDirection dir) {
	return dir == StepBackward ? -1 : 1;
}

#endif
