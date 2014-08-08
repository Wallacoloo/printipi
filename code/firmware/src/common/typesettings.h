#ifndef TYPESETTINGS_H
#define TYPESETTINGS_H

/* 
 * Printipi/typesettings.h
 * (c) 2014 Colin Wallace
 *
 * This file contains typedefs and enums for types used throughout the program.
 */

#include <cstdint> //for uint8_t
//define a clock type
#if DTARGET_RPI == 1
	#define TARGET_RPI
	//use a lower-latency clock available on the rpi:
	#include "drivers/rpi/chronoclockrpi.h"
	typedef drv::rpi::ChronoClockRpi EventClockT;
#else
	#include "boilerplate/chronoclockposix.h"
	typedef ChronoClockPosix EventClockT;
#endif
//allow for generation of code that still works in high-latency enviroments, like valgrind
#ifdef DRUNNING_IN_VM
	#define RUNNING_IN_VM 1
#else
	#define RUNNING_IN_VM 0
#endif

typedef uint8_t AxisIdType;
typedef float CelciusType;

enum PositionMode {
	POS_ABSOLUTE,
	POS_RELATIVE,
	POS_UNDEFINED //Sadly, need a way to tie extruder coords to positioning coords in the case that it's undefined.
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
