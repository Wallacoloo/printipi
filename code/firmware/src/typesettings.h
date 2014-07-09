#ifndef TYPESETTINGS_H
#define TYPESETTINGS_H

#include <cstdint>

typedef uint8_t AxisIdType;

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
