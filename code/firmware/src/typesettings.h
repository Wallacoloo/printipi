#ifndef TYPESETTINGS_H
#define TYPESETTINGS_H

typedef char AxisIdType;

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
	StepForward,
	StepBackward
};

template <typename T> StepDirection stepDirFromSign(T dir) {
	return dir < 0 ? StepBackward : StepForward;
}

#endif
