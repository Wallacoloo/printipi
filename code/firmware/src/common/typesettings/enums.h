#ifndef COMMON_TYPESETTINGS_ENUMS_H
#define COMMON_TYPESETTINGS_ENUMS_H


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
