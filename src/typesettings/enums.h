#ifndef TYPESETTINGS_ENUMS_H
#define TYPESETTINGS_ENUMS_H

enum CoordAxis {
    COORD_X,
    COORD_Y,
    COORD_Z,
    COORD_E
};

//Scheduler::Interface::onIdleCpu can be called with a flag indicating (roughly) how long it's been since it was last called.
enum OnIdleCpuIntervalT {
    OnIdleCpuIntervalShort,
    OnIdleCpuIntervalWide
};

#endif
