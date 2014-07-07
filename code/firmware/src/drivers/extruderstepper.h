/*#ifndef EXTRUDERSTEPPER_H
#define EXTRUDERSTEPPER_H

#include "linearstepper.h"

namespace drv {

template <int STEPS_PER_METER>
using ExtruderStepper = LinearStepper<STEPS_PER_METER, 'e'>; //not supported in gcc-4.6, so avoid use.

}

}

#endif*/
