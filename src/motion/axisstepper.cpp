#include "axisstepper.h"

#include <cassert>

namespace motion {

void AxisStepper::_nextStep(bool useEndstops) {
    //should be implemented in derivatives.
    (void)useEndstops;
    assert(false && "AxisStepper::_nextStep() must be overriden in any child classes");
}

}
