#include "axisstepper.h"

#include <cassert>

namespace drv {

Event AxisStepper::getEvent() const {
    return Event::StepperEvent(this->time, this->index(), this->direction);
}
Event AxisStepper::getEvent(float realTime) const {
    return Event::StepperEvent(realTime, this->index(), this->direction);
}

void AxisStepper::_nextStep() {
    //should be implemented in derivatives.
    assert(false && "AxisStepper::_nextStep() must be overriden in any child classes");
}

}
