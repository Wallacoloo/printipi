#include "axisstepper.h"

namespace drv {

Event AxisStepper::getEvent() const {
	return Event::StepperEvent(this->time, this->index, this->direction);
}

void AxisStepper::_nextStep() {
	//should be implemented in derivatives.
}

}
