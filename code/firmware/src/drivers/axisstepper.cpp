#include "axisstepper.h"

#include <stdexcept>

namespace drv {

Event AxisStepper::getEvent() const {
	return Event::StepperEvent(this->time, this->index(), this->direction);
}

void AxisStepper::_nextStep() {
	//should be implemented in derivatives.
	throw std::runtime_error("AxisStepper::_nextStep() must be overriden in any child classes");
}

}
