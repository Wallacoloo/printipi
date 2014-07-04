#include "event.h"


int Event::stepperNumber() const {
	return this->_stepperNum;
}
StepDirection Event::direction() const {
	return this->_isForward ? StepForward : StepBackward;
}
const struct timespec& Event::time() const {
	return this->_time;
}

Event Event::StepperEvent(float relTime, char stepperNum, StepDirection dir) {
	return Event();
}

