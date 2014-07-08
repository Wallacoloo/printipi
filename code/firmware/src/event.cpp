#include "event.h"


AxisIdType Event::stepperId() const {
	return this->_stepperNum;
}
StepDirection Event::direction() const {
	return this->_isForward ? StepForward : StepBackward;
}
const struct timespec& Event::time() const {
	return this->_time;
}

Event::Event(const timespec &t, AxisIdType stepperNum, StepDirection dir) : _time(t), _stepperNum(stepperNum), _isForward(dir==StepForward) {}

Event Event::StepperEvent(float relTime, AxisIdType stepperNum, StepDirection dir) {
	struct timespec t;
	t.tv_sec = relTime;
	t.tv_nsec = (relTime-t.tv_sec)*1000000000;
	return Event(t, stepperNum, dir);
}

