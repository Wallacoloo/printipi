#include "event.h"
#include "timeutil.h" //for timespecLt


AxisIdType Event::stepperId() const {
	return this->_stepperNum;
}
StepDirection Event::direction() const {
	return this->_isForward ? StepForward : StepBackward;
}
const struct timespec& Event::time() const {
	return this->_time;
}

bool Event::isTime() const {
	//time to handle event if now >= _time
	return !timespecLt(timespecNow(), this->_time);
}

Event::Event(const timespec &t, AxisIdType stepperNum, StepDirection dir) : _time(t), _stepperNum(stepperNum), _isForward(dir==StepForward) {}

Event Event::StepperEvent(float relTime, AxisIdType stepperNum, StepDirection dir) {
	struct timespec t;
	t.tv_sec = relTime;
	t.tv_nsec = (relTime-t.tv_sec)*1000000000;
	return Event(t, stepperNum, dir);
}

void Event::offset(const struct timespec& offset) {
	this->_time.tv_sec += offset.tv_sec;
	this->_time.tv_nsec += offset.tv_nsec;
	if (this->_time.tv_nsec > 999999999) {
        this->_time.tv_sec += 1;
        this->_time.tv_nsec -= 1000000000;
    }
}

void Event::offsetNano(unsigned nsec) {
	this->_time.tv_nsec += nsec;
	if (this->_time.tv_nsec > 999999999) {
        this->_time.tv_sec += 1;
        this->_time.tv_nsec -= 1000000000;
    }
}

bool Event::operator<(const Event &other) {
	return timespecLt(this->time(), other.time());
}

