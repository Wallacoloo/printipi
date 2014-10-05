#include "event.h"
//#include "common/timeutil.h" //for timespecLt


/*AxisIdType Event::stepperId() const {
	return this->_stepperNum;
}
StepDirection Event::direction() const {
	return this->_isForward ? StepForward : StepBackward;
}
EventClockT::time_point Event::time() const {
	return this->_time;
}*/

/*bool Event::isTime() const {
	//time to handle event if now >= _time
	return !timespecLt(timespecNow(), this->_time);
}*/

/*bool Event::isNull() const {
	return this->stepperId() == NULL_STEPPER_ID;
}*/

//Event::Event(EventClockT::time_point t, AxisIdType stepperNum, StepDirection dir) : _time(t), _stepperNum(stepperNum), _isForward(dir==StepForward) {}

Event Event::StepperEvent(float relTime, AxisIdType stepperNum, StepDirection dir) {
	/*struct timespec t;
	t.tv_sec = relTime;
	t.tv_nsec = (relTime-t.tv_sec)*1000000000;
	return Event(timespecToTimepoint<EventClockT::time_point>(t), stepperNum, dir);*/
	auto t = std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(relTime));
	return Event(EventClockT::time_point(t), stepperNum, dir);
}

/*void Event::offset(const EventClockT::duration &offset) {
	this->_time += offset;
}*/
/*void Event::offsetNano(unsigned nsec) {
	this->offset(std::chrono::duration_cast<EventClockT::duration>(std::chrono::nanoseconds(nsec)));
}*/

/*bool Event::operator<(const Event &other) const {
	return this->time() < other.time();
}

bool Event::operator>(const Event &other) const {
	return this->time() > other.time();
}*/

