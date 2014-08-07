#include "intervaltimer.h"


//IntervalTimer::IntervalTimer() : _last({0, 0}) {
//}
IntervalTimer::IntervalTimer() : _last() {
}
void IntervalTimer::reset() {
	//_last = {0, 0};
	_last = EventClockT::time_point();
}
const EventClockT::time_point& IntervalTimer::clock() {
	//_last = timespecNow();
	_last = EventClockT::now();
	return _last;
}
const EventClockT::time_point& IntervalTimer::get() const {
	return _last;
}
int IntervalTimer::clockCmp(const EventClockT::time_point &cmp, int dflt) {
	//return -1 if elapsed time is < cmp, +1 if > cmp, 0 if == cmp
	int ret;
	EventClockT::time_point now = EventClockT::now();
	if (_last == EventClockT::time_point()) { //no last time
		ret = dflt;
	} else {
		ret = now > cmp ? 1 : (now < cmp ? -1 : 0);
	}
	/*timespec now = timespecNow();
	if (_last.tv_sec == 0 && _last.tv_nsec == 0) { //no last time
		ret = dflt;
	} else {
		ret = timespecCmp(timespecSub(now, _last), cmp);
	}*/
	_last = now;
	return ret;
}
