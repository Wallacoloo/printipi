#include "intervaltimer.h"


IntervalTimer::IntervalTimer() : _last({0, 0}) {
}
void IntervalTimer::reset() {
	_last = {0, 0};
}
const timespec& IntervalTimer::clock() {
	_last = timespecNow();
	return _last;
}
const timespec& IntervalTimer::get() const {
	return _last;
}
int IntervalTimer::clockCmp(const timespec &cmp, int dflt) {
	//return -1 if elapsed time is < cmp, +1 if > cmp, 0 if == cmp
	int ret;
	timespec now = timespecNow();
	if (_last.tv_sec == 0 && _last.tv_nsec == 0) { //no last time
		ret = dflt;
	} else {
		ret = timespecCmp(timespecSub(now, _last), cmp);
	}
	_last = now;
	return ret;
}
