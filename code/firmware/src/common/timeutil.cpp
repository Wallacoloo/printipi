#include "timeutil.h"

/*timespec timespecNow() {
	struct timespec curTime;
	clock_gettime(CLOCK_MONOTONIC, &curTime);
	return curTime;
}

timespec timespecAdd(const struct timespec &a, const struct timespec &b) {
	struct timespec ret;
	ret.tv_sec = a.tv_sec + b.tv_sec;
	ret.tv_nsec = a.tv_nsec + b.tv_nsec;
	
    if (ret.tv_nsec > 999999999) {
        ret.tv_sec += 1;
        ret.tv_nsec -= 1000000000;
    }
    return ret;
}

timespec timespecSub(const struct timespec &a, const struct timespec &b) {
	struct timespec ret;
	ret.tv_sec = a.tv_sec - b.tv_sec;
	if (a.tv_nsec >= b.tv_nsec) {
		ret.tv_nsec = a.tv_nsec - b.tv_nsec;
	} else {
		ret.tv_nsec = 1000000000 + a.tv_nsec - b.tv_nsec;
		ret.tv_sec -= 1;
	}
	return ret;
}

const timespec& timespecMin(const struct timespec &a, const struct timespec &b) {
	return timespecLt(a, b) ? a : b;
}

bool timespecLt(const struct timespec &a, const struct timespec &b) {
	return a.tv_sec < b.tv_sec || (a.tv_sec == b.tv_sec && a.tv_nsec < b.tv_nsec);
}

int timespecCmp(const struct timespec &a, const struct timespec &b) {
	return timespecLt(a, b) ? -1 : (timespecLt(b, a) ? 1 : 0);
}

float timespecToFloat(const struct timespec &a) {
	return a.tv_sec + a.tv_nsec/1000000000.;
}
timespec floatToTimespec(float f) {
	long sec = (long)f;
	long nsec = (f-sec)*1000000000;
	return timespec({sec, nsec});
}*/
