#include "timeutil.h"

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
