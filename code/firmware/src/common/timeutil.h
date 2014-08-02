#ifndef COMMON_TIMEUTIL_H
#define COMMON_TIMEUTIL_H

/* 
 * Printipi/timeutil.h
 * (c) 2014 Colin Wallace
 *
 * This file provides convenience functions for dealing with Linux high-resolution timers.
 */

#include <time.h>

timespec timespecNow();
timespec timespecAdd(const struct timespec &a, const struct timespec &b);
timespec timespecSub(const struct timespec &a, const struct timespec &b);
//return -1 for a < b, 0 for a == b, 1 for a > b
int timespecCmp(const struct timespec &a, const struct timespec &b);
bool timespecLt(const struct timespec &a, const struct timespec &b);
inline bool timespecLte(const struct timespec &a, const struct timespec &b) {
	//if not b < a,
	//then b >= a
	//so a <= b
	return !timespecLt(b, a);
}
inline bool timespecGt(const struct timespec &a, const struct timespec &b) {
	//if not a <= b,
	//then a > b
	return !timespecLte(a, b);
}
float timespecToFloat(const struct timespec &a);
timespec floatToTimespec(float f);

#endif
