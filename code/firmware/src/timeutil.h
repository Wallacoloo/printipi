#ifndef TIMEUTIL_H
#define TIMEUTIL_H

#include <time.h>

timespec timespecNow();
timespec timespecAdd(const struct timespec &a, const struct timespec &b);
timespec timespecSub(const struct timespec &a, const struct timespec &b);
bool timespecLt(const struct timespec &a, const struct timespec &b);
float timespecToFloat(const struct timespec &a);

#endif
