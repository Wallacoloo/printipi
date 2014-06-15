/* Kernel modules can handle sub-microsecond interrupts: http://raspberrypi.stackexchange.com/questions/8586/arm-timer-in-kernel-module-with-precision-less-than-microsecond
cat /proc/timer_list indicates that high-res timer (hrtimer) resolution is actually 1 ns
hrtimer is available to userspace programs. http://elinux.org/High_Resolution_Timers
Or is it kernel modules? http://raspberrypi.stackexchange.com/questions/8586/arm-timer-in-kernel-module-with-precision-less-than-microsecond
hrtimers are now used in Posix timers and nanosleep and itimers: http://lwn.net/Articles/168399/
  https://www.kernel.org/doc/Documentation/timers/hrtimers.txt
  
Distributions for testSleepPrecision() @ 1/2 second:
  Ubuntu Laptop (40 samples): mean: 206546.75 ns, sd: 40484.71056 ns, about 40 uSec
  Raspberry Pi (40 samples): mean: 107167.375 ns, sd: 8504.03636 ns, about 8.5 uSec

What precision is needed?
Aiming for 100 mm/sec extrusion. Assume 10 steps per mm, then 1000 steps/sec = 1 step per 1000 uSec.

Distributions for testSleepAndSpinPrecision() @ 1/2 second:
  Ubuntu Laptop (40 samples, 240000 ns buffer): mean: 2784.875 ns, sd: 7604.23667 ns (multiple outliers)
  Raspberry Pi (40 samples, 130000 ns buffer): mean: 2315.925 ns, sd: 9485.23442 ns (14 ns w/o the outlier)
  Raspberry Pi (40 samples, 130000 ns buffer, nice: -10): mean: 1911.775, sd: 8231.99072
  Raspberry Pi (40 samples, 130000 ns buffer, nice: -20): mean: 388.525, sd: 1745.81225 (repeatable thrice to within sd=2500 ns)
    RPi appears to have resolution of 1 uSec (samples looked like 5 4 3 2 1 0 999 998 997 ...)
  
Can use Linux process "niceness"; -20 for most priority, +19 for least.
*/

#include <time.h>
#include <stdio.h>

void timespec_add(struct timespec * tv_a, const struct timespec * tv_b) { //a is modified
    tv_a->tv_sec += tv_b->tv_sec;
    tv_a->tv_nsec += tv_b->tv_nsec;
   
    if (tv_a->tv_nsec > 999999999) {
        tv_a->tv_sec += 1;
        tv_a->tv_nsec -= 1000000000;
    }
}
void timespec_sub(struct timespec * tv_a, const struct timespec * tv_b) { //a is modified
	tv_a->tv_sec -= tv_b->tv_sec;
    tv_a->tv_nsec -= tv_b->tv_nsec;
   
    if (tv_a->tv_nsec < 0) {
        tv_a->tv_sec -= 1;
        tv_a->tv_nsec += 1000000000;
    }
}
long timespec_to_nano(struct timespec *t) {
	return t->tv_sec * 1000000000 + t->tv_nsec;
}

void getClockInfo() {
    clockid_t types[] = {CLOCK_REALTIME, CLOCK_MONOTONIC, CLOCK_PROCESS_CPUTIME_ID, CLOCK_THREAD_CPUTIME_ID, (clockid_t)-1};

    struct timespec spec;
    for (int i=0; types[i] != (clockid_t)-1; i++) {
        if (clock_getres( types[i], &spec ) != 0) {
            printf("Timer %d not supported.\n", types[i]);
        } else {
            printf("Timer: %d, Seconds: %ld Nanos: %ld\n", i, spec.tv_sec, spec.tv_nsec);
        }
    }
}

void logTime(int clockId) {
    struct timespec time;
    clock_gettime(clockId, &time);
    printf("Time: %lu.%lu\n", time.tv_sec, time.tv_nsec);
} 

void testNanoSleep() {
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 500000000;
    logTime(0);
    nanosleep(&tim, &tim2);
    logTime(0);
}

long testSleepPrecision() {
    struct timespec sleepDur, remaining, startTime, endTime;
    sleepDur.tv_sec = 0;
    sleepDur.tv_nsec = 500000000;
    clock_gettime(0, &startTime);
    nanosleep(&sleepDur, &remaining);
    clock_gettime(0, &endTime);
    //endTime.tv_sec -= startTime.tv_sec;
    timespec_sub(&endTime, &sleepDur);
    timespec_sub(&endTime, &startTime);
    return timespec_to_nano(&endTime);
    //return (endTime.tv_sec - sleepDur.tv_sec)*1000000000 + (endTime.tv_nsec - startTime.tv_nsec - sleepDur.tv_nsec);
}

long testSleepAndSpinPrecision() {
	struct timespec sleepDur, remaining, startTime, endTime, desiredEndTime, sleepPad;
    sleepDur.tv_sec = 0;
    sleepDur.tv_nsec = 500000000;
    sleepPad.tv_sec = 0;
    sleepPad.tv_nsec = 130000;
    endTime.tv_sec = 0;
    endTime.tv_nsec = 0;
    clock_gettime(0, &startTime);
    desiredEndTime.tv_sec = startTime.tv_sec;
    desiredEndTime.tv_nsec = startTime.tv_nsec;
    timespec_add(&desiredEndTime, &sleepDur);
    timespec_sub(&sleepDur, &sleepPad);
    nanosleep(&sleepDur, &remaining);
    while (endTime.tv_sec < desiredEndTime.tv_sec || endTime.tv_nsec < desiredEndTime.tv_nsec) {
    	clock_gettime(0, &endTime);
    }
    timespec_sub(&endTime, &desiredEndTime);
    return timespec_to_nano(&endTime);
}

long costOfGetTime() {
	struct timespec t1, t2;
	clock_gettime(0, &t1);
	clock_gettime(0, &t2);
	timespec_sub(&t2, &t1);
	return timespec_to_nano(&t2);
}


int main(int argc, char** argv) {
    getClockInfo();
    testNanoSleep();
    for (int i=0; i<40; ++i) {
        //printf("%ld, \n", testSleepPrecision());
        printf("%ld, \n", testSleepAndSpinPrecision());
        //printf("%ld, \n", costOfGetTime());
    }
}
