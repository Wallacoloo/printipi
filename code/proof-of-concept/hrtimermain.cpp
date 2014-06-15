/* Kernel modules can handle sub-microsecond interrupts: http://raspberrypi.stackexchange.com/questions/8586/arm-timer-in-kernel-module-with-precision-less-than-microsecond
cat /proc/timer_list indicates that high-res timer (hrtimer) resolution is actually 1 ns
hrtimer is available to userspace programs. http://elinux.org/High_Resolution_Timers
Or is it kernel modules? http://raspberrypi.stackexchange.com/questions/8586/arm-timer-in-kernel-module-with-precision-less-than-microsecond
hrtimers are now used in Posix timers and nanosleep and itimers: http://lwn.net/Articles/168399/
  https://www.kernel.org/doc/Documentation/timers/hrtimers.txt
  
Distributions for testNanoSleep @ 1/2 second:
Ubunutu Laptop (40 samples): mean: 206546.75 ns, sd: 40484.71056 ns, about 40 uSec
*/

#include <time.h>
#include <stdio.h>

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
    endTime.tv_sec -= startTime.tv_sec;
    return (endTime.tv_sec - sleepDur.tv_sec)*1000000000 + (endTime.tv_nsec - startTime.tv_nsec - sleepDur.tv_nsec);
}


int main(int argc, char** argv) {
    getClockInfo();
    testNanoSleep();
    for (int i=0; i<40; ++i) {
        printf("%lu, ", testSleepPrecision());
    }
}
