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
According to Rostock wiki, average 30 steps/mm, max at 300 steps/mm. http://reprap.org/wiki/Rostock#What.27s_the_resolution_in_X.2FY_direction.3F
  Then max is 300*100 steps/sec = 30000 steps/sec = 1 step per 33 uSec

Distributions for testSleepAndSpinPrecision() @ 1/2 second:
  Ubuntu Laptop (40 samples, 240000 ns buffer): mean: 2784.875 ns, sd: 7604.23667 ns (multiple outliers)
  Raspberry Pi (40 samples, 130000 ns buffer): mean: 2315.925 ns, sd: 9485.23442 ns (14 ns w/o the outlier)
  Raspberry Pi (40 samples, 130000 ns buffer, nice: -10): mean: 1911.775, sd: 8231.99072
  Raspberry Pi (40 samples, 130000 ns buffer, nice: -20): mean: 388.525, sd: 1745.81225 (repeatable thrice to within sd=2500 ns)
    RPi appears to take 1 uSec to gettime. (samples looked like 5 4 3 2 1 0 999 998 997 ...)
  Raspberry Pi (40 samples, 180000 ns buffer, nice: -20, costOfGetTime): mean: -94.85, sd: 158.41726 (almost always sd<2500 ns. Saw 7000 ns ONCE)
  
Can use Linux process "niceness"; -20 for most priority, +19 for least.
Is there a delayUs function to get finer accuracy over the gettime loop?
  - wiringPi contains a delayMicroseconds function. http://www.raspberrypi.org/forums/viewtopic.php?f=33&t=17688
  - appears that the raspberry pi's hardware clock only supports microsecond precision (exactly):
    https://projects.drogon.net/accurate-delays-on-the-raspberry-pi/
  - Knowing the clockrate, one can delay for a specific number of cycles (600 MHz = 600 cycles / uS)
  
More realtime:
  thread about realtime kernel: http://www.raspberrypi.org/forums/viewtopic.php?t=2376
    Real-time Application Interface: www.rtai.org (OUT OF DATE - use Xenomai)
  SO post about rt on raspberry pi: http://raspberrypi.stackexchange.com/questions/1408/is-it-possible-to-run-real-time-software
    Secondary RT kernel: http://www.xenomai.org/
      runs side-by-side with normal Linux kernel
  Stackable arduino (sits right on top RPi): http://wyolum.com/shop/25-alamode.html
  enable CONFIG_PREEMPT_RT in kernel: http://www.emlid.com/raspberry-pi-real-time-kernel-available-for-download/
  * Stepper driving on RPi: https://www.youtube.com/watch?v=uIXkvz1-weQ
    Explains to use clock_nanosleep instead of nanosleep, in order to specify a time to sleep TO, rather than a duration to sleep.
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, timespec, NULL)
    Use (to increase priority): struct sched_param sp; sp.sched_priority=30; pthread_setthreadparam(pthread_self(), SCHED_FIFO, &sp)
      Note: piHiPri does exactly this.
    Use (to prevent mem-swapping): mlockall(MCL_FUTURE|MCL_CURRENT)
  Linux RT wiki: https://rt.wiki.kernel.org/index.php/Main_Page
*/

#include <time.h>
#include <stdio.h>

long avgCostOfGetTime;

timespec* timespec_add(struct timespec * tv_a, const struct timespec * tv_b) { //a is modified
    tv_a->tv_sec += tv_b->tv_sec;
    tv_a->tv_nsec += tv_b->tv_nsec;
   
    if (tv_a->tv_nsec > 999999999) {
        tv_a->tv_sec += 1;
        tv_a->tv_nsec -= 1000000000;
    }
    return tv_a;
}
timespec* timespec_sub(struct timespec * tv_a, const struct timespec * tv_b) { //a is modified
	tv_a->tv_sec -= tv_b->tv_sec;
    tv_a->tv_nsec -= tv_b->tv_nsec;
   
    if (tv_a->tv_nsec < 0) {
        tv_a->tv_sec -= 1;
        tv_a->tv_nsec += 1000000000;
    }
    return tv_a;
}
long timespec_to_nano(struct timespec *t) {
	return (long)t->tv_sec * 1000000000 + (long)t->tv_nsec;
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
	struct timespec sleepDur, remaining, startTime, curTime, desiredEndTime, sleepPad;
    sleepDur.tv_sec = 0;
    sleepDur.tv_nsec = 500000000;
    sleepPad.tv_sec = 0;
    sleepPad.tv_nsec = 180000;
    //endTime.tv_sec = 0;
    //endTime.tv_nsec = 0;
    clock_gettime(0, &startTime);
    desiredEndTime.tv_sec = startTime.tv_sec;
    desiredEndTime.tv_nsec = startTime.tv_nsec;
    timespec_add(&desiredEndTime, &sleepDur);
    timespec_sub(&sleepDur, &sleepPad);
    nanosleep(&sleepDur, &remaining);
    //while (endTime.tv_sec < desiredEndTime.tv_sec || endTime.tv_nsec < desiredEndTime.tv_nsec) {
    do {
    	clock_gettime(0, &curTime);
    } while (timespec_to_nano(timespec_sub(&curTime, &desiredEndTime)) < -avgCostOfGetTime/2);
    //timespec_sub(&endTime, &desiredEndTime);
    //return timespec_to_nano(&endTime);
    return timespec_to_nano(&curTime);
}

long costOfGetTime() {
	struct timespec t1, t2;
	clock_gettime(0, &t1);
	clock_gettime(0, &t2);
	timespec_sub(&t2, &t1);
	return timespec_to_nano(&t2);
}

long getAvgCostOfGetTime(int n=40) {
	long sum = 0;
	for (int i=0; i<n; ++i) {
		sum += costOfGetTime();
	}
	return sum/n;
}

int main(int argc, char** argv) {
	struct timespec a, b;
	a.tv_sec = 0;
	a.tv_nsec = 1;
	b.tv_sec = 0;
	b.tv_nsec = 2;
	printf("Test timespec_sub: %ld\n", timespec_to_nano(timespec_sub(&a, &b)));
    getClockInfo();
    avgCostOfGetTime = getAvgCostOfGetTime();
    printf("Average cost of gettime: %lu\n", avgCostOfGetTime);
    testNanoSleep();
    for (int i=0; i<40; ++i) {
        //printf("%ld, \n", testSleepPrecision());
        printf("%ld, \n", testSleepAndSpinPrecision());
        //printf("%ld, \n", costOfGetTime());
    }
}
