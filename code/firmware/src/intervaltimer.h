#ifndef INTERVALTIMER_H
#define INTERVALTIMER_H

/* 
 * Printipi/gparse/com.h
 * (c) 2014 Colin Wallace
 *
 * IntervalTimer provides a way to clock the time between two events (or between calls to a recurring event).
 * This can be used to detect when an input isn't being serviced regularaly enough (eg in src/drivers/tempcontrol.h)
 */
#include "timeutil.h"

class IntervalTimer {
	timespec _last;
	public:
		IntervalTimer();
		const timespec& clock();
		const timespec& get() const;
		int clockCmp(const timespec &cmp, int dflt=0);
};



#endif
