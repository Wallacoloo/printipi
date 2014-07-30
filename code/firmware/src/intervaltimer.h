#ifndef INTERVALTIMER_H
#define INTERVALTIMER_H

/* Class implements a way to clock the time between two events (or between calls to a recurring event)
*/
#include "timeutil.h"

class IntervalTimer {
	timespec _last;
	public:
		IntervalTimer();
		const timespec& clock();
		const timespec& get();
		int clockCmp(const timespec &cmp, int dflt=0);
};



#endif
