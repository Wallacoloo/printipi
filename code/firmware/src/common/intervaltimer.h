#ifndef COMMON_INTERVALTIMER_H
#define COMMON_INTERVALTIMER_H

/* 
 * Printipi/gparse/com.h
 * (c) 2014 Colin Wallace
 *
 * IntervalTimer provides a way to clock the time between two events (or between calls to a recurring event).
 * This can be used to detect when an input isn't being serviced regularaly enough (eg in src/drivers/tempcontrol.h)
 */
//#include "common/timeutil.h"
#include "common/typesettings.h"

class IntervalTimer {
	//timespec _last;
	EventClockT::time_point _last;
	public:
		IntervalTimer() : _last() {}
		inline void reset() {
			_last = EventClockT::time_point();
		}
		inline const EventClockT::time_point& clock() {
			return _last = EventClockT::now();
		}
		inline const EventClockT::time_point& get() const {
			return _last;
		}
		template <typename DurT> int clockCmp(const DurT &cmp, int dflt=0) {
			int ret;
			EventClockT::time_point now = EventClockT::now();
			if (_last == EventClockT::time_point()) { //no last time
				ret = dflt;
			} else {
				auto duration = now - _last;
				ret = duration > cmp ? 1 : (duration < cmp ? -1 : 0);
			}
			_last = now;
			return ret;
		}
};



#endif
