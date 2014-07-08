#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
//#include <functional>
#include "event.h"

//#include <pthread.h> //for pthread_setschedparam
//#include <time.h> //for clock_nanosleep
//#include "logging.h"

#ifndef SCHED_PRIORITY
#define SCHED_PRIORITY 30
#endif
#ifndef SCHED_CAPACITY
#define SCHED_CAPACITY 1024
#endif
/*#ifndef SCHED_PUSH_SPIN_INTERVAL
#define SCHED_PUSH_SPIN_INTERVAL 100 //msec
#endif*/


class Scheduler {
	std::queue<Event> eventQueue;
	std::mutex mutex;
	std::unique_lock<std::mutex> _lockPushes;
	bool _arePushesLocked;
	std::condition_variable nonemptyCond;
	public:
		//queue and nextEvent can be called from separate threads, but nextEvent must NEVER be called from multiple threads.
		void queue(const Event& evt);
		Scheduler();
		Event nextEvent();
		void initSchedThread(); //call this from whatever threads call nextEvent to optimize that thread's priority.
};


#endif
