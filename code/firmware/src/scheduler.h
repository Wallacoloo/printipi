#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include "event.h"

#include <pthread.h> //for pthread_setschedparam
#include <time.h> //for clock_nanosleep
#include "logging.h"

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
	//std::mutex allowPushMutex; //lock this when capacity is exceeded.
	bool _isPushLocked;
	std::condition_variable nonemptyCond;
	public:
		void queue(const Event& evt);
		Scheduler();
		Event nextEvent();
};


#endif
