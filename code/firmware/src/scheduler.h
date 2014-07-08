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


class Scheduler {
	std::queue<Event> eventQueue;
	std::mutex mutex;
	std::condition_variable nonemptyCond;
	public:
		void queue(const Event& evt);
		//Scheduler();
		Event nextEvent();
};


#endif
