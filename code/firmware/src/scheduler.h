#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <time.h> //for timespec
#include <array>
#include <vector>
#include <atomic>
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
#ifndef SCHED_NUM_EXIT_HANDLER_LEVELS
	#define SCHED_NUM_EXIT_HANDLER_LEVELS 2
#endif
#define SCHED_IO_EXIT_LEVEL 0
#define SCHED_MEM_EXIT_LEVEL 1


class Scheduler {
	std::queue<Event> eventQueue;
	mutable std::mutex mutex;
	std::unique_lock<std::mutex> _lockPushes;
	bool _arePushesLocked;
	std::condition_variable nonemptyCond;
	struct timespec lastEventHandledTime;
	unsigned bufferSize;
	static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
	static std::atomic<bool> isExiting; //typical implementations of exit() call the exit handlers from within the thread that called exit. Therefore, if the exiting thread causes another thread to call exit(), this value must be atomic.
	private:
		static void callExitHandlers();
	public:
		static void configureExitHandlers();
		static void registerExitHandler(void (*handler)(), unsigned level);
		//queue and nextEvent can be called from separate threads, but nextEvent must NEVER be called from multiple threads.
		void queue(const Event& evt);
		Scheduler();
		Event nextEvent();
		void initSchedThread(); //call this from whatever threads call nextEvent to optimize that thread's priority.
		struct timespec lastSchedTime() const; //get the time at which the last event is scheduled, or the current time if no events queued.
		void setBufferSize(unsigned size);
		unsigned getBufferSize() const;
};


#endif
