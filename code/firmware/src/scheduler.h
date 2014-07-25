#ifndef SCHEDULER_H
#define SCHEDULER_H

//#include <queue>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <time.h> //for timespec
#include <chrono> 
#include <array>
#include <vector>
#include <atomic>
#include <tuple>
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

struct PwmInfo {
	unsigned nsHigh;
	unsigned nsLow;
	PwmInfo() : nsHigh(0), nsLow(0) {}
	PwmInfo(float duty, float period) : 
		nsHigh(std::max(0, (int)(duty*period*1000000000))), //clamp the times to >= 0
		nsLow(std::max(0, (int)((1-duty)*period*1000000000))) {}
	float period() const {
		return nsHigh + nsLow;
	}
};

class Scheduler {
	std::array<PwmInfo, 256> pwmInfo; 
	//std::queue<Event> eventQueue;
	std::deque<Event> eventQueue;
	
	mutable std::mutex mutex;
	std::unique_lock<std::mutex> _lockPushes;
	bool _arePushesLocked;
	std::condition_variable nonemptyCond;
	
	//struct timespec lastEventHandledTime; //used? Only written - never read!
	unsigned bufferSize;
	static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
	static std::atomic<bool> isExiting; //typical implementations of exit() call the exit handlers from within the thread that called exit. Therefore, if the exiting thread causes another thread to call exit(), this value must be atomic.
	private:
		static void callExitHandlers();
		void orderedInsert(const Event &evt);
	public:
		static void configureExitHandlers();
		static void registerExitHandler(void (*handler)(), unsigned level);
		//queue and nextEvent can be called from separate threads, but nextEvent must NEVER be called from multiple threads.
		void queue(const Event &evt);
		void schedPwm(AxisIdType idx, const PwmInfo &p);
		inline void schedPwm(AxisIdType idx, float duty) {
			PwmInfo pi(duty, pwmInfo[idx].period());
			schedPwm(idx, pi);
		}
		Scheduler();
		Event nextEvent(bool doSleep=true, std::chrono::microseconds timeout=std::chrono::microseconds(1000000));
		void sleepUntilEvent(const Event &evt) const;
		void initSchedThread() const; //call this from whatever threads call nextEvent to optimize that thread's priority.
		struct timespec lastSchedTime() const; //get the time at which the last event is scheduled, or the current time if no events queued.
		void setBufferSize(unsigned size);
		unsigned getBufferSize() const;
		template <typename T> void eventLoop(T* callbackObj, void(T::*onEvent)(const Event &e), bool(T::*onWait)()) {
			Event evt;
			while (1) {
				bool needCpuTime = (callbackObj->*onWait)();
				evt = this->nextEvent(false, std::chrono::microseconds(needCpuTime ? 0 : 100000)); //get next event, but don't sleep. Yield to the OS for ~100ms if we DON'T need the cpu time.
				if (!evt.isNull()) { //wait for event to occur if it is non-null.
					while (!evt.isTime()) {
						if (!(callbackObj->*onWait)()) { //if we don't need any future waiting, then sleep to give cpu to other processes:
							this->sleepUntilEvent(evt);
						}
					}
					(callbackObj->*onEvent)(evt);
					needCpuTime = true;
				}
			}
		}
};


#endif
