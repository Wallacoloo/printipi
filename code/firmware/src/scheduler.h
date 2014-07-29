#ifndef SCHEDULER_H
#define SCHEDULER_H

//#include <queue>
//#include <deque>
#include <set>
#include <thread> //for this_thread.sleep
//#include <mutex>
//#include <condition_variable>
#include <time.h> //for timespec
#include <chrono> 
#include <array>
#include <vector>
//#include <atomic>
#include <tuple>
//#include <algorithm> //for push_heap
//#include <functional>
#include "event.h"
#include "logging.h"
#include "timeutil.h"

#include <pthread.h> //for pthread_setschedparam
//#include <time.h> //for clock_nanosleep
//#include "logging.h"

#ifndef SCHED_PRIORITY
	#define SCHED_PRIORITY 30
#endif
#ifndef SCHED_CAPACITY
	#define SCHED_CAPACITY 256
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
		nsHigh(std::min(999999999, std::max(0, (int)(duty*period*1000000000)))), //clamp the times to >= 0 and <= 1
		nsLow(std::min(999999999, std::max(0, (int)((1-duty)*period*1000000000)))) {}
	float period() const {
		return nsHigh + nsLow;
	}
	bool isNonNull() const {
		return nsHigh || nsLow;
	}
};

class SchedulerBase {
	static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
	//static std::atomic<bool> isExiting; //typical implementations of exit() call the exit handlers from within the thread that called exit. Therefore, if the exiting thread causes another thread to call exit(), this value must be atomic.
	static bool isExiting;
	private:
		static void callExitHandlers();
	public:
		static void configureExitHandlers();
		static void registerExitHandler(void (*handler)(), unsigned level);
};

//When queueing an event, one may hint that it is either near or far away, and this may give a performance boost to the container.
enum InsertHint {
	INSERT_FRONT,
	INSERT_BACK
};

template <typename Interface> class Scheduler : public SchedulerBase {
	Interface interface;
	//std::array<PwmInfo, 256> pwmInfo; 
	std::array<PwmInfo, Interface::numIoDrivers()> pwmInfo; 
	//std::queue<Event> eventQueue;
	//std::deque<Event> eventQueue; //queue is ordered such that the soonest event is the front and the latest event is the back
	std::multiset<Event> eventQueue; //mutimap is ordered such that begin() is smallest, rbegin() is largest
	//mutable std::mutex mutex;
	//std::unique_lock<std::mutex> _lockPushes;
	//bool _arePushesLocked;
	//std::condition_variable nonemptyCond;
	//std::condition_variable eventConsumedCond;
	
	unsigned bufferSize;
	private:
		void orderedInsert(const Event &evt, InsertHint insertBack=INSERT_BACK);
	public:
		void queue(const Event &evt);
		void schedPwm(AxisIdType idx, const PwmInfo &p);
		inline void schedPwm(AxisIdType idx, float duty) {
			PwmInfo pi(duty, pwmInfo[idx].period());
			schedPwm(idx, pi);
		}
		Scheduler(Interface interface);
		//Event nextEvent(bool doSleep=true, std::chrono::microseconds timeout=std::chrono::microseconds(1000000));
		bool isEventNear(const Event &evt) const;
		void sleepUntilEvent(const Event &evt) const;
		void initSchedThread() const; //call this from whatever threads call nextEvent to optimize that thread's priority.
		struct timespec lastSchedTime() const; //get the time at which the last event is scheduled, or the current time if no events queued.
		void setBufferSize(unsigned size);
		unsigned getBufferSize() const;
		unsigned numActivePwmChannels() const;
		void eventLoop();
		void yield(bool forceWait=false);
};



template <typename Interface> Scheduler<Interface>::Scheduler(Interface interface) 
	: interface(interface),bufferSize(SCHED_CAPACITY) {
	//clock_gettime(CLOCK_MONOTONIC, &(this->lastEventHandledTime)); //initialize to current time.
}


template <typename Interface> void Scheduler<Interface>::queue(const Event& evt) {
	//LOGV("Scheduler::queue\n");
	while (this->eventQueue.size() >= this->bufferSize) {
		//yield();
		yield(true);
	}
	this->orderedInsert(evt, INSERT_BACK);
	yield(); //fast yield. Not really necessary if the earlier yield statement was ever reached.
}

template <typename Interface> void Scheduler<Interface>::orderedInsert(const Event &evt, InsertHint insertHint) {
	//Most inserts will already be ordered (ie the event will occur after all scheduled events)
	//glibc push_heap will be logarithmic no matter WHAT: https://gcc.gnu.org/onlinedocs/gcc-4.6.3/libstdc++/api/a01051_source.html
	//it may be beneficial to compare against the previously last element.
	//if buffer size is 512, then that gives 1 compare instead of 9.
	//on the other hand, if the buffer is that big, insertion time probably isn't crucial.
	/*if (this->eventQueue.empty()) {
		this->eventQueue.push_back(evt);
	} else { //fetching eventQueue.back() is only valid if the queue is non-empty.
		const Event &oldBack = this->eventQueue.back();
		this->eventQueue.push_back(evt);
		if (timespecLt(evt.time(), oldBack.time())) { //If not already ordered, we must order it.
			std::push_heap(this->eventQueue.begin(), this->eventQueue.end());
		}
	}*/
	//NOTE: heap is not a sorted collection.
	/*this->eventQueue.push_back(evt);
	std::push_heap(this->eventQueue.begin(), this->eventQueue.end(), std::greater<Event>());
	LOGV("orderedInsert: front().time(), back().time(): %lu.%u, %lu.%u. %i\n", eventQueue.front().time().tv_sec, eventQueue.front().time().tv_nsec, eventQueue.back().time().tv_sec, eventQueue.back().time().tv_nsec, std::is_heap(eventQueue.begin(), eventQueue.end(), std::greater<Event>()));*/
	eventQueue.insert(insertHint == INSERT_BACK ? eventQueue.end() : eventQueue.begin(), evt); 
}

template <typename Interface> void Scheduler<Interface>::schedPwm(AxisIdType idx, const PwmInfo &p) {
	LOGV("Scheduler::schedPwm: %i, %u, %u. Current: %u, %u\n", idx, p.nsHigh, p.nsLow, pwmInfo[idx].nsHigh, pwmInfo[idx].nsLow);
	if (pwmInfo[idx].isNonNull()) { //already scheduled and running. Just update times.
		pwmInfo[idx] = p; //note: purposely redundant with below; must check isNonNull() before modifying the pwmInfo.
	} else { //have to schedule:
		LOGV("Scheduler::schedPwm: queueing\n");
		pwmInfo[idx] = p;
		Event evt(timespecNow(), idx, p.nsHigh ? StepForward : StepBackward); //if we have any high-time, then start with forward, else backward.
		this->queue(evt);
	}
}


template <typename Interface> bool Scheduler<Interface>::isEventNear(const Event &evt) const {
	timespec thresh = timespecAdd(timespecNow(), timespec{0, 20000}); //20000 = 20 uSec
	return timespecLt(evt.time(), thresh);
}

template <typename Interface> void Scheduler<Interface>::sleepUntilEvent(const Event &evt) const {
	struct timespec sleepUntil = evt.time();
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sleepUntil, NULL); //sleep to event time.
}

template <typename Interface> void Scheduler<Interface>::initSchedThread() const {
	struct sched_param sp; 
	sp.sched_priority=SCHED_PRIORITY; 
	if (int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
		LOGW("Warning: pthread_setschedparam (increase thread priority) at scheduler.cpp returned non-zero: %i\n", ret);
	}
}

template <typename Interface> struct timespec Scheduler<Interface>::lastSchedTime() const {
	if (this->eventQueue.empty()) {
		return timespecNow();
	} else {
		return this->eventQueue.rbegin()->time();
	}
}

template <typename Interface> void Scheduler<Interface>::setBufferSize(unsigned size) {
	this->bufferSize = size;
	LOG("Scheduler buffer size set: %u\n", size);
}
template <typename Interface> unsigned Scheduler<Interface>::getBufferSize() const {
	return this->bufferSize;
}

template <typename Interface> unsigned Scheduler<Interface>::numActivePwmChannels() const {
	unsigned r=0;
	for (const PwmInfo &p : this->pwmInfo) {
		if (p.isNonNull()) {
			r += 1;
		}
	}
	return r;
}

template <typename Interface> void Scheduler<Interface>::eventLoop() {
	while (1) {
		yield(true);
		if (eventQueue.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(40));
		}
	}
}

template <typename Interface> void Scheduler<Interface>::yield(bool forceWait) {
	while (1) {
		//LOGV("Scheduler::eventQueue.size(): %zu\n", eventQueue.size());
		//LOGV("front().time(), back().time(): %lu.%u, %lu.%u\n", eventQueue.front().time().tv_sec, eventQueue.front().time().tv_nsec, eventQueue.back().time().tv_sec, eventQueue.back().time().tv_nsec);
		if (eventQueue.empty()) { //if no events, then run idle events and return.
			if (!interface.onIdleCpu()) { //loop is implied by the outer while(1)
				return;
			}
			//avoid the following, in case onIdleCpu causes the generation of events:
			//do {} while (interface.onIdleCpu());
		} else {
			//interface.onIdleCpu();
			//const Event &evt = this->eventQueue.front();
			//Event evt = this->eventQueue.front();
			Event evt = *this->eventQueue.begin();
			//do NOT pop the event here, because it might not be handled this time around.
			while (!evt.isTime()) {
				if (!interface.onIdleCpu()) { //if we don't need any onIdleCpu, then either sleep for event or yield to rest of program:
					if (!isEventNear(evt) && !forceWait) { //if the event is far away, then return control to program.
						return;
					} else { //retain control if the event is near, or if the queue must be emptied.
						this->sleepUntilEvent(evt);
						break;
					}
				}
			}
			this->eventQueue.erase(eventQueue.begin());
			//this->eventQueue.pop_front();
			interface.onEvent(evt);
			
			//manage PWM events:
			const PwmInfo &pwm = pwmInfo[evt.stepperId()];
			if (pwm.isNonNull()) {
				Event nextPwm;
				/*         for | back
				 * nsLow    0     1
				 * nsHigh   1     0   */
				//dir = (nsLow ^ for)
				if (evt.direction() == StepForward) {
					//next event will be StepBackward, or refresh this event if there is no off-duty.
					nextPwm = Event(evt.time(), evt.stepperId(), pwm.nsLow ? StepBackward : StepForward);
					nextPwm.offsetNano(pwm.nsHigh);
				} else {
					//next event will be StepForward, or refresh this event if there is no on-duty.
					nextPwm = Event(evt.time(), evt.stepperId(), pwm.nsHigh ? StepForward : StepBackward);
					nextPwm.offsetNano(pwm.nsLow);
				}
				this->orderedInsert(nextPwm, INSERT_FRONT);
			}
			//this->eventQueue.pop_front(); //this is OK to put after PWM generation, because the next PWM event will ALWAYS occur after the current pwm event, so the queue front won't change. Furthermore, if interface.onEvent(evt) generates a new event (which it shouldn't), it most probably won't be scheduled for the past.
			forceWait = false; //avoid draining ALL events - just drain the first.
		}
	}
}

#endif
