#ifndef SCHEDULER_H
#define SCHEDULER_H

/* 
 * Printipi/scheduler.h
 * (c) 2014 Colin Wallace
 *
 * The Scheduler controls program flow between tending communications and executing events at precise times.
 * It also allows for software PWM of any output.
 * It is designed to run in a single-threaded environment so it can have maximum control.
 * As such, the program should call Scheduler.yield() periodically if doing any long-running task.
 * Events can be queued with Scheduler.queue, and Scheduler.eventLoop should be called after any program setup is completed.
 */


#include <cassert> //for assert
#include <set>
//#include <thread> //for this_thread.sleep
//#include <time.h> //for timespec
//#include <chrono> 
#include <array>
#include <vector>
//#include <atomic>
#include <tuple>
#include "event.h"
#include "common/logging.h"
#include "common/timeutil.h"
#include "intervaltimer.h"

#include <pthread.h> //for pthread_setschedparam

#ifndef SCHED_PRIORITY
	#define SCHED_PRIORITY 30
#endif
//really this is the default scheduler buffer size:
#ifndef SCHED_CAPACITY
	#define SCHED_CAPACITY 128
#endif
#ifndef SCHED_NUM_EXIT_HANDLER_LEVELS
	#define SCHED_NUM_EXIT_HANDLER_LEVELS 2
#endif
#define SCHED_IO_EXIT_LEVEL 0
#define SCHED_MEM_EXIT_LEVEL 1

//When queueing an event, one may hint that it is either near or far away, and this may give a performance boost to the container.
enum InsertHint {
	INSERT_FRONT,
	INSERT_BACK
};

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

/* Base class from which all templated schedulers derive.
Defines things such as exit handlers */
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

/* A Scheduler is useless without an interface. In order to use a scheduler, implement a class with the following methods
and pass it as a template argument to the Scheduler<Interface> type. */
struct DefaultSchedulerInterface {
	void onEvent(const Event& /*evt*/) { }
	bool onIdleCpu() {
		return false; //no more cpu needed
	}
	static constexpr std::size_t numIoDrivers() {
		return 0; //no IoDrivers;
	}
};

template <typename Interface=DefaultSchedulerInterface> class Scheduler : public SchedulerBase {
	struct SchedAdjuster {
		/* The logic is a bit odd here, but the idea is to compensate for missed events.
		  If the scheduler isn't serviced on time, we don't want 10 backed-up events all happening at the same time. Instead, we offset them and pick them up then. We can never execute events with intervals smaller than they would register - this would indicate real movement of, say, 70mm/sec when the user only asked for 60mm/sec. Thus the scheduler can never be made "on track" again, unless there is a gap in scheduled events.
		  If, because of the stall, actual velocity was decreased to 10mm/sec, we cannot jump instantly back to 60mm/sec (this would certainly cause MORE missed steps)! Instead, we accelerate back up to it.
		  The tricky bit is - how do we estimate what the actual velocity is? We don't want to overcompensate. Unfortunately for now, some of the logic might :P */
		//static constexpr float a = -5.0;
		static constexpr float a = -12.0;
		IntervalTimer lastRealTime;
		EventClockT::time_point lastSchedTime;
		float lastSlope;
		SchedAdjuster() : lastSchedTime(), lastSlope(1) {}
		/* Reset() should destroy any offset, so that adjust(time.now()) gives time.now() */
		void reset() {
			lastRealTime.reset();
			lastSchedTime = EventClockT::time_point();
			lastSlope = 1;
		}
		EventClockT::time_point adjust(EventClockT::time_point tp) const {
			//SHOULD work precisely with x0, y0 = (0, 0)
			float s_s0 = std::chrono::duration_cast<std::chrono::duration<float> >(tp - lastSchedTime).count();
			float offset;
			if (s_s0 < (1.-lastSlope)/2./a) { //acclerating:
				offset = a*s_s0*s_s0 + lastSlope*s_s0;
			} else { //stabilized:
				offset = (1.-lastSlope)*(1.-lastSlope)/-4/a + s_s0;
			}
			EventClockT::time_point ret(lastRealTime.get() + std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(offset)));
			if (ret < tp) {
				LOGV("SchedAdjuster::adjust adjusted into the past!\n");
			}
			return ret;
		}
		//call this when the event scheduled at time t is actually run.
		void update(EventClockT::time_point tp) {
			//SHOULD work reasonably with x0, y0 = (0, 0)
			auto y0 = lastRealTime.get();
			if (EventClockT::now()-y0 > std::chrono::milliseconds(50)) {
				//only sample every few ms, to mitigate Events scheduled on top of eachother.
				auto y1 = lastRealTime.clock();
				//the +X.XXX is to prevent a division-by-zero, and to minimize the effect that small sched errors have on the timeline:
				auto avgSlope = (std::chrono::duration_cast<std::chrono::duration<float> >(y1 - y0).count()+0.030) / (0.030+std::chrono::duration_cast<std::chrono::duration<float> >(tp-lastSchedTime).count());
				lastSlope = std::min(RUNNING_IN_VM ? 1. : 20., 2.*avgSlope- lastSlope); //set a minimum for the speed that can be run at.
				lastSchedTime = tp;
			}
		}
	};
	static const EventClockT::duration MAX_SLEEP; //need to call onIdleCpu handlers every so often, even if no events are ready.
	typedef std::multiset<Event> EventQueueType;
	Interface interface;
	std::array<PwmInfo, Interface::numIoDrivers()> pwmInfo; 
	//std::deque<Event> eventQueue; //queue is ordered such that the soonest event is the front and the latest event is the back
	EventQueueType eventQueue; //mutimap is ordered such that begin() is smallest, rbegin() is largest
	SchedAdjuster schedAdjuster;
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
		void initSchedThread() const; //call this from whatever threads call nextEvent to optimize that thread's priority.
		EventClockT::time_point lastSchedTime() const; //get the time at which the last event is scheduled, or the current time if no events queued.
		void setBufferSize(unsigned size);
		void setBufferSizeToDefault();
		unsigned getBufferSize() const;
		bool isRoomInBuffer() const;
		unsigned numActivePwmChannels() const;
		void eventLoop();
		void yield(bool forceWait=false);
	private:
		void sleepUntilEvent(const Event *evt) const;
		bool isEventNear(const Event &evt) const;
		bool isEventTime(const Event &evt) const;
};

template <typename Interface> const EventClockT::duration Scheduler<Interface>::MAX_SLEEP(std::chrono::duration_cast<EventClockT::duration>(std::chrono::milliseconds(40)));


template <typename Interface> Scheduler<Interface>::Scheduler(Interface interface) 
	: interface(interface),bufferSize(SCHED_CAPACITY) {
	//clock_gettime(CLOCK_MONOTONIC, &(this->lastEventHandledTime)); //initialize to current time.
}


template <typename Interface> void Scheduler<Interface>::queue(const Event& evt) {
	//LOGV("Scheduler::queue\n");
	//while (this->eventQueue.size() >= this->bufferSize) {
	/*while (!isRoomInBuffer()) {
		//yield();
		yield(true);
	}*/
	assert(isRoomInBuffer());
	this->orderedInsert(evt, INSERT_BACK);
	//yield(); //fast yield. Not really necessary if the earlier yield statement was ever reached.
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
		Event evt(lastSchedTime(), idx, p.nsHigh ? StepForward : StepBackward); //if we have any high-time, then start with forward, else backward.
		setBufferSize(getBufferSize()+1); //Make some room for this event.
		this->queue(evt);
	}
}

template <typename Interface> void Scheduler<Interface>::initSchedThread() const {
	struct sched_param sp; 
	sp.sched_priority=SCHED_PRIORITY; 
	if (int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
		LOGW("Warning: pthread_setschedparam (increase thread priority) at scheduler.cpp returned non-zero: %i\n", ret);
	}
}

template <typename Interface> EventClockT::time_point Scheduler<Interface>::lastSchedTime() const {
	//TODO: Note, this method, as-is, is const!
	if (this->eventQueue.empty()) {
		const_cast<Scheduler<Interface>*>(this)->schedAdjuster.reset(); //we have no events; no need to preserve *their* reference times, so reset for simplicity.
		return EventClockT::now(); //an alternative is to not use ::now(), but instead a time set in the past that is scheduled to happen now. Minimum intervals are conserved, so there's that would actually work decently. In actuality, the eventQueue will NEVER be empty except at initialization, because it handles pwm too.
	} else {
		return this->eventQueue.rbegin()->time();
	}
}

template <typename Interface> void Scheduler<Interface>::setBufferSize(unsigned size) {
	if (size != this->bufferSize) {
		LOG("Scheduler buffer size set: %u\n", size);
	}
	this->bufferSize = size;
}
template <typename Interface> void Scheduler<Interface>::setBufferSizeToDefault() {
	setBufferSize(SCHED_CAPACITY);
}
template <typename Interface> unsigned Scheduler<Interface>::getBufferSize() const {
	return this->bufferSize;
}
template <typename Interface> bool Scheduler<Interface>::isRoomInBuffer() const {
	return this->eventQueue.size() < this->bufferSize;
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
			sleepUntilEvent(NULL);
			//std::this_thread::sleep_for(std::chrono::milliseconds(40));
		}
	}
}

template <typename Interface> void Scheduler<Interface>::yield(bool forceWait) {
	while (1) {
		//LOGV("Scheduler::eventQueue.size(): %zu\n", eventQueue.size());
		//LOGV("front().time(), back().time(): %lu.%u, %lu.%u\n", eventQueue.front().time().tv_sec, eventQueue.front().time().tv_nsec, eventQueue.back().time().tv_sec, eventQueue.back().time().tv_nsec);
		//interface.onIdleCpu();
		//const Event &evt = this->eventQueue.front();
		//Event evt = this->eventQueue.front();
		//EventQueueType::const_iterator iter = this->eventQueue.cbegin();
		//Event evt = *iter;
		//Event evt = *this->eventQueue.begin();
		//do NOT pop the event here, because it might not be handled this time around.
		//it's possible for onIdleCpu to call Scheduler.yield(), in which case another instantiation of this call could have already handled the event we're looking at. Therefore we need to be checking the most up-to-date event each time around.
		while (!eventQueue.empty() && !isEventTime(*eventQueue.cbegin())) {
			if (!interface.onIdleCpu()) { //if we don't need any onIdleCpu, then either sleep for event or yield to rest of program:
				EventQueueType::const_iterator iter = this->eventQueue.cbegin();
				if (!isEventNear(*iter) && !forceWait) { //if the event is far away, then return control to program.
					return;
				} else { //retain control if the event is near, or if the queue must be emptied.
					this->sleepUntilEvent(&*iter);
					//break;
				}
			}
		}
		//in the case that all events were consumed, or there were none to begin with, satisfy the idle cpu functions:
		if (eventQueue.empty()) {
			if (interface.onIdleCpu()) { //loop is implied by the outer while(1)
				continue;
			} else {
				return;
			}
		}
		EventQueueType::const_iterator iter = this->eventQueue.cbegin();
		Event evt = *iter;
		auto mapped = schedAdjuster.adjust(evt.time());
		auto now = EventClockT::now();
		LOGV("Scheduler executing event. original->mapped time, now, buffer: %lld -> %lld, %lld. sz: %zu\n", evt.time().time_since_epoch().count(), mapped.time_since_epoch().count(), now.time_since_epoch().count(), eventQueue.size());
		//this->eventQueue.erase(eventQueue.begin());
		this->eventQueue.erase(iter); //iterator unaffected even if other events were inserted OR erased.
		//The error: eventQueue got flooded with stepper #5 PWM events.
		//  They somehow got duplicated, likely by a failure to erase the *correct* previous pwm event.
		//  this should be fixed by saving the iter and erasing it.
		schedAdjuster.update(evt.time());
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

template <typename Interface> void Scheduler<Interface>::sleepUntilEvent(const Event *evt) const {
	//need to call onIdleCpu handlers occasionally - avoid sleeping for long periods of time.
	auto sleepUntil = EventClockT::now() + MAX_SLEEP;
	if (evt) { //allow calling with NULL to sleep for a configured period of time (MAX_SLEEP)
		auto evtTime = schedAdjuster.adjust(evt->time());
		sleepUntil = std::min(sleepUntil, evtTime);
	}
	//LOGV("Scheduler::sleepUntilEvent: %ld.%08lu\n", sleepUntil.tv_sec, sleepUntil.tv_nsec);
	timespec tsSleepUntil = timepointToTimespec(sleepUntil);
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tsSleepUntil, NULL); //sleep to event time.
}

template <typename Interface> bool Scheduler<Interface>::isEventNear(const Event &evt) const {
	auto thresh = EventClockT::now() + std::chrono::microseconds(20);
	return schedAdjuster.adjust(evt.time()) <= thresh;
}

template <typename Interface> bool Scheduler<Interface>::isEventTime(const Event &evt) const {
	return schedAdjuster.adjust(evt.time()) <= EventClockT::now();
}

#endif
