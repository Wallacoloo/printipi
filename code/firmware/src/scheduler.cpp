#include "scheduler.h"

#include <pthread.h> //for pthread_setschedparam
#include <time.h> //for clock_nanosleep
#include "logging.h"


/*Scheduler::Scheduler(const std::function<void(const Event&)>& callback) : consumer(std::thread(&Scheduler::consumerLoop, this, callback)) {}*/

//Scheduler::Scheduler() : consumer(std::thread(&Scheduler::consumerLoop, this)) {}
Scheduler::Scheduler() : _lockPushes(mutex, std::defer_lock), _arePushesLocked(false) {}


void Scheduler::queue(const Event& evt) {
	LOGV("Scheduler::queue\n");
	std::unique_lock<std::mutex> lock(this->mutex);
	this->eventQueue.push(evt);
	this->nonemptyCond.notify_one(); //notify the consumer thread that a new event is ready.
}


Event Scheduler::nextEvent() {
	Event evt;
	{
		//std::unique_lock<std::mutex> lock(this->mutex);
		if (!this->_arePushesLocked) { //check if the mutex is already locked *by us*
			_lockPushes.lock();
		}
		while (this->eventQueue.empty()) { //wait for an event to be pushed.
			this->nonemptyCond.wait(_lockPushes); //condition_variable.wait() can produce spurious wakeups; need the while loop.
		}
		evt = this->eventQueue.front();
		this->eventQueue.pop();
		if (this->eventQueue.size() < SCHED_CAPACITY) { //queue is underfilled; release the lock
			_lockPushes.unlock();
			this->_arePushesLocked = false;
		} else { //queue is filled; do not release the lock.
			this->_arePushesLocked = true;
		}
	} //unlock the mutex and then handle the event.
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(evt.time()), NULL); //sleep to event time.
	return evt;
}

void Scheduler::initSchedThread() {
	struct sched_param sp; 
	sp.sched_priority=SCHED_PRIORITY; 
	if (int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
		LOGW("Warning: pthread_setschedparam (increase thread priority) at scheduler.cpp returned non-zero: %i\n", ret);
	}
}

struct timespec Scheduler::lastSchedTime() const {
	Event evt;
	{
		std::unique_lock<std::mutex> lock(this->mutex);
		if (this->eventQueue.size()) {
			evt = this->eventQueue.back();
		} else {
			lock.unlock();
			timespec ts;
			clock_gettime(CLOCK_MONOTONIC, &ts);
			return ts;
		}
	}
	return evt.time();
}

