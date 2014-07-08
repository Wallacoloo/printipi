#include "scheduler.h"

#include <pthread.h> //for pthread_setschedparam
#include <time.h> //for clock_nanosleep
#include "logging.h"


/*Scheduler::Scheduler(const std::function<void(const Event&)>& callback) : consumer(std::thread(&Scheduler::consumerLoop, this, callback)) {}*/

//Scheduler::Scheduler() : consumer(std::thread(&Scheduler::consumerLoop, this)) {}
Scheduler::Scheduler() : _isPushLocked(false), _lockPushes(mutex, std::defer_lock) {}


void Scheduler::queue(const Event& evt) {
	LOGV("Scheduler::queue\n");
	/*do {
		std::unique_lock<std::mutex> lock(this->mutex);
		if (this->eventQueue.size() < SCHED_CAPACITY) {
			this->eventQueue.push(evt);
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(SCHED_PUSH_SPIN_INTERVAL));
	} while(1);*/
	//std::unique_lock<std::mutex> requestAccess(this->allowPushMutex);
	std::unique_lock<std::mutex> lock(this->mutex);
	this->eventQueue.push(evt);
	this->nonemptyCond.notify_one(); //notify the consumer thread that a new event is ready.
}


Event Scheduler::nextEvent() {
	Event evt;
	{
		//std::unique_lock<std::mutex> lock(this->mutex);
		if (!this->_isPushLocked) {
			_lockPushes.lock();
		}
		while (this->eventQueue.empty()) { //wait for an event to be pushed.
			this->nonemptyCond.wait(_lockPushes); //condition_variable.wait() can produce spurious wakeups; need the while loop.
		}
		evt = this->eventQueue.front();
		this->eventQueue.pop();
		/*if (this->eventQueue.size() >= SCHED_CAPACITY && !this->_isPushLocked) {
			this->allowPushMutex.lock();
			this->_isPushLocked = true;
		} else if (this->eventQueue.size() == SCHED_CAPACITY-1 && this->_isPushLocked){
			this->allowPushMutex.unlock();
			this->_isPushLocked = false;
		}*/
		if (this->eventQueue.size() < SCHED_CAPACITY) {
			_lockPushes.unlock();
			this->_isPushLocked = false;
		} else {
			this->_isPushLocked = true;
		}
	} //unlock the mutex and then handle the event.
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(evt.time()), NULL); //sleep to event time.
	return evt;
}

