#include "scheduler.h"

#include <pthread.h> //for pthread_setschedparam
#include <time.h> //for clock_nanosleep
#include "logging.h"


/*Scheduler::Scheduler(const std::function<void(const Event&)>& callback) : consumer(std::thread(&Scheduler::consumerLoop, this, callback)) {}*/

//Scheduler::Scheduler() : consumer(std::thread(&Scheduler::consumerLoop, this)) {}
//Scheduler::Scheduler() {}


void Scheduler::queue(const Event& evt) {
	LOGV("Scheduler::queue\n");
	std::unique_lock<std::mutex> lock(this->mutex);
	this->eventQueue.push(evt);
	this->nonemptyCond.notify_one(); //notify the consumer thread that a new event is ready.
}


Event Scheduler::nextEvent() {
	Event evt;
	{
		std::unique_lock<std::mutex> lock(this->mutex);
		while (this->eventQueue.empty()) { //wait for an event to be pushed.
			this->nonemptyCond.wait(lock); //condition_variable.wait() can produce spurious wakeups; need the while loop.
		}
		evt = this->eventQueue.front();
		this->eventQueue.pop();
	} //unlock the mutex and then handle the event.
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(evt.time()), NULL); //sleep to event time.
	return evt;
}

