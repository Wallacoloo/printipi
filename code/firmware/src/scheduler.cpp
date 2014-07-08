#include "scheduler.h"

//#include <stdio.h>
#include "logging.h"


Scheduler::Scheduler() : consumer(std::thread(&Scheduler::consumerLoop, this)) {
}

void Scheduler::queue(const Event& evt) {
	LOGV("Scheduler::queue\n");
	std::unique_lock<std::mutex> lock(this->mutex);
	this->eventQueue.push(evt);
	this->nonemptyCond.notify_one(); //notify the consumer thread that a new event is ready.
}

void Scheduler::consumerLoop() {
	LOGD("Scheduler::consumerLoop begin");
	Event evt;
	while (1) {
		{
			std::unique_lock<std::mutex> lock(this->mutex);
			while (this->eventQueue.empty()) { //wait for an event to be pushed.
				this->nonemptyCond.wait(lock); //condition_variable.wait() can produce spurious wakeups; need the while loop.
			}
			evt = this->eventQueue.front();
			this->eventQueue.pop();
		} //unlock the mutex and then handle the event.
		
	}
}
