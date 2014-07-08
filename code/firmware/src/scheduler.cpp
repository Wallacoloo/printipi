#include "scheduler.h"

//#include <stdio.h>
#include "logging.h"


Scheduler::Scheduler() : consumer(std::thread(&Scheduler::consumerLoop, this)) {
}

void Scheduler::queue(const Event& evt) {
	LOGV("Scheduler::queue\n");
	this->eventQueue.push(evt);
}

void Scheduler::consumerLoop() {
	LOGD("Scheduler::consumerLoop begin");
}
