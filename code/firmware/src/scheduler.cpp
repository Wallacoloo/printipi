#include "scheduler.h"

//#include <stdio.h>
#include "logging.h"


void Scheduler::queue(const Event& evt) {
	LOG("Scheduler::queue\n");
	this->eventQueue.push(evt);
}

