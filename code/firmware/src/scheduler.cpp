#include "scheduler.h"

//#include <stdio.h>
#include "logging.h"


void Scheduler::queue(const Event& evt) {
	LOGV("Scheduler::queue\n");
	this->eventQueue.push(evt);
}

