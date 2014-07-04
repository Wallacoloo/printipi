#include "scheduler.h"

#include <stdio.h>


void Scheduler::queue(const Event& evt) {
	printf("Scheduler::queue\n");
	this->eventQueue.push(evt);
}

