#include "scheduler.h"

#include <stdio.h>

namespace gparse {

void Scheduler::queue(const Event& evt) {
	printf("Scheduler::queue\n");
	this->eventQueue.push(evt);
}

}
