#include "scheduler.h"

namespace gparse {

void Scheduler::queue(const Event& evt) {
	this->eventQueue.push(evt);
}

}
