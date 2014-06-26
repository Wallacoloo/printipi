#ifndef GPARSE_SCHEDULER_H
#define GPARSE_SCHEDULER_H

#include <queue>
#include "event.h"

namespace gparse {

class Scheduler {
	std::queue<Event> eventQueue;
	public:
		void queue(const Event& evt);

};

}

#endif
