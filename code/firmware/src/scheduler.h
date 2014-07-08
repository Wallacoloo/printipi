#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <queue>
#include <thread>
#include "event.h"


class Scheduler {
	std::queue<Event> eventQueue;
	std::thread consumer;
	public:
		void queue(const Event& evt);
		Scheduler();
	private:
		void consumerLoop();

};


#endif
