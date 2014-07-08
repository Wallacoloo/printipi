#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "event.h"


class Scheduler {
	std::queue<Event> eventQueue;
	std::thread consumer;
	std::mutex mutex;
	std::condition_variable nonemptyCond;
	public:
		void queue(const Event& evt);
		Scheduler();
	private:
		void consumerLoop();

};


#endif
