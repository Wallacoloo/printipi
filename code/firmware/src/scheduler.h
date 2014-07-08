#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include "event.h"

#include <pthread.h> //for pthread_setschedparam
#include <time.h> //for clock_nanosleep
#include "logging.h"

#ifndef SCHED_PRIORITY
#define SCHED_PRIORITY 30
#endif


class Scheduler {
	std::queue<Event> eventQueue;
	std::thread consumer;
	std::mutex mutex;
	std::condition_variable nonemptyCond;
	public:
		void queue(const Event& evt);
		//Scheduler(const std::function<void(const Event&)>& callback);
		Scheduler();
	private:
		void consumerLoop();
		//void consumerLoop(const std::function<void(const Event&)>& callback);
	public:
		//template <typename T> Scheduler(T& callback) : consumer(std::thread(&Scheduler::consumerLoop<T>, this, callback)) {}
		/*template <typename T> Scheduler(T& callback) : consumer(std::thread([&]{ consumerLoop(callback); })) {}
		template <typename T> void consumerLoop(T& callback) {
			LOGD("Scheduler::consumerLoop begin\n");
	
			struct sched_param sp; //set high priority for the scheduling thread.
			sp.sched_priority=SCHED_PRIORITY; 
			int retval = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
			if (retval) {
				LOGW("Warning: pthread_setschedparam (set thread to high priority) in Scheduler::consumerLoop returned non-zero: %i\n", retval);
			}
			LOGD("Scheduler::consumerLoop priority set to %i\n", SCHED_PRIORITY);
		
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
				clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(evt.time()), NULL); //sleep to event time.
				callback.handleEvent(evt);
				//callback(evt); //process the event.
			}
		}*/
		
};


#endif
