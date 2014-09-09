#ifndef SCHEDULERBASE_H
#define SCHEDULERBASE_H

#include <algorithm> //for std::min
#include <array>
#include <vector>

#ifndef SCHED_PRIORITY
	#define SCHED_PRIORITY 30
#endif
//really this is the default scheduler buffer size:
#ifndef SCHED_CAPACITY
	#define SCHED_CAPACITY 128
#endif
#ifndef SCHED_NUM_EXIT_HANDLER_LEVELS
	#define SCHED_NUM_EXIT_HANDLER_LEVELS 2
#endif
#define SCHED_IO_EXIT_LEVEL 0
#define SCHED_MEM_EXIT_LEVEL 1

class Event; //forward declaration to avoid inclusion of event.h (as event.h includes typesettings.h, which may include this file)

//When queueing an event, one may hint that it is either near or far away, and this may give a performance boost to the container.
enum InsertHint {
	INSERT_FRONT,
	INSERT_BACK
};

//Scheduler::Interface::onIdleCpu can be called with a flag indicating (roughly) how long it's been since it was last called.
enum OnIdleCpuIntervalT {
	OnIdleCpuIntervalShort,
	OnIdleCpuIntervalWide
};

struct PwmInfo {
	unsigned nsHigh;
	unsigned nsLow;
	PwmInfo() : nsHigh(0), nsLow(0) {}
	PwmInfo(float duty, float period) : 
		nsHigh(std::min(999999999, std::max(0, (int)(duty*period*1000000000)))), //clamp the times to >= 0 and <= 1
		nsLow(std::min(999999999, std::max(0, (int)((1-duty)*period*1000000000)))) {}
	inline float period() const {
		return nsHigh + nsLow;
	}
	inline bool isNonNull() const {
		return nsHigh || nsLow;
	}
};

/* Base class from which all templated schedulers derive.
Defines things such as exit handlers */
class SchedulerBase {
	static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
	//static std::atomic<bool> isExiting; //typical implementations of exit() call the exit handlers from within the thread that called exit. Therefore, if the exiting thread causes another thread to call exit(), this value must be atomic.
	static bool isExiting;
	private:
		static void callExitHandlers();
	public:
		static void configureExitHandlers();
		static void registerExitHandler(void (*handler)(), unsigned level);
};

/* A Scheduler is useless without an interface. In order to use a scheduler, implement a class with the following methods
and pass it as a template argument to the Scheduler<Interface> type. */
struct DefaultSchedulerInterface {
	void onEvent(const Event& /*evt*/) { }
	inline bool onIdleCpu() {
		return false; //no more cpu needed
	}
	inline static constexpr std::size_t numIoDrivers() {
		return 0; //no IoDrivers;
	}
	struct HardwareScheduler {
	    inline bool canWriteOutputs() const {
	        //No, this default interface is not capable of writing output pins
	        return false;
	    }
	};
	HardwareScheduler hardwareScheduler;
};

#endif
