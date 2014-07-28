#include "rpi.h"
#include "bcm2835.h"

//#include <cstdlib> //for atexit
#include "scheduler.h"
#include "logging.h"

namespace drv {
namespace rpi {

bool wasBcmInit = false;

void atexit_bcm2835_close() {
	LOG("freeing bcm2835 memmaps\n");
	bcm2835_close(); //return value is int. Must be wrapped as void function for std::atexit to be used.
}

void initIO() {
	if (!wasBcmInit) { //it's OK to double-init (or double-free), but just wastes CPU, and we only really want 1 exit handler.
		wasBcmInit = true;
		LOG("Initializing bcm2835 memmaps\n");
		if (bcm2835_init()) {
			SchedulerBase::registerExitHandler(atexit_bcm2835_close, SCHED_MEM_EXIT_LEVEL);
			//std::atexit(atexit_bcm2835_close);
		}
	}
}

}
}
