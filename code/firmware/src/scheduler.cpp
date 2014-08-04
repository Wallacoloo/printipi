#include "scheduler.h"

//#include <pthread.h> //for pthread_setschedparam
//#include <time.h> //for clock_nanosleep
#include <signal.h> //for sigaction signal handlers
#include <cstdlib> //for atexit
#include <stdexcept> //for runtime_error

std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> SchedulerBase::exitHandlers;
//std::atomic<bool> SchedulerBase::isExiting(false);
bool SchedulerBase::isExiting(false);

void ctrlCOrZHandler(int s){
   printf("Caught signal %d\n",s);
   exit(1); 
}

void segfaultHandler(int /*signal*/, siginfo_t *si, void */*arg*/) {
    printf("Caught segfault at address %p\n", si->si_addr);
    exit(1);
}

void SchedulerBase::callExitHandlers() {
	//if (!isExiting.exchange(true)) { //try setting isExiting to true. If it was previously false, then call the exit handlers:
	if (!isExiting) {
		isExiting = true;
		LOG("Exiting\n");
		for (const std::vector<void(*)()>& level : exitHandlers) {
			for (void(*handler)() : level) {
				(*handler)();
			}
		}
	}
}


void SchedulerBase::configureExitHandlers() {
	std::atexit((void(*)())&SchedulerBase::callExitHandlers);
	//listen for ctrl+c, ctrl+z and segfaults. Then try to properly unmount any I/Os (crucial for disabling the heated nozzle)
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = ctrlCOrZHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL); //register ctrl+c
	sigaction(SIGTSTP, &sigIntHandler, NULL); //register ctrl+z
	sigaction(SIGABRT, &sigIntHandler, NULL); //register SIGABRT, which is triggered for critical errors (eg glibc detects double-free)
	
	struct sigaction sa;
    //memset(&sa, 0, sizeof(sigaction));
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = segfaultHandler;
    sa.sa_flags   = SA_SIGINFO;
    sigaction(SIGSEGV, &sa, NULL); //register segfault listener
}

void SchedulerBase::registerExitHandler(void (*handler)(), unsigned level) {
	if (level > exitHandlers.size()) {
		throw std::runtime_error("Tried to register an exit handler at too high of a level");
	}
	exitHandlers[level].push_back(handler);
	//std::atexit(handler);
}

