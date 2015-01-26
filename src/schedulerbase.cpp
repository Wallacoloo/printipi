/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "schedulerbase.h"

#include <signal.h> //for sigaction signal handlers
#include <cstdlib> //for atexit
#include <stdexcept> //for runtime_error
#include "common/logging.h"

//initialize static variables:
std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> SchedulerBase::exitHandlers;
bool SchedulerBase::isExiting(false);

static void ctrlCOrZHandler(int s){
   printf("Caught signal %d\n",s);
   exit(1); 
}

static void segfaultHandler(int signal, siginfo_t *si, void *arg) {
    (void)signal; (void)arg; //unused
    printf("Caught segfault at address %p\n", si->si_addr);
    exit(1);
}

void SchedulerBase::callExitHandlers() {
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
    sigaction(SIGINT, &sigIntHandler, nullptr); //register ctrl+c
    sigaction(SIGTSTP, &sigIntHandler, nullptr); //register ctrl+z
    sigaction(SIGABRT, &sigIntHandler, nullptr); //register SIGABRT, which is triggered for critical errors (eg glibc detects double-free)
    sigaction(SIGTERM, &sigIntHandler, nullptr); //register SIGTERM
    sigaction(SIGILL, &sigIntHandler, nullptr); //resgier SIGILL - illegal instruction
    struct sigaction sa;
    //memset(&sa, 0, sizeof(sigaction));
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = segfaultHandler;
    sa.sa_flags   = SA_SIGINFO;
    sigaction(SIGSEGV, &sa, nullptr); //register segfault listener
}

bool SchedulerBase::registerExitHandler(void (*handler)(), unsigned level) {
    if (level > exitHandlers.size()) {
        throw std::runtime_error("Tried to register an exit handler at too high of a level");
    }
    exitHandlers[level].push_back(handler);
    return 0; //ok
}

