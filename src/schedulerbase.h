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
 
/*
 * Printipi/schedulerbase.h
 *
 * SchedulerBase allows any file to insert exit handlers, without adding the entire Scheduler as a dependency.
 */

#ifndef SCHEDULERBASE_H
#define SCHEDULERBASE_H

#include <array>
#include <vector>


#ifndef SCHED_PRIORITY
    //Linux scheduler priority. Higher = more realtime
    #define SCHED_PRIORITY 30
#endif
#ifndef SCHED_NUM_EXIT_HANDLER_LEVELS
    #define SCHED_NUM_EXIT_HANDLER_LEVELS 2
#endif
#define SCHED_IO_EXIT_LEVEL 0
#define SCHED_MEM_EXIT_LEVEL 1

//Scheduler::Interface::onIdleCpu() can be called with a flag indicating (roughly) how long it's been since it was last called.
enum OnIdleCpuIntervalT {
    OnIdleCpuIntervalShort,
    OnIdleCpuIntervalWide
};


/* Base class from which all templated schedulers derive.
Defines things such as exit handlers */
class SchedulerBase {
    //for the exitHandlers, we could use a set, but a vector is even less likely to fail,
    //  and the exitHandlers are called in the case of an extreme error (eg segfauly; corrupted data)
    static std::array<std::vector<void(*)()>, SCHED_NUM_EXIT_HANDLER_LEVELS> exitHandlers;
    static bool isExiting;
    private:
        static void callExitHandlers();
    public:
        static void configureExitHandlers();
        //need to return a value so we can do tricks like `static bool _wasInit=registerExitHandler(...)` to do something just once
        static bool registerExitHandler(void (*handler)(), unsigned level);
};

#endif
