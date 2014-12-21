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
 * Printipi/event.h
 *
 * An Event encapsulates information about something that needs to happen at a specific time.
 *   Eg, "advance stepper #2 at time T", or "set hotend #1 high at time T"
 * Events are queued in the Scheduler and then handled by the State at their appropriate time.
 */

#ifndef EVENT_H
#define EVENT_H

//There will be these events:
//  step forward (motor 0, 1, 2, 3)
//  step backward (motor 0, 1, 2, 3)

#include "compileflags.h" //for AxisIdType
#include "platforms/auto/chronoclock.h" //for EventClockT

enum StepDirection {
    StepBackward,
    StepForward
};

template <typename T> StepDirection stepDirFromSign(T dir) {
    return dir < 0 ? StepBackward : StepForward;
}
template <typename T> T stepDirToSigned(StepDirection dir) {
    return dir == StepBackward ? -1 : 1;
}

/*class Event {
    EventClockT::time_point _time;
    AxisIdType _stepperNum;
    bool _isForward;
    public:
        static const AxisIdType NULL_STEPPER_ID = 255;
        inline AxisIdType stepperId() const {
            return this->_stepperNum;
        }
        inline StepDirection direction() const {
            return this->_isForward ? StepForward : StepBackward;
        }
        inline EventClockT::time_point time() const {
            return _time;
        }
        inline bool isNull() const {
            return this->stepperId() == NULL_STEPPER_ID;
        }
        inline Event() : _time(), _stepperNum(NULL_STEPPER_ID) {}
        inline Event(EventClockT::time_point t, AxisIdType stepperNum, StepDirection dir) : _time(t), _stepperNum(stepperNum), _isForward(dir==StepForward) {}
        static Event StepperEvent(float relTime, AxisIdType stepperNum, StepDirection dir);
        
        template <typename DurationT> void offset(const DurationT offset) {
            this->_time += std::chrono::duration_cast<EventClockT::duration>(offset);
        }
        inline bool operator<(const Event &other) const {
            return this->time() < other.time();
        }
        inline bool operator>(const Event &other) const {
            return this->time() > other.time();
        }
        
};*/

#endif
