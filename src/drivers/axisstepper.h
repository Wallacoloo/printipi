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
 * Printipi/drivers/axisstepper.h
 *
 * AxisSteppers are used to queue movements.
 * When a movement is desired, an AxisStepper is instantiated for each MECHANICAL axis (eg each pillar of a Kossel, plus extruders. Or perhaps an X stepper, a Y stepper, a Z stepper, and an extruder for a cartesian bot).
 * The AxisStepper provides the relative time at which its associated axis should next be advanced, as well as in what mechanical direction, given an initial mechanical position and cartesian velocity.
 * It also implements the 'nextStep' method, which will update the time & direction of the step that would follow the current one. In this way, the AxisStepper can be queried for the 1st step, 2nd step, and so on, for the given path.
 *
 * Note: AxisStepper is an interface, and not an implementation.
 * An implementation is needed for each coordinate style - Cartesian, deltabot, etc.
 * These implementations must provide the functions outlined further down in the header.
 */
 

#ifndef DRIVERS_AXISSTEPPER_H
#define DRIVERS_AXISSTEPPER_H

#include "event.h"
#include "compileflags.h" //for AxisIdType
#include <tuple>
#include <array>
#include <cmath> //for isnan
#include "common/tupleutil.h"

namespace drv {

class AxisStepper {
    private:
        AxisIdType _index; //ID of axis. Does not necessarily have to be stored as a variable (other option is one template instance per ID, which pretty much already happens), but this allows AxisStepper::nextStep() to not be virtual
    public:
        float time; //time of next step
        StepDirection direction; //direction of next step
        inline int index() const { return _index; } //NOT TO BE OVERRIDEN
        AxisStepper() {}
        //standard initializer:
        template <std::size_t sz> AxisStepper(int idx, const std::array<int, sz>& /*curPos*/, float /*vx*/, float /*vy*/, float /*vz*/, float /*ve*/)
            : _index(idx) {}
        //initializer when homing to endstops:
        AxisStepper(int idx, float /*vHome*/) : _index(idx) {}
        template <typename TupleT> static AxisStepper& getNextTime(TupleT &axes);
        template <typename TupleT, std::size_t MechSize> static void initAxisSteppers(TupleT &steppers, const std::array<int, MechSize>& curPos, float vx, float vy, float vz, float ve);
        template <typename TupleT, std::size_t MechSize> static void initAxisArcSteppers(TupleT &steppers, const std::array<int, MechSize>& curPos, float xCenter, float yCenter, float zCenter, float ux, float uy, float uz, float vx, float vy, float vz, float arcRad, float arcVel, float extVel);
        template <typename TupleT> static void initAxisHomeSteppers(TupleT &steppers, float vHome);
        Event getEvent() const; //NOT TO BE OVERRIDEN
        Event getEvent(float realTime) const; //NOT TO BE OVERRIDEN
        template <typename TupleT> void nextStep(TupleT &axes); //NOT TO BE OVERRIDEN
    protected:
        AxisStepper(int idx) : _index(idx) {} //only callable via children
        void _nextStep(); //OVERRIDE THIS. Will be called upon initialization.
    public:
        //homing steppers. GetHomeStepperTypes<std::tuple<StepperT, ...> >::HomeStepperTypes is equivalent to std::tuple<StepperT::HomeStepperT, ...>
        template <typename... Types> struct GetHomeStepperTypes {
            typedef std::tuple<typename Types::HomeStepperT...> HomeStepperTypes;
        };
        template <typename... Types> struct GetHomeStepperTypes<std::tuple<Types...> > : GetHomeStepperTypes<Types...> {};
        //Arc steppers:
        template <typename... Types> struct GetArcStepperTypes {
            typedef std::tuple<typename Types::ArcStepperT...> ArcStepperTypes;
        };
        template <typename... Types> struct GetArcStepperTypes<std::tuple<Types...> > : GetArcStepperTypes<Types...> {};
        
};

//Helper classes for AxisStepper::getNextTime method
//C++ doesn't support partial template function specialization, so we need to use templated classes instead.
//Below function(s) select the AxisStepper with the smallest time attribute from a tuple of such AxisSteppers
template <typename TupleT, int idx> struct _AxisStepper__getNextTime {
    AxisStepper& operator()(TupleT &axes) {
        AxisStepper &m1 = _AxisStepper__getNextTime<TupleT, idx-1>()(axes);
        AxisStepper &m2 = std::get<idx>(axes);
        //assume that .time can be finite, infinite, or NaN.
        //comparisons against NaN are ALWAYS false.
        if (m1.time <= 0) { return m2; } //if one of the times is non-positive (ie no next step), return the other one.
        if (m2.time <= 0) { return m1; }
        //Now return the smallest of the two, discarding any NaNs:
        //if m2.time == NaN, then (m1.time < m2.time || isnan(m2.time)) ? m1 : m2 will return m1.time
        //elif m1.time == NaN, then (m1.time < m2.time || isnan(m2.time)) ? m1 : m2 will return m2.time
        return (m1.time < m2.time || std::isnan(m2.time)) ? m1 : m2;
    }
};

//TODO: re-implement with tupleutil

template <typename TupleT> struct _AxisStepper__getNextTime<TupleT, 0> {
    AxisStepper& operator()(TupleT &axes) {
        return std::get<0>(axes);
    }
};

template <typename TupleT> AxisStepper& AxisStepper::getNextTime(TupleT &axes) {
    return _AxisStepper__getNextTime<TupleT, std::tuple_size<TupleT>::value-1>()(axes);
}

//Helper classes for AxisStepper::initAxisSteppers

struct _AxisStepper__initAxisSteppers {
    template <std::size_t MyIdx, typename TupleT, typename T, std::size_t MechSize> void operator()(CVTemplateWrapper<MyIdx> _myIdx, T &stepper, TupleT *steppers, std::array<int, MechSize>& curPos, float vx, float vy, float vz, float ve) {
        (void)_myIdx; (void)stepper; //unused
        std::get<MyIdx>(*steppers) = T(MyIdx, curPos, vx, vy, vz, ve);
        std::get<MyIdx>(*steppers)._nextStep();
    }
};

template <typename TupleT, std::size_t MechSize> void AxisStepper::initAxisSteppers(TupleT &steppers, const std::array<int, MechSize>& curPos, float vx, float vy, float vz, float ve) {
    callOnAll(steppers, _AxisStepper__initAxisSteppers(), &steppers, curPos, vx, vy, vz, ve);
}

//Helper classes for AxisStepper::initAxisHomeSteppers

struct _AxisStepper__initAxisHomeSteppers {
    template <std::size_t MyIdx, typename T, typename TupleT> void operator()(CVTemplateWrapper<MyIdx> _myIdx, T &stepper, TupleT *steppers, float vHome) {
        (void)_myIdx; (void)stepper; //unused
        std::get<MyIdx>(*steppers) = T(MyIdx, vHome);
        std::get<MyIdx>(*steppers)._nextStep();
    }
};

template <typename TupleT> void AxisStepper::initAxisHomeSteppers(TupleT &steppers, float vHome) {
    callOnAll(steppers, _AxisStepper__initAxisHomeSteppers(), &steppers, vHome);
}

//Helper classes for AxisStepper::initAxisArcSteppers

struct _AxisStepper__initAxisArcSteppers {
    template <std::size_t MyIdx, typename TupleT, typename T, std::size_t MechSize> void operator()(CVTemplateWrapper<MyIdx> _myIdx, T &stepper, TupleT *steppers, std::array<int, MechSize>& curPos, float xCenter, float yCenter, float zCenter, float ux, float uy, float uz, float vx, float vy, float vz, float arcRad, float arcVel, float extVel) {
        (void)_myIdx; (void)stepper; //unused
        std::get<MyIdx>(*steppers) = T(MyIdx, curPos, xCenter, yCenter, zCenter, ux, uy, uz, vx, vy, vz, arcRad, arcVel, extVel);
        std::get<MyIdx>(*steppers)._nextStep();
    }
};

template <typename TupleT, std::size_t MechSize> void AxisStepper::initAxisArcSteppers(TupleT &steppers, const std::array<int, MechSize>& curPos, float xCenter, float yCenter, float zCenter, float ux, float uy, float uz, float vx, float vy, float vz, float arcRad, float arcVel, float extVel) {
    callOnAll(steppers, _AxisStepper__initAxisArcSteppers(), &steppers, curPos, xCenter, yCenter, zCenter, ux, uy, uz, vx, vy, vz, arcRad, arcVel, extVel);
}

//Helper classes for AxisStepper::nextStep method
//this iterates through all steppers and checks if their index is equal to the index of the desired stepper to step.
//if so, it calls _nextStep().
//This allows for _nextStep to act as if it were virtual (by defining a method of that name in a derived type), but without using a vtable.
//It also allows for the compiler to easily optimize the if statements into a jump-table.

struct _AxisStepper__nextStep {
    template <typename T> void operator()(std::size_t myIdx, T &stepper, std::size_t desiredIdx) {
        if (desiredIdx == myIdx) {
            stepper._nextStep();
        }
    }
};
template <typename TupleT> void AxisStepper::nextStep(TupleT &axes) {
    callOnAll(axes, _AxisStepper__nextStep(), this->index());
}




}

#endif
