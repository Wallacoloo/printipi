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
 

#ifndef MOTION_LINEARSTEPPER_H
#define MOTION_LINEARSTEPPER_H

#include "axisstepper.h"
#include "iodrivers/endstop.h"
#include "common/logging.h"
#include <tuple>
#include <cmath> //for fabs
#include <cassert>

namespace motion {

enum CartesianAxis {
    CARTESIAN_AXIS_X=0,
    CARTESIAN_AXIS_Y=1,
    CARTESIAN_AXIS_Z=2,
    CARTESIAN_AXIS_E=3
};

/* 
 * LinearStepper implements the AxisStepper interface for Cartesian-style robots.
 */
template <typename StepperDriverT, CartesianAxis CoordType> class LinearStepper : public AxisStepperWithDriver<StepperDriverT> {
    const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
    float timePerStep;
    public:
        typedef LinearStepper<StepperDriverT, CoordType> ArcStepperT;
    protected:
        template <typename CoordMapT> float TIME_PER_STEP(const CoordMapT &map, const Vector4f &vel) const {
            //return units of time. v is mm/sec, STEPS_MM is steps/mm.
            //therefore v*STEPS_MM = steps/sec.
            //1/(v*STEPS_MM) is sec/steps.
            //multiplied by 1 step and units are sec. Therefore t = 1/(v*STEPS_MM);
            //NOTE: this may return a NEGATIVE time, indicating that the stepping direction is backward.
            assert(CoordType==CARTESIAN_AXIS_X || CoordType==CARTESIAN_AXIS_Y || CoordType==CARTESIAN_AXIS_Z || CoordType==CARTESIAN_AXIS_E);
            return 1./ (vel.array()[CoordType] * map.STEPS_MM(CoordType));
        }
    public:
        //default constructor
        inline LinearStepper() : endstop(nullptr) {}
        //Linear movement constructor
        template <typename CoordMapT, std::size_t sz> LinearStepper(int idx, const CoordMapT &map, const std::array<int, sz>& curPos, 
          const Vector4f &vel)
        : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<CoordType>()),
          endstop(&map.getEndstop(CoordType)),
          timePerStep(std::fabs( TIME_PER_STEP(map, vel) )) {
                (void)map; (void)curPos; //unused
                this->time = 0;
                this->direction = stepDirFromSign( TIME_PER_STEP(map, vel) );
            }
        //Arc movement constructor
        template <typename CoordMapT, std::size_t sz> LinearStepper(int idx, const CoordMapT &map, const std::array<int, sz> &curPos, 
          const Vector3f &center, const Vector3f &u, const Vector3f &v,  
          float arcRad, float arcVel, float extVel)
        : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<CoordType>()), 
          endstop(&map.getEndstop(CoordType)),
          timePerStep(std::fabs(1./ (extVel * map.STEPS_MM(CoordType)))) {
            (void)map; (void)idx; (void)curPos; (void)center; (void)u; (void)v; (void)arcRad; (void)arcVel; //unused
            assert(CoordType == CARTESIAN_AXIS_E); //can only use a LinearStepper as an Arc implementation for the extruder axis, since that moves at const velocity
            this->time = 0;
            this->direction = stepDirFromSign(extVel);
        }
    //protected:
        inline void _nextStep(bool useEndstops) {
            if (useEndstops && endstop->isEndstopTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                this->time += timePerStep;
            }
        }
};

}

#endif
