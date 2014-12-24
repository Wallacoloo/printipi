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
 * Printipi/motion/linearstepper.h
 * 
 * LinearStepper implements the AxisStepper interface for Cartesian-style robots.
 * Additionally, the LinearHomeStepper is used to home the axis for some other types of robots.
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

template <typename StepperDriverT, std::size_t AxisIdx> class LinearHomeStepper : public AxisStepperWithDriver<StepperDriverT> {
    const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
    float timePerStep;
    public:
        inline LinearHomeStepper() : endstop(nullptr) {}
        template <typename CoordMapT> LinearHomeStepper(int idx, const CoordMapT &map, float vHome)
          : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<AxisIdx>()), endstop(&map.getEndstop(AxisIdx)) {
            (void)map; //unused
            this->time = 0;
            this->direction = StepForward;
            this->timePerStep = 1./ (vHome*map.STEPS_MM(AxisIdx));
        }
        
        inline void _nextStep() {
            if (endstop->isNull() || endstop->isTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                this->time += timePerStep;
            }
        }
};


template <typename StepperDriverT, CartesianAxis CoordType> class LinearStepper : public AxisStepperWithDriver<StepperDriverT> {
    private:
        float timePerStep;
    public:
        typedef LinearHomeStepper<StepperDriverT, CoordType> HomeStepperT;
        typedef LinearStepper<StepperDriverT, CoordType> ArcStepperT;
    protected:
        inline float GET_COORD(float x, float y, float z, float e) const {
            //static_assert(CoordType==CARTESIAN_AXIS_X || CoordType==CARTESIAN_AXIS_Y || CoordType==CARTESIAN_AXIS_Z || CoordType==CARTESIAN_AXIS_E, "CoordType can only be x, y, z, or e");
            return CoordType==CARTESIAN_AXIS_X ? x : \
                  (CoordType==CARTESIAN_AXIS_Y ? y : \
                  (CoordType==CARTESIAN_AXIS_Z ? z : 
                  (CoordType==CARTESIAN_AXIS_E ? e : 0) ) );
        }
        template <typename CoordMapT> float TIME_PER_STEP(const CoordMapT &map, float vx, float vy, float vz, float ve) const {
            //return units of time. v is mm/sec, STEPS_MM is steps/mm.
            //therefore v*STEPS_MM = steps/sec.
            //1/(v*STEPS_MM) is sec/steps.
            //multiplied by 1 step and units are sec. Therefore t = 1/(v*STEPS_MM);
            //NOTE: this may return a NEGATIVE time, indicating that the stepping direction is backward.
            return 1./ (GET_COORD(vx, vy, vz, ve) * map.STEPS_MM(CoordType));
        }
    public:
        //default constructor
        inline LinearStepper() {}
        //Linear movement constructor
        template <typename CoordMapT, std::size_t sz> LinearStepper(int idx, const CoordMapT &map, const std::array<int, sz>& curPos, 
          float vx, float vy, float vz, float ve)
        : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<CoordType>()),
          timePerStep(std::fabs( TIME_PER_STEP(map, vx, vy, vz, ve) )) {
                (void)map; (void)curPos; //unused
                this->time = 0;
                this->direction = stepDirFromSign( TIME_PER_STEP(map, vx, vy, vz, ve) );
            }
        //Arc movement constructor
        template <typename CoordMapT, std::size_t sz> LinearStepper(int idx, const CoordMapT &map, const std::array<int, sz> &curPos, 
          float centerX, float centerY, float centerZ, float ux, float uy, float uz, float vx, float vy, float vz, 
          float arcRad, float arcVel, float extVel)
        : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<CoordType>()), 
          timePerStep(std::fabs(1./ (extVel * map.STEPS_MM(CoordType)))) {
            (void)map; (void)idx; (void)curPos; (void)centerX; (void)centerY; (void)centerZ; (void)ux; (void)uy; (void)uz; (void)vx; (void)vy; (void)vz; (void)arcRad; (void)arcVel; //unused
            assert(CoordType == CARTESIAN_AXIS_E); //can only use a LinearStepper as an Arc implementation for the extruder axis. 
            this->time = 0;
            this->direction = stepDirFromSign(extVel);
        }
    //protected:
        inline void _nextStep() {
            this->time += timePerStep;
        }
};

}

#endif
