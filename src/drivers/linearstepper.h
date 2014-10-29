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
 * Printipi/drivers/linearstepper.h
 * 
 * LinearStepper implements the AxisStepper interface for Cartesian-style robots.
 * Additionally, the LinearHomeStepper is used to home the axis for some other types of robots.
 */
 

#ifndef LINEARSTEPPER_H
#define LINEARSTEPPER_H

#include "axisstepper.h"
#include "endstop.h"
#include "common/logging.h"
#include <tuple>
#include <cmath> //for fabs

enum CoordAxis {
    COORD_X,
    COORD_Y,
    COORD_Z,
    COORD_E
};

namespace drv {

template <int STEPS_M, typename EndstopT> class LinearHomeStepper : public AxisStepper {
    EndstopT endstop;
    float timePerStep;
    static constexpr float STEPS_MM = STEPS_M / 1000.;
    public:
        LinearHomeStepper() {}
        LinearHomeStepper(int idx, float vHome) : AxisStepper(idx, vHome) {
            this->time = 0;
            this->direction = StepForward;
            this->timePerStep = 1./ (vHome*STEPS_MM);
        }
        
        void _nextStep() {
            //if (EndstopT::isTriggered()) {
            if (endstop.isTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                this->time += timePerStep;
            }
        }
};

template <int STEPS_M> class LinearHomeStepper<STEPS_M, EndstopNoExist> : public AxisStepper {
    public:
        LinearHomeStepper() {}
        LinearHomeStepper(int idx, float vHome) : AxisStepper(idx, vHome) {
            this->time = NAN; //device isn't homeable, so never step.
        }
        void _nextStep() {}
};

template <int STEPS_PER_METER, CoordAxis CoordType, typename EndstopT=EndstopNoExist> class LinearStepper : public AxisStepper {
    private:
        static constexpr float STEPS_MM = STEPS_PER_METER/1000.0;
    private:
        float timePerStep;
    public:
        typedef LinearHomeStepper<STEPS_PER_METER, EndstopT> HomeStepperT;
        static constexpr float GET_COORD(float x, float y, float z, float e) {
            static_assert(CoordType==COORD_X || CoordType==COORD_Y || CoordType==COORD_Z || CoordType==COORD_E, "CoordType can only be x, y, z, or e");
            return CoordType==COORD_X ? x : \
                  (CoordType==COORD_Y ? y : \
                  (CoordType==COORD_Z ? z : 
                  (CoordType==COORD_E ? e : 0) ) );
        }
        static constexpr float TIME_PER_STEP(float vx, float vy, float vz, float ve) {
            //return units of time. v is mm/sec, STEPS_MM is steps/mm.
            //therefore v*STEPS_MM = steps/sec.
            //1/(v*STEPS_MM) is sec/steps.
            //multiplied by 1 step and units are sec. Therefore t = 1/(v*STEPS_MM);
            //NOTE: this may return a NEGATIVE time, indicating that the stepping direction is backward.
            return 1./ (GET_COORD(vx, vy, vz, ve) * STEPS_MM);
        }
        LinearStepper() {}
        template <std::size_t sz> LinearStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
            : AxisStepper(idx, curPos, vx, vy, vz, ve),
            timePerStep(std::fabs( TIME_PER_STEP(vx, vy, vz, ve) )) {
                this->time = 0;
                this->direction = stepDirFromSign( TIME_PER_STEP(vx, vy, vz, ve) );
            }
        void _nextStep() {
            this->time += timePerStep;
        }
};

}

#endif
