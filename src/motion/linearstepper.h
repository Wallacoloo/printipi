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
template <typename StepperDriverT> class LinearStepper : public AxisStepperWithDriver<StepperDriverT> {
    CartesianAxis coordType;
    const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
    float line_timePerStep;

    // for arc, x(t) = arc_center + cos(arc_m*t)*arc_su + sin(arc_m*t)*arc_sv;
    float arc_su, arc_sv;
    float arc_center, arc_m;
    int arc_sTotal;

    bool isArcMotion;
    float _MM_STEPS;
    float MM_STEPS() const { return _MM_STEPS; }
    public:
        template <typename CoordMapT> inline LinearStepper(int idx, CartesianAxis coordType, const CoordMapT &map, const StepperDriverT &stepper, const iodrv::Endstop *endstop)
         : AxisStepperWithDriver<StepperDriverT>(idx, stepper),
           coordType(coordType),
           endstop(endstop),
           _MM_STEPS(map.MM_STEPS(coordType)) {
            (void)map;
            assert(coordType==CARTESIAN_AXIS_X || coordType==CARTESIAN_AXIS_Y || coordType==CARTESIAN_AXIS_Z || coordType==CARTESIAN_AXIS_E);
        }
        //Linear movement constructor
        template <typename CoordMapT, std::size_t sz> void beginLine(const CoordMapT &map, const std::array<int, sz>& curPos, 
          const Vector4f &vel) {
            (void)curPos; //unused
            //timePerStep is in units of sec/step. v is mm/sec, STEPS_MM is steps/mm.
            //therefore v*STEPS_MM = steps/sec, so 1. / (v*STEPS_MM) is steps/sec
            float myVel = vel.array()[coordType];
            float signedTimePerStep = 1. / (myVel * map.STEPS_MM(coordType));
            line_timePerStep = std::fabs(signedTimePerStep);
            this->time = 0;
            this->direction = stepDirFromSign(signedTimePerStep);
            this->isArcMotion = false;
        }
        //Arc movement constructor
        template <typename CoordMapT, std::size_t sz> void beginArc(const CoordMapT &map, const std::array<int, sz> &curPos, 
          const Vector3f &center, const Vector3f &u, const Vector3f &v,  
          float arcRad, float arcVel, float extVel) {
            (void)map; (void)curPos; (void)center; (void)u; (void)v; (void)arcRad; (void)arcVel; //unused
            if (coordType == CARTESIAN_AXIS_E) {
                // extruder movement for cartesian arcs is still linear.
                beginLine(map, curPos, Vector4f(0.f, 0.f, 0.f, extVel));
                //timePerStep = std::fabs(1./ (extVel * map.STEPS_MM(coordType)));
                //this->time = 0;
                //this->direction = stepDirFromSign(extVel);
            } else {
                // Arc motion is:
                // Let P = P(t) = Po + s*cos(m*t)u + s*sin(m*t)v where s is the radius of the curve
                // First thing to note is that x, y, z are all independent in this form
                // e.g. Px = Pox + s*cos(m*t)ux + s*sin(m*t)vx
                // Given a P, want to solve for t:
                // 0 == s*cos(m*t)ux + s*sin(m*t)vx + Pox - Px
                // 0 == cos(m*t)*[s*ux] 
                //    + sin(m*t)*[s*vx] 
                //    + Pox - Px
                // Now we can use the identity: {m,n,p} . {Sin[mt], Cos[mt], 1} == 0 has solutions of:
                //       mt = arctan((-n*p - m*sqrt(m*m+n*n-p*p))/(m*m+n*n),  (-m*p + n*sqrt(m*m+n*n-p*p))/(m*m + n*n)  )  where arctan(x, y) = atan(y/x)
                //    OR mt = arctan((-n*p + m*sqrt(m*m+n*n-p*p))/(m*m+n*n),  (-m*p - n*sqrt(m*m+n*n-p*p))/(m*m + n*n)  )
                this->isArcMotion = true;
                this->arc_su = arcRad*u.array()[coordType];
                this->arc_sv = arcRad*v.array()[coordType];
                this->arc_center = center.array()[coordType];
                this->arc_m = arcVel;
                this->arc_sTotal = map.getAxisPosition(curPos, coordType);
                this->time = 0;
            }
        }
        inline float arcTestDir(float s) {
            // for arc, x(t) = arc_center + cos(arc_m*t)*arc_su + sin(arc_m*t)*arc_sv;
            float m = arc_sv;
            float n = arc_su;
            float p = arc_center - s;
            // solve {m,n,p} . {Sin[q], Cos[q], 1} == 0:
            float mt_1 = atan2f((-m*p + n*sqrtf(m*m+n*n-p*p))/(m*m + n*n), (-n*p - m*sqrtf(m*m+n*n-p*p))/(m*m+n*n));
            float mt_2 = atan2f((-m*p - n*sqrtf(m*m+n*n-p*p))/(m*m + n*n), (-n*p + m*sqrtf(m*m+n*n-p*p))/(m*m+n*n));
            float t1 = mt_1/this->arc_m;
            float t2 = mt_2/this->arc_m;
            //two possible solutions; choose the NEAREST one that is not in the past:
            //TODO: are solutions to mt = x +/- c*2*pi?
            if (t1 < this->time && t2 < this->time) { return NAN; }
            else if (t1 < this->time) { return t2; }
            else if (t2 < this->time) { return t1; }
            else { return std::min(t1, t2); }
        }
    //protected:
        inline void _nextStep(bool useEndstops) {
            if (useEndstops && endstop->isEndstopTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                if (isArcMotion) {
                    float negTime = arcTestDir((this->arc_sTotal-1)*MM_STEPS()); //get the time at which next steps would occur.
                    float posTime = arcTestDir((this->arc_sTotal+1)*MM_STEPS());
                    if (negTime < this->time || std::isnan(negTime)) { //negTime is invalid
                        if (posTime > this->time) {
                            //LOGV("LinearDeltaStepper<%u>::chose %f (pos) vs %f (neg)\n", axisIdx, posTime, negTime);
                            this->time = posTime;
                            this->direction = StepForward;
                            ++this->arc_sTotal;
                        } else {
                            this->time = NAN;
                        }
                    } else if (posTime < this->time || std::isnan(posTime)) { //posTime is invalid
                        if (negTime > this->time) {
                            //LOGV("LinearDeltaStepper<%u>::chose %f (neg) vs %f (pos)\n", axisIdx, negTime, posTime);
                            this->time = negTime;
                            this->direction = StepBackward;
                            --this->arc_sTotal;
                        } else {
                            this->time = NAN;
                        }
                    } else { //neither time is invalid
                        if (negTime < posTime) {
                            //LOGV("LinearDeltaStepper<%u>::chose %f (neg) vs %f (pos)\n", axisIdx, negTime, posTime);
                            this->time = negTime;
                            this->direction = StepBackward;
                            --this->arc_sTotal;
                        } else {
                            //LOGV("LinearDeltaStepper<%u>::chose %f (pos) vs %f (neg)\n", axisIdx, posTime, negTime);
                            this->time = posTime;
                            this->direction = StepForward;
                            ++this->arc_sTotal;
                        }
                    }
                } else {
                    this->time += line_timePerStep;
                }
            }
        }
};

}

#endif
