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

/* Raspberry Pi (1 A/B/A+/B+) float performance can be found here: http://www.raspberrypi.org/forums/viewtopic.php?t=7336
  float +,-,*: 2 cycles
  float /: 32 cycles (same for doubles)
  float sqrt: 48 cycles (same for doubles)
  float atan2: ?
    Could be approximated well in ~100 cycles (for float; more for double)
    This website has some efficient trig implementations: http://http.developer.nvidia.com/Cg/atan2.html
*/

/* Kinematics described in detail here: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
 *
 * For solving this just in the YZ plane,
 * we get the following constraints:
 * |J1 - E1| = re, |F1-J1| = rf and J1 = F1 + rf<0, -cos(a), -sin(a)>
 * Since E1' = E1-x, this implies the following more useful constraints:
 * |J1 - E1'| = sqrt(re^2-x^2), and the above: |F1-J1| = rf and J1 = F1 + rf<0, -cos(a), -sin(a)>
 *
 * Note: y-z plane is defined with directions:
 *        ^ 
 *        | z
 *        |
 * <------+
 *  y     
 *
 * Rewrite |x|^2 as x . x:
 *       (J1 - E1') . (J1 - E1') == re^2-E1x^2
 *   and (J1 - F1) . (J1 - F1)   == rf^2
 * Expand:
 *       J1 . J1 - 2 J1 . E1' + E1' . E1' == re^2-E1x^2
 *   and J1 . J1 - 2 J1 . F1  + F1  . F1  == rf^2
 * Subtract equations:
 *   - 2 J1 . E1' + 2 J1 . F1 + E1' . E1' - F1 . F1 == re^2-E1x^2 - rf^2
 * Simplify:
 *   2 J1 . (F1 - E1') + E1' . E1' - F1 . F1 == re^2-E1x^2 - rf^2
 * Substitute J1 = F1 + rf<0, -cos(a), -sin(a)>
 *   2(F1 + rf<0, -cos(a), -sin(a)>) . (F1 - E1') + E1' . E1' - F1 . F1 == re^2-E1x^2 - rf^2
 * Expand:
 *   2F1 . F1 - 2F1 . E1' + 2rf<0, -cos(a), -sin(a)> . (F1-E1') + E1' . E1' - F1 . F1 == re^2-E1x^2 - rf^2
 * Combine terms:
 *    F1 . F1 - 2F1 . E1' + 2rf<0, -cos(a), -sin(a)> . (F1-E1') + E1' . E1' == re^2-E1x^2 - rf^2
 * Rearrange:
 *    F1 . F1 - 2F1 . E1' + E1' . E1' + 2rf<0, -cos(a), -sin(a)> . (F1-E1') == re^2-E1x^2 - rf^2
 * Notice that F1 . F1 - 2F1 . E1' + E1' . E1' is equal to (F1 - E1') . (F1 - E1') = |F1 - E1'|^2
 *   2rf<0, -cos(a), -sin(a)> . (F1-E1') == re^2-E1x^2 - rf^2 - |F1-E1'|^2
 * Expand and divide by 2rf:
 *   -(F1y-E1'y)cos(a) - (F1z-E1'z)sin(a) == 1/2rf*(re^2-E1x^2 - rf^2 - |F1-E1'|^2)
 * Move to one side:
 *   -(F1y-E1'y)cos(a) - (F1z-E1'z)sin(a) - 1/2rf*(re^2-E1x^2 - rf^2 - |F1-E1'|^2) == 0
 *
 * This is solved later in the testDir function.
 */


#ifndef MOTION_ANGULARDELTASTEPPER_H
#define MOTION_ANGULARDELTASTEPPER_H

#include "axisstepper.h"
#include "linearstepper.h" //for LinearHomeStepper
#include "iodrivers/endstop.h"
#include "common/logging.h"

namespace motion {

enum DeltaAxis {
    ANGULARDELTA_AXIS_A=0,
    ANGULARDELTA_AXIS_B=1,
    ANGULARDELTA_AXIS_C=2,
    ANGULARDELTA_AXIS_E=3
};

/* 
 * LinearDeltaStepper implements the AxisStepper interface for (rail-based) Delta-style robots like the Kossel, 
 *   for linear (G0/G1) and arc movements (G2/G3)
 */
template <typename StepperDriverT> class AngularDeltaStepper : public AxisStepperWithDriver<StepperDriverT> {
    DeltaAxis axisIdx;
    const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
    float e, f, re, rf, _zoffset;
    float w; //angle of this arm about the +z axis, in radians
    float _DEGREES_STEP;

    float M0; //initial coordinate of THIS axis in degrees
    int sTotal; //current step offset from M0

    //variables used during linear motion
    Vector3f line_P0; //initial cartesian position, in mm
    Vector3f line_v; //cartesian velocity vector, in mm/sec
    float DEGREES_STEP() const { return _DEGREES_STEP; } //number of degrees per step
    public:
        template <typename CoordMapT> AngularDeltaStepper(
            int idx, DeltaAxis axisIdx, const CoordMapT &map, const StepperDriverT &stepper, const iodrv::Endstop *endstop,
            float e, float f, float re, float rf, float zoffset)
         : AxisStepperWithDriver<StepperDriverT>(idx, stepper),
           axisIdx(axisIdx),
           endstop(endstop),
           e(e),
           f(f),
           re(re),
           rf(rf),
           _zoffset(zoffset),
           w(axisIdx*2*M_PI/3), 
           _DEGREES_STEP(map.DEGREES_STEP(axisIdx)) {
              //E axis has to be controlled using a LinearStepper (as if it were cartesian)
              assert(axisIdx == ANGULARDELTA_AXIS_A || axisIdx == ANGULARDELTA_AXIS_B || axisIdx == ANGULARDELTA_AXIS_C);
        }

        //function to initiate a linear (through cartesian space) motion
        template <typename CoordMapT, std::size_t sz> void beginLine(const CoordMapT &map, const std::array<int, sz>& curPos, 
        const Vector4f &vel) {
            this->M0 = map.getAxisPosition(curPos, axisIdx)*map.DEGREES_STEP(axisIdx); 
            this->sTotal = 0;
            this->line_P0 = map.xyzeFromMechanical(curPos).xyz();
            this->line_v = vel.xyz();
            this->time = 0;
        }
        //function to initiate a circular arc (through cartesian space) motion
        template <typename CoordMapT, std::size_t sz> void beginArc(const CoordMapT &map, const std::array<int, sz> &curPos, 
        const Vector3f &center, const Vector3f &u, const Vector3f &v,  
        float arcRad, float arcVel, float extVel) {
            //arc motions not yet supported.
            (void)map; (void)curPos; (void)center; (void)u; (void)v; (void)arcRad; (void)arcVel; (void)extVel; //unused
            assert(false);
        }

        inline float testDir(float s) {
            /*
             * Given the constraint equations derived further up the page:
             *    -(F1y-E1'y)cos(a) - (F1z-E1'z)sin(a) - 1/(2rf)*(re^2-E1x^2 - rf^2 - |F1-E1'|^2) == 0
             * For linear motion, E1' = E1'o + E1'v*t and E1 = E1o + E1v*t
             * Then -(F1y-E1'oy-E1'vy*t)cos(a) - (F1z-E1'oz-E1'vz*t)sin(a) - 1/(2rf)*(re^2-(E1ox+E1vx*t)^2 - rf^2 - |F1-E1'o-E1'v*t|^2) == 0
             *
             * We desire to test at what time t will a = D*RADIANS_STEP = (M0+s)*RADIANS_STEP
             * Expand to separate the various powers of t:
             *    -(F1y-E1'oy)cos(D) + E1'vy*t*cos(D) - (F1z-E1'oz)sin(D) + E1'vz*t*sin(D) - 1/(2rf)*(re^2 - (E1ox^2+2*E1ox*E1vx*t+E1vx^2*t^2) - rf^2 - (F1-E1'o-E1'v*t) . (F1-E1'o-E1'v*t)) == 0
             * Multiply by 2rf & expand a little more
             *    2rf( -(F1y-E1'oy)cos(D) + E1'vy*t*cos(D) - (F1z-E1'oz)sin(D) + E1'vz*t*sin(D) ) - re^2 + E1ox^2+2*E1ox*E1vx*t+E1vx^2*t^2 + rf^2 + (F1-E1'o) . (F1-E1'o) - 2(F1-E1'o) . E1'v*t + |E1'v*t|^2 == 0
             * Group by t^0, t^1 and t^2
             *    t^2 * (E1vx^2 + |E1'v|^2)
                + t*(2rf*E1'vy*cos(D) + 2rf*E1'vz*sin(D) + 2*E1ox*E1vx - 2(F1-E1'o) . E1'v)
                + -2rf(F1y-E1'oy)*cos(D) - 2rf(F1z-E1'oz)sin(D) - re^2 + E1ox^2 + rf^2 + (F1-E1'o) . (F1-E1'o) == 0
             * This is a quadratic equation of t that we can solve using t = (-b +/- sqrt(b^2-4a*c)) / (2*a)
             * There are two solutions; both may be valid, but have different meanings. 
             *   If one solution is at a time in the past, then it's just a projection of the current path into the past. 
             *   If both solutions are in the future, then pick the nearest one; 
             *   it means that there are two points in this path where the arm angle should be the same.
             */
            // rotate everything by the angle of this axis so that we can do all calculations in the 'y-z' plane
            auto rot = Matrix3x3::rotationAboutPositiveZ(-w);
            auto E1v = rot.transform(line_v);
            // The location of F1 is fixed in our rotated reference frame
            Vector3f F1 = Vector3f(0, -f/(2*sqrt(3)), _zoffset);
            // line_P0 specifies the initial center of the effector; translate that into our rotated coordinate system
            Vector3f E0_0 = rot.transform(line_P0);
            // E1 is a fixed offset from E0
            Vector3f E1_0 = E0_0  + Vector3f(0, -e/(2*sqrt(3)), 0);
            // E1' is the projection of E1 onto the y-z plane.
            Vector3f E1prime0 = Vector3f(0, E1_0.y(), E1_0.z()); //initial E1' at the start of the move.
            Vector3f E1primev = Vector3f(0, E1v.y(), E1v.z()); // movement of E1' (which is restricted to our plane)
            // determine the queried angle of our arm, in radians
            float angle = (M0+s) * M_PI / 180.0;
            // determine the coefficients to at^2 + bt + c = 0, which gives the time at which our arm will be at the above angle (possibly non-existant).
            float a = E1v.x()*E1v.x() + E1primev.magSq();
            float b = 2*rf*E1primev.y()*cos(angle) + 2*rf*E1primev.z()*sin(angle)  + 2*E1_0.x()*E1v.x() - 2*(F1-E1prime0).dot(E1primev);
            float c = -2*rf*(F1.y()-E1prime0.y())*cos(angle) - 2*rf*(F1.z()-E1prime0.z())*sin(angle) - re*re  + E1_0.x()*E1_0.x() + rf*rf + (F1-E1prime0).magSq();

            // Solve the quadratic equation: x = (-b +/- sqrt(b^2-4ac)) / (2a)
            float term1 = -b;
            float rootParam = (b*b-4*a*c);
            float divisor = 2*a;
            //check if the sqrt argument is negative
            //TODO: can it ever be negative?
            if (rootParam < 0) {
                return NAN;
            }
            float root = std::sqrt(rootParam);
            float t1 = (term1 - root)/divisor;
            float t2 = (term1 + root)/divisor;
            //return the nearest of the two times that is > current time.
            if (root > term1) { //if this is true, then t1 MUST be negative.
                //return t2 if t2 > last_step_time else None
                return t2 > this->time ? t2 : NAN;
            } else {
                return t1 > this->time ? t1 : (t2 > this->time ? t2 : NAN); //ensure no value < time is returned.
            }
        }
    
        inline void _nextStep(bool useEndstops) {
            //called to set this->time and this->direction; the time (in seconds) and the direction at which the next step should occur for this axis
            //General formula is outlined in comments at the top of this file.
            //First, we test the time at which a forward step (sTotal + 1) should occur given constant angular velocity.
            //Then we test that time for a backward step (sTotal - 1).
            //We choose the nearest resulting time as our next step.
            //This is necessary because axis velocity can actually reverse direction during a circular cartesian movement.
            if (useEndstops && endstop->isEndstopTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                float negTime = testDir((this->sTotal-1)*DEGREES_STEP()); //get the time at which next steps would occur.
                float posTime = testDir((this->sTotal+1)*DEGREES_STEP());
                if (negTime < this->time || std::isnan(negTime)) { //negTime is invalid
                    if (posTime > this->time) {
                        LOGV("LinearDeltaStepper<%u>::chose %f (pos) vs %f (neg)\n", axisIdx, posTime, negTime);
                        this->time = posTime;
                        this->direction = StepForward;
                        ++this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else if (posTime < this->time || std::isnan(posTime)) { //posTime is invalid
                    if (negTime > this->time) {
                        LOGV("LinearDeltaStepper<%u>::chose %f (neg) vs %f (pos)\n", axisIdx, negTime, posTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else { //neither time is invalid
                    if (negTime < posTime) {
                        LOGV("LinearDeltaStepper<%u>::chose %f (neg) vs %f (pos)\n", axisIdx, negTime, posTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        LOGV("LinearDeltaStepper<%u>::chose %f (pos) vs %f (neg)\n", axisIdx, posTime, negTime);
                        this->time = posTime;
                        this->direction = StepForward;
                        ++this->sTotal;
                    }
                }
            }
        }
};


}


#endif

