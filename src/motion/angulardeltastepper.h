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
 * Note: given F1x = 0, E1'x = 0, E1'y = E1y, E1'z = E1z, then the following relation holds:
 *     E1x^2 + |F1-E1'|^2 = E1x^2 + (F1y-E1y)^2 + (F1z-E1z)^2
 *                        = (F1x-E1x)^2 + (F1y-E1y)^2 + (F1z-E1z)^2
 *                        = |F1-E1|^2
 * Therefore:
 *   -(F1y-E1y)cos(a) - (F1z-E1z)sin(a) - 1/(2rf)*(re^2 - rf^2 - |F1-E1|^2) == 0
 * Also, -(F1y-E1y)cos(a) - (F1z-E1z)sin(a) = (F1-E1) . (0, cos(a), sin(a))
 * So
 *   -(F1-E1) . (0, cos(a), sin(a)) - 1/(2rf)*(re^2 - rf^2 - |F1-E1|^2) == 0
 * Multiply by 2rf and negate:
 *   2rf*(F1-E1) . (0, cos(a), sin(a)) + re^2 - rf^2 - |F1-E1|^2 == 0
 * Optionally, factor the dot product:
 *   re^2 - rf^2 + (F1-E1) . (2rf*(0, cos(a), sin(a)) - (F1-E1)) == 0
 *
 * This is solved later in the testDir function.
 */


#ifndef MOTION_ANGULARDELTASTEPPER_H
#define MOTION_ANGULARDELTASTEPPER_H

#include "axisstepper.h"
#include "linearstepper.h" //for LinearHomeStepper
#include "iodrivers/endstop.h"
#include "common/matrix.h"
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
    // calibration settings from the CoordMap
    float e, f, re, rf, _zoffset;
    float w; //angle of this arm about the +z axis, in radians
    float _RADIANS_STEP;

    float M0_rad; //initial coordinate of THIS axis in radians
    int sTotal; //current step offset from M0

    Vector3f F1; //position of F1 joint in our YZ reference frame

    //variables used during linear motion
    Vector3f line_E1v; //velocity of the E1 point in our YZ reference frame
    Vector3f line_E1_0; //initial position of the E1 point in our YZ reference frame at the start of the movement
    float line_inverseVelocitySquared;

    //variables used during arc motion
    Vector3f arc_E1_0; //arc centerpoint (cartesian)
    Vector3f arc_u, arc_v; //u and v vectors (cartesian); P(t) = Pc + u*cos(mt) + v*sin(mt)
    float arc_rad; //radius of arc
    float arc_m; //angular velocity of the arc.
    bool isArcMotion;

    float RADIANS_STEP() const { return _RADIANS_STEP; } //number of radians per step
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
           _RADIANS_STEP(map.DEGREES_STEP(axisIdx) * M_PI / 180.0f) {
              // E axis has to be controlled using a LinearStepper (as if it were cartesian)
              assert(axisIdx == ANGULARDELTA_AXIS_A || axisIdx == ANGULARDELTA_AXIS_B || axisIdx == ANGULARDELTA_AXIS_C);
              // The location of F1 is fixed in our rotated reference frame
              this->F1 = Vector3f(0, -f/(2*sqrtf(3.f)), _zoffset);
        }

        //function to initiate a linear (through cartesian space) motion
        template <typename CoordMapT, std::size_t sz> void beginLine(const CoordMapT &map, const std::array<int, sz>& curPos, 
        const Vector4f &vel) {
            this->M0_rad = map.getAxisPosition(curPos, axisIdx)*RADIANS_STEP(); 
            this->sTotal = 0;
            this->time = 0;
            this->isArcMotion = false;
            Vector3f line_P0 = map.xyzeFromMechanical(curPos).xyz();
            Vector3f line_v = vel.xyz();

            // rotate the cartesian position function into our flat YZ reference frame
            auto rot = Matrix3x3::rotationAboutPositiveZ(-w);
            this->line_E1v = rot.transform(line_v);
            // line_P0 specifies the initial center of the effector; translate that into our rotated coordinate system
            Vector3f E0_0 = rot.transform(line_P0);
            // E1 is a fixed offset from E0
            this->line_E1_0 = E0_0  + Vector3f(0, -e/(2*sqrt(3)), 0);
            this->line_inverseVelocitySquared = 1.f / line_v.magSq();
        }
        //function to initiate a circular arc (through cartesian space) motion
        template <typename CoordMapT, std::size_t sz> void beginArc(const CoordMapT &map, const std::array<int, sz> &curPos, 
        const Vector3f &center, const Vector3f &u, const Vector3f &v,  
        float arcRad, float arcVel, float extVel) {
            (void)extVel; //unused
            this->M0_rad = map.getAxisPosition(curPos, axisIdx)*RADIANS_STEP(); 
            this->sTotal = 0;
            this->time = 0;
            this->isArcMotion = true;

            // rotate the cartesian position function into our flat YZ reference frame
            auto rot = Matrix3x3::rotationAboutPositiveZ(-w);
            this->arc_u = rot.transform(u);
            this->arc_v = rot.transform(v);
            Vector3f E0_0 = rot.transform(center);
            // E1 is a fixed offset from E0
            this->arc_E1_0 = E0_0  + Vector3f(0, -e/(2*sqrt(3)), 0);
            this->arc_rad = arcRad;
            this->arc_m = arcVel;
        }

        inline float testDir(float s) {
            if (isArcMotion) {
                /*
                 * Given the constraint equations derived further up the page:
                 *   2rf*(F1-E1) . (0, cos(a), sin(a)) + re^2 - rf^2 - |F1-E1|^2 == 0
                 * For circular motion, E1 = E1o + s*cos(m*t)u + s*sin(m*t)v
                 *   where s is the radius of the curve, u and v are two perpindicular vectors from the curve center to its edge,
                 * We desire to test at what time t will a = D*RADIANS_STEP = (M0+s)*RADIANS_STEP
                 *   0 == 2rf*(F1 - E1o - s*cos(m*t)u - s*sin(m*t)v) . (0, cos(a), sin(a)) + re^2 - rf^2 - |(F1 - E1o) - s*cos(m*t)u - s*sin(m*t)v|^2
                 * Substitute:
                 *   0 == 2rf*[(F1y-E1'oy)cos(a) + (F1z-E1'oz)sin(a) - s*uy*cos(m*t)*cos(a) - s*vy*sin(m*t)*cos(a) - s*uz*cos(m*t)*sin(a) - s*vz*sin(m*t)*sin(a)] + re^2 - rf^2 - [|F1-E1o|^2 + s^2*cos(m*t)^2|u|^2 + s^2*sin(m*t)^2|v|^2 - 2*(F1-E1o) . s*cos(m*t)u - 2*(F1-E1o) . s*sin(m*t)v + s^2*cos(m*t)sin(m*t)*u . v]
                 * u and v are perpindicular, therefore u . v = 0
                 * also |u|=1 and |v|=1
                 *   0 == 2rf*[(F1y-E1'oy)cos(a) + (F1z-E1'oz)sin(a) - s*uy*cos(m*t)*cos(a) - s*vy*sin(m*t)*cos(a) - s*uz*cos(m*t)*sin(a) - s*vz*sin(m*t)*sin(a)] + re^2 - rf^2 - [|F1-E1o|^2 + s^2 - 2*(F1-E1o) . s*cos(m*t)u - 2*(F1-E1o) . s*sin(m*t)v]
                 * Expand to separate the cos(m*t) from the sin(m*t):
                 *   0 == 2rf*[(F1y-E1'oy)cos(a) + (F1z-E1'oz)sin(a)] - 2rf*s*uy*cos(m*t)*cos(a) - 2rf*s*vy*sin(m*t)*cos(a) - 2rf*s*uz*cos(m*t)*sin(a) - 2rf*s*vz*sin(m*t)*sin(a) + re^2 - rf^2 - |F1-E1o|^2 - s^2 + 2*(F1-E1o) . s*cos(m*t)u + 2*(F1-E1o) . s*sin(m*t)v
                 * Group the cos(m*t), sin(m*t) and constant terms:
                 *   0 == 2rf*[(F1y-E1'oy)cos(a) + (F1z-E1'oz)sin(a)] + re^2 - rf^2 - |F1-E1o|^2 - s^2
                 *     + cos(m*t)*[ - 2rf*s*uy*cos(a) - 2rf*s*uz*sin(a) + 2*(F1-E1o) . s*u]
                 *     + sin(m*t)*[ - 2rf*s*vy*cos(a) - 2rf*s*vz*sin(a) + 2*(F1-E1o) . s*v]
                 *
                 * Now we can use the identity: {m,n,p} . {Sin[q], Cos[q], 1} == 0 has solutions of:
                 *        q = arctan((-n*p - m*sqrt(m*m+n*n-p*p))/(m*m+n*n),  (-m*p + n*sqrt(m*m+n*n-p*p))/(m*m + n*n)  )  where arctan(x, y) = atan(y/x)
                 *     OR q = arctan((-n*p + m*sqrt(m*m+n*n-p*p))/(m*m+n*n),  (-m*p - n*sqrt(m*m+n*n-p*p))/(m*m + n*n)  )
                 */
                // determine the queried angle of our arm, in radians
                float angle = M0_rad+s;
                float sinAngle = sinf(angle);
                float cosAngle = cosf(angle);
                Vector3f E1_0 = arc_E1_0;
                Vector3f u = arc_u;
                Vector3f v = arc_v;
                // sin(m*t)*[ - 2rf*s*vy*cos(a) - 2rf*s*vz*sin(a) + 2*(F1-E1o) . s*v]
                float m = -2*rf*arc_rad*v.dot(0, cosAngle, sinAngle) + 2*arc_rad*(F1-E1_0).dot(v);
                // cos(m*t)*[ - 2rf*s*uy*cos(a) - 2rf*s*uz*sin(a) + 2*(F1-E1o) . s*u]
                float n = -2*rf*arc_rad*u.dot(0, cosAngle, sinAngle) + 2*arc_rad*(F1-E1_0).dot(u);
                // 2rf*[(F1y-E1'oy)cos(a) + (F1z-E1'oz)sin(a)] + re^2 - rf^2 - |F1-E1o|^2 - s^2*|u|^2
                float p = 2*rf*(F1-E1_0).dot(0, cosAngle, sinAngle) + re*re - rf*rf - (F1-E1_0).magSq() - arc_rad*arc_rad;

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
            } else {
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
                
                Vector3f E1v = line_E1v;
                Vector3f E1_0 = line_E1_0;
                // determine the queried angle of our arm, in radians
                float angle = M0_rad+s;
                float sinAngle = sinf(angle);
                float cosAngle = cosf(angle);
                
                // determine the coefficients to at^2 + bt + c = 0, which gives the time at which our arm will be at the above angle (possibly non-existant).
                // unoptimized (preserve for reference)
                // float a = E1v.x()*E1v.x() + E1primev.magSq();
                // float b = 2*(rf*E1primev.y()*cos(angle) + rf*E1primev.z()*sin(angle)  + E1_0.x()*E1v.x() - (F1-E1prime0).dot(E1primev));
                // float c = -2*rf*(F1.y()-E1prime0.y())*cosAngle - 2*rf*(F1.z()-E1prime0.z())*sinAngle - re*re  + E1_0.x()*E1_0.x() + rf*rf + (F1-E1prime0).magSq();
                
                // More efficient implementation than the above
                // cache the inverse of a to avoid a float division
                float inv_a = line_inverseVelocitySquared;
                // calculate only b/(2*a) (see below for explanation)
                // float b_2a = inv_a*(rf*E1primev.y()*cosAngle + rf*E1primev.z()*sinAngle  + E1_0.x()*E1v.x() - (F1-E1prime0).dot(E1primev));
                float b_2a = inv_a*(rf*E1v.y()*cosAngle + rf*E1v.z()*sinAngle  - (F1-E1_0).dot(E1v));
                // calculate only c/a (see below for explanation)
                // float c_a = inv_a*(-2*rf*(F1.y()-E1prime0.y())*cosAngle - 2*rf*(F1.z()-E1prime0.z())*sinAngle - re*re  + E1_0.x()*E1_0.x() + rf*rf + (F1-E1prime0).magSq());
                float c_a = inv_a*(-2*rf*(F1.y()-E1_0.y())*cosAngle - 2*rf*(F1.z()-E1_0.z())*sinAngle - re*re  + rf*rf + (F1-E1_0).magSq());

                // Solve the quadratic equation: x = (-b +/- sqrt(b^2-4ac)) / (2a)
                // Note: if we divide numerator and denominator by 2, we get:
                //   x = (-b/2 +/- sqrt((b/2)^2-ac)) / a, which is slightly easier to compute given that our b has a factor of 2.
                // also, divide top and bottom by a, and we get:
                //   x = -b/a/2 +/- sqrt((b/a/2)^2-c/a)
                // we can do that because a is guaranteed to be positive, since it is the square of the velocity
                float term1 = -b_2a;
                float rootParam = b_2a*b_2a - c_a;
                //check if the sqrt argument is negative
                //TODO: can it ever be negative?
                if (rootParam < 0) {
                    return NAN;
                }
                float root = sqrtf(rootParam);
                float t1 = term1 - root;
                float t2 = term1 + root;
                //return the nearest of the two times that is > current time.
                //note: root will always be positive.
                if (root > term1) { //if this is true, then t1 MUST be negative.
                    //return t2 if t2 > last_step_time else None
                    return t2 > this->time ? t2 : NAN;
                } else {
                    return t1 > this->time ? t1 : (t2 > this->time ? t2 : NAN); //ensure no value < time is returned.
                }
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
                float negTime = testDir((this->sTotal-1)*RADIANS_STEP()); //get the time at which next steps would occur.
                float posTime = testDir((this->sTotal+1)*RADIANS_STEP());
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

