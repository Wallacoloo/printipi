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

/* Raspberry Pi float performance can be found here: http://www.raspberrypi.org/forums/viewtopic.php?t=7336
  float +,-,*: 2 cycles
  float /: 32 cycles (same for doubles)
  float sqrt: 48 cycles (same for doubles)
  float atan2: ?
    Could be approximated well in ~100 cycles (for float; more for double)
    This website has some efficient trig implementations: http://http.developer.nvidia.com/Cg/atan2.html
*/

/* Useful kinematics document: https://docs.google.com/viewer?a=v&pid=forums&srcid=MTgyNjQwODAyMDkxNzQxMTUwNzIBMDc2NTg4NjQ0MjUxMTE1ODY5OTkBdmZiejRRR2phZjhKATAuMQEBdjI
 * 
 *Locations of towers:
 * Each tower is situated at the corner of an equilateral triangle,
 *   at a distance 'r' from the center of the triangle.
 * Tower A is at (rsin(0)  , rcos(0)  ) = (0           , r     )
 * Tower B is at (rsin(120), rcos(120)) = (sqrt(3)/2 r , -1/2 r)
 * Tower C is at (rsin(240), rcos(240)) = (-sqrt(3)/2 r, -1/2 r)
 *
 * Top-down view:
 *         A
 *        /|\
 *       / | \
 *      /  |  \
 *     /  r|   \
 *    /    .    \
 *   /   (0,0)   \
 *  /             \
 * /               \
 *C-----------------B
 *
 * Front-face view:
 *
 * C         A        B
 * |\       /|     __/|
 * | \ L   / |  __/   |
 * |  \   /  |_/      |
 * |   \./__/|        |
 * | (x,y,z) |        |
 * |                  |
 * |                  |
 *
 * The '.' represents the effector.
 * Each tower has a rod of fixed-length 'L' connecting to the effector. 
 *   The other end of the rod is connected to a carriage that slides up and down the axis. The connection points allow the rot to pivot freely.
 * The height of the carriage above the bed is indicated by 'A' for the A carriage, 'B' for the B carriage, and 'C' for the C carriage.
 *
 * From here, we have the constraint equation: |P - <rsin(w), rcos(w), D>| = L
 *   where <rsin(w), rcos(w), D> is the carriage position and P is the effector position
 *
 * For linear movement, we are given P(t) = P0 + v*t
 * This is combined with the constraint equation and solved further down in the LinearDeltaStepper::testDir() function
 *
 *
 *
 *If we want to move in an ARC along x,y at a constant velocity (acceleration will be introduced later):
 *   a circle in 2d is x(t) = rcos(wt), y(t) = rsin(wt)
 *   can write this as P(t) = rcos(wt)*i + rsin(wt)*j
 *   Replace i and j with perpindicular vectors to extend to multiple dimensions:
 *   P(t) = <xc, yc, zc> + rcos(wt)*u+ rsin(wt)*v
 *   Let x0, y0, z0 be P(0) (the starting point), and P(end) = Pe=<xe, ye, ze>, and Pc=<xc, yc, zc> will be the center of the arc.
 *   The u is just <x0-xc, y0-yc, z0-zc>,
 *   and v will be a vector perpindicular to u and with equal magnitude that is in the plane of the arc.
 *
 *   Given u, v, Pc, and let m be the angular velocity:
 *     x = xc + r*Cos[m*t]*ux + r*Sin[m*t]*vx
 *     y = yc + r*Cos[m*t]*uy + r*Sin[m*t]*vy
 *     z = zc + r*Cos[m*t]*uz + r*Sin[m*t]*vz
 *
 *   This is solved further down in the LinearDeltaStepper::testDir() function.  
 *
 * Note: all motion in this file is planned at a constant velocity. 
 *   Cartesian-space acceleration is introduced by a post-transformation of the step times applied elsewhere in the motion planning system. 
 */


#ifndef MOTION_LINEARDELTASTEPPER_H
#define MOTION_LINEARDELTASTEPPER_H

#include "axisstepper.h"
#include "linearstepper.h" //for LinearHomeStepper
#include "iodrivers/endstop.h"
#include "common/logging.h"

namespace motion {

enum DeltaAxis {
    DELTA_AXIS_A=0,
    DELTA_AXIS_B=1,
    DELTA_AXIS_C=2,
    DELTA_AXIS_E=3
};

/* 
 * LinearDeltaStepper implements the AxisStepper interface for (rail-based) Delta-style robots like the Kossel, 
 *   for linear (G0/G1) and arc movements (G2/G3)
 */
template <typename StepperDriverT, DeltaAxis AxisIdx> class LinearDeltaStepper : public AxisStepperWithDriver<StepperDriverT> {
    private:
        const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
        float _r, _L, _MM_STEPS; //calibration settings which will be obtained from the CoordMap
        float w; //angle of this axis, in radians
        float M0; //initial coordinate of THIS axis. CW from +y axis
        int sTotal; //current step offset from M0
        
        //variables used during linear motion
        Vector3f line_P0; //initial cartesian position, in mm
        Vector3f line_v; //cartesian velocity vector, in mm/sec
        
        Vector3f arc_Pc; //arc centerpoint (cartesian)
        Vector3f arc_u, arc_v; //u and v vectors (cartesian); P(t) = Pc + u*cos(mt) + v*sin(mt)
        float arcRad; //radius of arc
        float arc_m; //angular velocity of the arc.
        bool isArcMotion;
        inline float r() const { return _r; }
        inline float L() const { return _L; }
        inline float MM_STEPS() const { return _MM_STEPS; }
    public:
        template <typename CoordMapT> LinearDeltaStepper(int idx, const CoordMapT &map, const StepperDriverT &stepper, const iodrv::Endstop *endstop)
         : AxisStepperWithDriver<StepperDriverT>(idx, stepper),
           endstop(endstop),
           _r(map.r()), 
           _L(map.L()),
           _MM_STEPS(map.MM_STEPS(AxisIdx)),
           w(AxisIdx*2*M_PI/3) {
              static_assert(AxisIdx < 3, "LinearDeltaStepper only supports axis A, B, or C (0, 1, 2)");
        }

        //linear motion initializer
        template <typename CoordMapT, std::size_t sz> void beginLine(const CoordMapT &map, const std::array<int, sz>& curPos, 
        const Vector4f &vel) {
            M0 = map.getAxisPosition(curPos, AxisIdx)*map.MM_STEPS(AxisIdx); 
            sTotal = 0;
            line_P0 = map.xyzeFromMechanical(curPos).xyz();
            line_v = vel.xyz();
            isArcMotion = false;
            this->time = 0; //this may NOT be zero-initialized by parent.
        }
        //arc motion initializer
        template <typename CoordMapT, std::size_t sz> void beginArc(const CoordMapT &map, const std::array<int, sz> &curPos, 
        const Vector3f &center, const Vector3f &u, const Vector3f &v,  
        float arcRad, float arcVel, float extVel) {
            (void)map, (void)extVel; //unused
            M0 = map.getAxisPosition(curPos, AxisIdx)*map.MM_STEPS(AxisIdx);
            sTotal = 0;
            arc_Pc = center;
            arc_u = u;
            arc_v = v;
            arcRad = arcRad;
            arc_m = arcVel;
            isArcMotion = true;
            this->time = 0; //this may NOT be zero-initialized by parent.
        }
    //protected:
        inline float testDir(float s) {
          	//returns the next time at which this axis should be at `s' mm above the initial position,
          	//  or NAN if the axis should never be at that position during the motion.
            if (isArcMotion) {
            	/*
            	 * Let P = P(t) = <xc, yc, zc> + s*cos(m*t)u + s*sin(m*t)v where s is the radius of the curve
    			     *   Have the constraint equation: |P - <rsin(w), rcos(w), D>| = L, where <rsin(w), rcos(w), D> is the carriage position and P is the effector position
    			     *   Then substitute P into the constraint equation and square each side:
    			     *     L^2 = |<xc-rsin(w), yc-rcos(w), zc-D> + s*cos(m*t)u + s*sin(m*t)v|^2
    			     *   Manually expand based upon the property that |v|^2 = v . v
    			     *     (xc-rsin(w))^2 + (yc-rcos(w))^2 + (zc-D)^2 + s^2*cos^2(m*t) + s^2*sin^2(m*t) 
    			       + 2<xc-rsin(w), yc-rcos(w), zc-D> . s*cos(m*t)u + 2<xc-rsin(w), yc-rcos(w), zc-D> . s*sin(m*t)v == L^2
    			     *   Note: removed all terms involving u . v, because u is perpindicular to v so u.v = 0
    			     *   Can manually simplify a bit and put into Mathematica notation. Note: used mt=m*t.
    			     *     (xc-r*Sin[w])^2 + (yc-r*Cos[w])^2 + (zc-D)^2 + s^2 
    			       + 2*s*{xc-r*Sin[w], yc-r*Cos[w], zc-D} . (Cos[mt]*u + Sin[mt]*v) == L^2
    			     *   Can directly apply Solve on the above equation and mt, but produces LARGE output.
    			       So apply FullSimplify on the above (with u->{ux, uy, uz}, v->{vx, vy, vz}):
    			     *     s^2+(D-zc)^2+(yc-r Cos[w])^2+(xc-r Sin[w])^2+2 s ((yc-r Cos[w]) (uy Cos[mt]+vy Sin[mt])+(-D+zc) (uz Cos[mt]+vz Sin[mt])+(ux Cos[mt]+vx Sin[mt]) (xc-r Sin[w])) == L^2
    			     *   Expand:
    			     *     0 == D^2-L^2+s^2+xc^2+yc^2-2 D zc+zc^2-2 D s uz Cos[mt]+2 s ux xc Cos[mt]+2 s uy yc Cos[mt]+2 s uz zc Cos[mt]-2 r yc Cos[w]-2 r s uy Cos[mt] Cos[w]+r^2 Cos[w]^2-2 D s vz Sin[mt]+2 s vx xc Sin[mt]+2 s vy yc Sin[mt]+2 s vz zc Sin[mt]-2 r s vy Cos[w] Sin[mt]-2 r xc Sin[w]-2 r s ux Cos[mt] Sin[w]-2 r s vx Sin[mt] Sin[w]+r^2 Sin[w]^2
    			     *   Apply Collect[%, {Sin[mt], Cos[mt]}] to above to group terms:
    			     *   0 ==  D^2-L^2+s^2+xc^2+yc^2-2 D zc+zc^2-2 r yc Cos[w]+r^2 Cos[w]^2-2 r xc Sin[w]+r^2 Sin[w]^2
    			     + Cos[mt] (-2 D s uz+2 s ux xc+2 s uy yc+2 s uz zc-2 r s uy Cos[w]-2 r s ux Sin[w])
    			     + Sin[mt] (-2 D s vz+2 s vx xc+2 s vy yc+2 s vz zc-2 r s vy Cos[w]-2 r s vx Sin[w])
    			     *   apply FullSimplify to each term:
    			     *   0 == -L^2+r^2+s^2+xc^2+yc^2+(D-zc)^2-2 r (yc Cos[w]+xc Sin[w])
    			     + Cos[mt] (2 s (-D uz+ux xc+uy yc+uz zc-r (uy Cos[w]+ux Sin[w])))
    			     + Sin[mt] (2 s (-D vz+vx xc+vy yc+vz zc-r (vy Cos[w]+vx Sin[w])))
    			     *   Now we can use the identity: {m,n,p} . {Sin[mt], Cos[mt], 1} == 0 has solutions of:
    			     *        mt = arctan((-n*p - m*sqrt(m*m+n*n-p*p))/(m*m+n*n),  (-m*p + n*sqrt(m*m+n*n-p*p))/(m*m + n*n)  )  where arctan(x, y) = atan(y/x)
    			     *     OR mt = arctan((-n*p + m*sqrt(m*m+n*n-p*p))/(m*m+n*n),  (-m*p - n*sqrt(m*m+n*n-p*p))/(m*m + n*n)  )
            	 */
                auto Pc = arc_Pc;
                auto u = arc_u;
                auto v = arc_v;
                float D = M0+s;
                //        r^2      +s^2          +xc^2 +yc^2 +(D-zc)^2     -2 r   (yc Cos[w]+xc Sin[w]) - L^2
                float p = r()*r()  +arcRad*arcRad+Pc.x()*Pc.x()+Pc.y()*Pc.y()+(D-Pc.z())*(D-Pc.z())-2*r()*(Pc.y()*cos(w)+Pc.x()*sin(w)) - L()*L();
                //        2 s      (-D uz+ux xc+uy yc+uz zc-r   (uy Cos[w]+ux Sin[w]))
                //float n = 2*arcRad*(-D*uz+ux*xc+uy*yc+uz*zc-r()*(uy*cos(w)+ux*sin(w)));
                float n = 2*arcRad*(-D*u.z()+u.dot(Pc) - r()*(u.y()*cos(w)+u.x()*sin(w)));
                //        2 s      (-D vz+vx xc+vy yc+vz zc-r   (vy Cos[w]+vx Sin[w]))
                float m = 2*arcRad*(-D*u.z()+u.dot(Pc) - r()*(v.y()*cos(w)+v.x()*sin(w)));
                
                float mt_1 = atan2((-m*p + n*sqrt(m*m+n*n-p*p))/(m*m + n*n), (-n*p - m*sqrt(m*m+n*n-p*p))/(m*m+n*n));
                float mt_2 = atan2((-m*p - n*sqrt(m*m+n*n-p*p))/(m*m + n*n), (-n*p + m*sqrt(m*m+n*n-p*p))/(m*m+n*n));
                float t1 = mt_1/this->arc_m;
                float t2 = mt_2/this->arc_m;
                //two possible solutions; choose the NEAREST one that is not in the past:
                //TODO: are solutions to mt = x +/- c*2*pi?
                if (t1 < this->time && t2 < this->time) { return NAN; }
                else if (t1 < this->time) { return t2; }
                else if (t2 < this->time) { return t1; }
                else { return std::min(t1, t2); }
            } else {
                /* For linear movement, we are given P(t) = P0 + v*t
                 * thus L^2 = |(P0+v*t) - {r*Sin[w], r*Cos[w], D}|^2
                 * manually expand based on property that |x|^2 = x . x:
                 *   L^2 = (P0+v*t) . (P0+v*t) - 2*(P0+v*t) . {r*Sin[w], r*Cos[w], D} + r^2*Sin[w]^2 + r^2*Cos[w]^2 + D^2
                 * manually simplify:
                 *   0 = (P0+v*t) . (P0+v*t) - 2*(P0+v*t) . {r*Sin[w], r*Cos[w], D} + r^2 + D^2 - L^2
                 *   0 = P0 . P0 + 2*t*P0 . v + t^2*v . v - 2*P0 . {r*Sin[w], r*Cos[w], D} - 2*t*v . {r*Sin[w], r*Cos[w], D} + r^2 + D^2 - L^2
                 * Collect t terms:
                 *   0 == P0 . P0 - 2*P0 . {r*Sin[w], r*Cos[w], D} + r^2 + D^2 - L^2
                      + t*(2*P0 . v - 2*v . {r*Sin[w], r*Cos[w], D})
                      + t^2*(v . v)
                 * Simplify the terms:
                 *   0 == |P0 - {r*Sin[w], r*Cos[w], D}|^2 - L^2
                      + t*2*v . (P0 - {r*Sin[w], r*Cos[w], D})
                      + t^2*(v . v)
                 * Apply quadratic equation to solve for t as a function of D.
                 * There are two solutions; both may be valid, but have different meanings. 
                 *   If one solution is at a time in the past, then it's just a projection of the current path into the past. 
                 *   If both solutions are in the future, then pick the nearest one; 
                 *   it means that there are two points in this path where the carriage should be at the same spot. 
                 *   This happens, for example, when the effector nears a tower for half of the line-segment, and 
                 *    then continues past the tower, so that the carriage movement is (pseudo) parabolic.
                 */
                float D = M0 + s;
                Vector3f carriagePos(r()*sin(w), r()*cos(w), D);
                //obtain coefficients for t = a*x^2 + b*x + c
                float a = line_v.magSq();
                float b = 2*line_v.dot(line_P0 - carriagePos);
                float c = (line_P0 - carriagePos).magSq() - L()*L();
                //solve t = (-b +- sqrt(b*b-4*a*c))/(2a)
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
                //float t1 = term1 - root;
                //float t2 = term1 + root;
                //return the nearest of the two times that is > current time.
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
                float negTime = testDir((this->sTotal-1)*MM_STEPS()); //get the time at which next steps would occur.
                float posTime = testDir((this->sTotal+1)*MM_STEPS());
                if (negTime < this->time || std::isnan(negTime)) { //negTime is invalid
                    if (posTime > this->time) {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (pos) vs %f (neg)\n", AxisIdx, posTime, negTime);
                        this->time = posTime;
                        this->direction = StepForward;
                        ++this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else if (posTime < this->time || std::isnan(posTime)) { //posTime is invalid
                    if (negTime > this->time) {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (neg) vs %f (pos)\n", AxisIdx, negTime, posTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else { //neither time is invalid
                    if (negTime < posTime) {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (neg) vs %f (pos)\n", AxisIdx, negTime, posTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (pos) vs %f (neg)\n", AxisIdx, posTime, negTime);
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

