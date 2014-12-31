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
 * Printipi/motion/lineardeltastepper.h
 * 
 * LinearDeltaStepper implements the AxisStepper interface for (rail-based) Delta-style robots like the Kossel
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
 * Each tower has a rod of fixed-length 'L' connecting to the effector. The other end of the rod is connected to a carriage that slides up and down the axis. The connection points allow the rot to pivot freely.
 * The height of the carriage above the bed is indicated by 'A' for the A carriage, 'B' for the B carriage, and 'C' for the C carriage.
 *
 * This gives us the following equations for relating A, B, C to x, y, z:
 * (A-z)^2 + (x-rsin(0)  )^2 + (y-rcos(0)  )^2 = L^2
 * (B-z)^2 + (x-rsin(120))^2 + (y-rcos(120))^2 = L^2
 * (C-z)^2 + (x-rsin(240))^2 + (y-rcos(240))^2 = L^2
 *
 * We can solve this system for A, B, and C to get the "Inverse Kinematics":
 * A = z + sqrt(L^2 - (y-rcos(0)  )^2 - (x-rsin(0)  )^2)
 * B = z + sqrt(L^2 - (y-rcos(120))^2 - (x-rsin(120))^2)
 * C = z + sqrt(L^2 - (y-rcos(240))^2 - (x-rsin(240))^2)
 *
 * If we want to move linearly along x,y,z at a constant velocity (acceleration will be introduced later):
 *   let x(t) = x0 + vx t
 *       y(t) = y0 + vy t
 *       z(t) = z0 + vz t
 *
 * Then A(t) = z0 + vz*t + sqrt( L^2 - (y0 + vy*t - rcos(0)  )^2 + (x0 + vx*t - rsin(0)  )^2 )
 *      B(t) = z0 + vz*t + sqrt( L^2 - (y0 + vy*t - rcos(120))^2 + (x0 + vx*t - rsin(120))^2 )
 *      C(t) = z0 + vz*t + sqrt( L^2 - (y0 + vy*t - rcos(240))^2 + (x0 + vx*t - rsin(240))^2 )
 *
 * Now, if we are at A=A0, we want to find the next time at which we'll be at A=A0 +/- 1,
 *   That is, the time at which A is stepped exactly +/-1 step.
 *   It is important to note that along a linear movement, it is possible for dA/dt to change sign. That is, a given carriage might be moving up at the beginning of the line, and moving down at the end of the line.
 *
 * So, let A0 = A(t=0).
 * Then we want to solve A(t) = A0 + s for t, where s is the number of steps from A0.
 *   This will allow us to test t(A0-1) and t(A0+1) to determine the direction and time to take our first step. If it is positive, then we will next test t(A0+1-1) and t(A0+1+1), and so on, to form our path.
 * To make this generic, we will replace A, B, C with D, and cos(0), cos(120), ... with cos(w).
 *   That way we only have to solve one axis, and can then obtain results for all axes.
 * 
 * D0 + s = z0 + vz*t + sqrt( L^2 - (y0 + vy*t - rcos(w))^2 + (x0 + vx*t - rsin(w))^2 )
 * Expand:
 *   (D0 + s - z0 - vz*t)^2 = L^2 - ((y0 - rcos(w)) + vy*t)^2 + ((x0 - rsin(w)) + vx*t)^2
 *
 *   ((D0 + s - z0) - vz*t)^2 - L^2 = (y0-rcos(w))^2 + 2vy*t*(y0 - rcos(w)) + vy*vy*t*t   +   (x0-rsin(w))^2 + 2vx*t*(x0-rsin(w)) + vx*vx*t
 *
 *   (D0 + s - z0)^2 - 2(D0 + s - z0)*vz*t + vz*vz*t*t - L^2 = (y0-rcos(w))^2 + 2vy*t*(y0 - rcos(w)) + vy*vy*t*t   +   (x0-rsin(w))^2 + 2vx*t*(x0-rsin(w)) + vx*vx*t
 *
 * This looks like a quadratic equation of t; group the powers of t:
 *   0 = -(D0 + s - z0)^2 + 2(D0 + s - z0)*vz*t - vz*vz*t*t + L^2 + (y0-rcos(w))^2 + 2vy*t*(y0 - rcos(w)) + vy*vy*t*t   +   (x0-rsin(w))^2 + 2vx*t*(x0-rsin(w)) + vx*vx*t
 *
 *   0 = t^2*(-vz^2 + vy^2 +vx^2)  +  t*(2(D0 + s - z0)*vz + 2vy*(y0 - rcos(w)) + 2vx*(x0-rsin(w)))  +  (-(D0 + s - z0)^2 + L^2 + (y0-rcos(w))^2 + (x0-rsin(w))^2)
 *
 * Thus, 0 = a t^2 + b t + c, where
 *   a = -vz^2 + vy^2 +vx^2
 *   b = 2(D0 + s - z0)*vz + 2vy*(y0 - rcos(w)) + 2vx*(x0-rsin(w))
 *   c = -(D0 + s - z0)^2 + L^2 + (y0-rcos(w))^2 + (x0-rsin(w))^2
 * So t = [-b +/- sqrt(b^2 - 4 ac)] / [2a]  according to the quadratic formula
 *
 * There are two solutions; both may be valid, but have different meanings. If one solution is at a time in the past, then it's just a projection of the current path into the past. If both solutions are in the future, then pick the nearest one; it means that there are two points in this path where the carriage should be at the same spot. This happens, for example, when the effector nears a tower for half of the line-segment, and then continues past the tower, so that the carriage movement is (pseudo) parabolic.
 * The above quadratic solution is used in LinearDeltaStepper::_nextStep(), although it has been optimized.
 *
 * Note: all motion in this file is planned at a constant velocity. Cartesian-space acceleration is introduced by a post-transformation of the step times applied elsewhere in the motion planning system.
 *
 *
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
 *   This is solved further down in the testDir() function.   
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

template <typename StepperDriverT, DeltaAxis AxisIdx> class LinearDeltaArcStepper : public AxisStepperWithDriver<StepperDriverT> {
    private:
        const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
        float _r, _L, _MM_STEPS; //calibration settings which will be obtained from the CoordMap
        float M0; //initial coordinate of THIS axis.
        int sTotal; //current step offset from M0
        float xc, yc, zc; //arc centerpoint
        float ux, uy, uz; //u and v vectors; P(t) = <xc, yc, zc> + u*cos(mt) + v*sin(mt)
        float vx, vy, vz; 
        float arcRad; //radius of arc
        float m; //angular velocity of the arc.
        float w; //angle of this axis. CW from +y axis
        inline float r() const { return _r; }
        inline float L() const { return _L; }
        inline float MM_STEPS() const { return _MM_STEPS; }
    public:
        inline LinearDeltaArcStepper() : endstop(nullptr), _r(0), _L(0), _MM_STEPS(0) {}
        template <typename CoordMapT, std::size_t sz> LinearDeltaArcStepper(int idx, const CoordMapT &map, const std::array<int, sz> &curPos, 
        const Vector3f &center, const Vector3f &u, const Vector3f &v,  
        float arcRad, float arcVel, float extVel)
        : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<AxisIdx>()),
          endstop(&map.getEndstop(AxisIdx)),
          _r(map.r()),
          _L(map.L()),
          _MM_STEPS(map.MM_STEPS(AxisIdx)),
          M0(map.getAxisPosition(curPos, AxisIdx)*map.MM_STEPS(AxisIdx)), 
          sTotal(0),
          xc(center.x()),
          yc(center.y()),
          zc(center.z()),
          ux(u.x()),
          uy(u.y()),
          uz(u.z()),
          vx(v.x()),
          vy(v.y()),
          vz(v.z()),
          arcRad(arcRad),
          m(arcVel),
          w(AxisIdx*2*M_PI/3) {
                (void)map, (void)idx; (void)extVel; //unused
                static_assert(AxisIdx < 3, "LinearDeltaStepper only supports axis A, B, or C (0, 1, 2)");
                this->time = 0; //this may NOT be zero-initialized by parent.
        }
    //protected:
        inline float testDir(float s) {
        	//returns the next time at which this axis should be at `s' mm above the initial position,
        	//  or NAN if the axis should never be at that position during the motion.

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
            float D = M0+s;
            //        r^2      +s^2          +xc^2 +yc^2 +(D-zc)^2     -2 r   (yc Cos[w]+xc Sin[w]) - L^2
            float p = r()*r()  +arcRad*arcRad+xc*xc+yc*yc+(D-zc)*(D-zc)-2*r()*(yc*cos(w)+xc*sin(w)) - L()*L();
            //        2 s      (-D uz+ux xc+uy yc+uz zc-r   (uy Cos[w]+ux Sin[w]))
            float n = 2*arcRad*(-D*uz+ux*xc+uy*yc+uz*zc-r()*(uy*cos(w)+ux*sin(w)));
            //        2 s      (-D vz+vx xc+vy yc+vz zc-r   (vy Cos[w]+vx Sin[w]))
            float m = 2*arcRad*(-D*vz+vx*xc+vy*yc+vz*zc-r()*(vy*cos(w)+vx*sin(w)));
            
            float mt_1 = atan2((-m*p + n*sqrt(m*m+n*n-p*p))/(m*m + n*n), (-n*p - m*sqrt(m*m+n*n-p*p))/(m*m+n*n));
            float mt_2 = atan2((-m*p - n*sqrt(m*m+n*n-p*p))/(m*m + n*n), (-n*p + m*sqrt(m*m+n*n-p*p))/(m*m+n*n));
            float t1 = mt_1/this->m;
            float t2 = mt_2/this->m;
            //two possible solutions; choose the NEAREST one that is not in the past:
            //TODO: are solutions to mt = x +/- c*2*pi?
            if (t1 < this->time && t2 < this->time) { return NAN; }
            else if (t1 < this->time) { return t2; }
            else if (t2 < this->time) { return t1; }
            else { return std::min(t1, t2); }
        }
        inline void _nextStep(bool useEndstops) {
            //called to set this->time and this->direction; the time (in seconds) and the direction at which the next step should occur for this axis
            //General formula is outlined in comments at the top of this file.
            //First, we test the time at which a forward step (sTotal + 1) should occur given constant angular velocity.
            //Then we test that time for a backward step (sTotal - 1).
            //We choose the nearest resulting time as our next step.
            //This is necessary because axis velocity can actually reverse direction during a circular cartesian movement.
            if (useEndstops && endstop->isTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                float negTime = testDir((this->sTotal-1)*MM_STEPS()); //get the time at which next steps would occur.
                float posTime = testDir((this->sTotal+1)*MM_STEPS());
                if (negTime < this->time || std::isnan(negTime)) { //negTime is invalid
                    if (posTime > this->time) {
                        LOGV("LinearDeltaArcStepper<%u>::chose %f (pos) vs %f (neg)\n", AxisIdx, posTime, negTime);
                        this->time = posTime;
                        this->direction = StepForward;
                        ++this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else if (posTime < this->time || std::isnan(posTime)) { //posTime is invalid
                    if (negTime > this->time) {
                        LOGV("LinearDeltaArcStepper<%u>::chose %f (neg) vs %f (pos)\n", AxisIdx, negTime, posTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else { //neither time is invalid
                    if (negTime < posTime) {
                        LOGV("LinearDeltaArcStepper<%u>::chose %f (neg) vs %f (pos)\n", AxisIdx, negTime, posTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        LOGV("LinearDeltaArcStepper<%u>::chose %f (pos) vs %f (neg)\n", AxisIdx, posTime, negTime);
                        this->time = posTime;
                        this->direction = StepForward;
                        ++this->sTotal;
                    }
                }
            }
        }
};

template <typename StepperDriverT, DeltaAxis AxisIdx> class LinearDeltaStepper : public AxisStepperWithDriver<StepperDriverT> {
    private:
        const iodrv::Endstop *endstop; //must be pointer, because cannot move a reference
        float _r, _L, _MM_STEPS; //settings which will be obtained from the CoordMap
        float M0; //initial coordinate of THIS axis.
        int sTotal; //current step offset from M0
        float inv_v2; //1/v^2, where v is the linear speed in cartesian-space
        float vz_over_v2; //vz/v^2, where vz is the 
        float _almostTerm1; //used for caching & reducing computational complexity inside nextStep()
        float _almostRootParam;
        float _almostRootParamV2S;
        inline float r() const { return _r; }
        inline float L() const { return _L; }
        inline float MM_STEPS() const { return _MM_STEPS; }
    public:
        typedef LinearDeltaArcStepper<StepperDriverT, AxisIdx> ArcStepperT;
        inline LinearDeltaStepper() : endstop(nullptr), _r(0), _L(0), _MM_STEPS(0) {}
        template <typename CoordMapT, std::size_t sz> LinearDeltaStepper(int idx, const CoordMapT &map, const std::array<int, sz>& curPos, 
        const Vector4f &vel)
        : AxisStepperWithDriver<StepperDriverT>(idx, map.template getStepperDriver<AxisIdx>()),
             endstop(&map.getEndstop(AxisIdx)),
             _r(map.r()), _L(map.L()), _MM_STEPS(map.MM_STEPS(AxisIdx)),
             M0(map.getAxisPosition(curPos, AxisIdx)*map.MM_STEPS(AxisIdx)), 
             sTotal(0),
             //vx(vx), vy(vy), vz(vz),
             //v2(vx*vx + vy*vy + vz*vz), 
             inv_v2(1/(vel.xyz().magSq())),
             vz_over_v2(vel.z()*inv_v2) {
                static_assert(AxisIdx < 3, "LinearDeltaStepper only supports axis A, B, or C (0, 1, 2)");
                this->time = 0; //this may NOT be zero-initialized by parent.
                float vx, vy, vz;
                std::tie(vx, vy, vz) = vel.xyz().tuple();
                float x0, y0, z0;
                std::tie(x0, y0, z0) = map.xyzeFromMechanical(curPos).xyz().tuple();
                //precompute as much as possible:
                _almostRootParamV2S = 2*M0 - 2*z0;
                if (AxisIdx == DELTA_AXIS_A) {
                    _almostTerm1 = inv_v2*(r()*vy - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
                    //rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + (M0 + s - z0)*(M0 + s - z0));
                    //rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 + 2*M0*s - 2*M0*z0 + s*s - 2*s*z0 + z0*z0);
                    //rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 - 2*M0*z0 + z0*z0) - v2*s*(2*M0 + s - 2*z0);
                    _almostRootParam = -inv_v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 - 2*M0*z0 + z0*z0);
                    //_almostRootParamV2S = 2*M0 - 2*z0; // (...+s)*-1/v2*s
                } else if (AxisIdx == DELTA_AXIS_B) { 
                    _almostTerm1 = inv_v2*(r()*(sqrt(3)*vx - vy)/2. - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
                    //rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
                    _almostRootParam = -inv_v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + M0*M0 - 2*M0*z0 + z0*z0);
                    //_almostRootParamV2S = 2*M0 - 2*z0;
                } else if (AxisIdx == DELTA_AXIS_C) {
                    _almostTerm1 = inv_v2*(-r()*(sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
                    //rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
                    _almostRootParam = -inv_v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + M0*M0 - 2*M0*z0 + z0*z0);
                    //_almostRootParamV2S = 2*M0 - 2*z0;
                }
            }
        inline void getTerm1AndRootParam(float &term1, float &rootParam, float s) {
            //Therefore, we should cache values calculatable at init-time, like all of the second-half on rootParam.
            term1 = _almostTerm1 + vz_over_v2*s;
            rootParam = term1*term1 + _almostRootParam - inv_v2*s*(_almostRootParamV2S + s);
            /*if (AxisIdx == 0) {
                term1 = r()*vy - vx*x0 - vy*y0 + vz*(M0 + s - z0);
                rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + (M0 + s - z0)*(M0 + s - z0));
            } else if (AxisIdx == 1) { 
                term1 = r()*(sqrt(3)*vx - vy)/2. - vx*x0 - vy*y0 + vz*(M0 + s - z0);
                rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
            } else if (AxisIdx == 2) {
                term1 = -r()*(sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(M0 + s - z0);
                rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
            }*/
            /*float root = std::sqrt(rootParam);
            float t1 = (term1 - root)/v2;
            float t2 = (term1 + root)/v2;*/
        }
        inline float testDir(float s) {
            float term1, rootParam;
            getTerm1AndRootParam(term1, rootParam, s);
            if (rootParam < 0) {
                return NAN;
            }
            float root = std::sqrt(rootParam);
            float t1 = term1 - root;
            float t2 = term1 + root;
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
            //First, we test the time at which a forward step (sTotal + 1) should occur given constant cartesian velocity.
            //Then we test that time for a backward step (sTotal - 1).
            //We choose the nearest resulting time as our next step.
            //This is necessary because axis velocity can actually reverse direction during a linear cartesian movement.
            if (useEndstops && endstop->isTriggered()) {
                this->time = NAN; //at endstop; no more steps.
            } else {
                float negTime = testDir((this->sTotal-1)*MM_STEPS()); //get the time at which next steps would occur.
                float posTime = testDir((this->sTotal+1)*MM_STEPS());
                if (negTime < this->time || std::isnan(negTime)) { //negTime is invalid
                    if (posTime > this->time) {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (pos)\n", AxisIdx, posTime);
                        this->time = posTime;
                        this->direction = StepForward;
                        ++this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else if (posTime < this->time || std::isnan(posTime)) { //posTime is invalid
                    if (negTime > this->time) {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (neg)\n", AxisIdx, negTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        this->time = NAN;
                    }
                } else { //neither time is invalid
                    if (negTime < posTime) {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (neg)\n", AxisIdx, negTime);
                        this->time = negTime;
                        this->direction = StepBackward;
                        --this->sTotal;
                    } else {
                        //LOGV("LinearDeltaStepper<%u>::chose %f (pos)\n", AxisIdx, posTime);
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

