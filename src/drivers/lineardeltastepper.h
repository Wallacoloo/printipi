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
 * Printipi/drivers/lineardeltastepper.h
 * 
 * LinearDeltaStepper implements the AxisStepper interface for (rail-based) Delta-style robots like the Kossel
 */

/* Raspberry Pi float performance can be found here: http://www.raspberrypi.org/forums/viewtopic.php?t=7336
  float +,-,*: 2 cycles
  float /: 32 cycles (same for doubles)
  float sqrt: 48 cycles (same for doubles)
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
 * The above quadratic solution is used in LinearDeltaStepper::_nextStep(), although it has been optimized.
 *
 * Note: all motion in this file is planned at a constant velocity. Cartesian-space acceleration is introduced by a post-transformation of the step times applied elsewhere in the motion planning system
 *
 *
 *
 *
 *
 *If we want to move in an ARC along x,y at a constant velocity (acceleration will be introduced later):
 *   let x(t) = x0 + q cos(u*t)
 *       y(t) = y0 + q sin(u*t)
 *       z(t) = z0
 *   or, p(t) = <x0, y0, z0> + <qcos(u*t), qsin(u*t), 0>
 *   Then, in order to apply a phase to the rotation (i.e., not start at (x0+q, y0)), we just apply a rotation matrix (based about the z-axis) to the second parameter:
 *   p(t) = <x0, y0, z0> + Rz <qcos(u*t), qsin(u*t), 0>
 *   In order to support 3d rotations, we add Rx and Ry matrices as well. Thus,
 *   p(t) = <x0, y0, z0> + Rx Ry Rz q<cos(u*t), sin(u*t), 0>
 *
 *   Note: Rz = [1     0     0
 *               0     cosa  -sina
 *               0     sina  cosa]
 *         Ry = [cosb  0     sinb
 *               0     1     0
 *               -sinb 0     cosb]
 *         Rx = [cosc  -sinc 0
 *               sinc  cosc  0
 *               0     0     1]
 *
 * Mathematica Notation:
 *   R = {{1,0,0},{0,Cos[a],-Sin[a]},{0,Sin[a],Cos[a]}} . {{Cos[b],0, Sin[b]},{0,1,0},{-Sin[b],0,Cos[b]}} . {{Cos[c],-Sin[c],0},{Sin[c],Cos[c],0},{0,0,1}} = {{Cos[b] Cos[c], -(Cos[b] Sin[c]), Sin[b]}, {Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c], Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c], -(Cos[b] Sin[a])}, {-(Cos[a] Cos[c] Sin[b]) + Sin[a] Sin[c], Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c], Cos[a] Cos[b]}}
 * Then, p(t) = {{x0}, {y0}, {z0}} + q{{Cos[b] Cos[c] Cos[t u] - Cos[b] Sin[c] Sin[t u]}, 
 {Cos[t u] (Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c]) + (Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) Sin[t u]},
 {Cos[t u] (-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) + (Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) Sin[t u]}}
 *   
 * Then we substitute this information into the above derived D = z + sqrt(L^2 - (y-rcos(w))^2 - (x-rsin(w))^2), where w is tower angle and D is some axis coordinate.
 *   D(t) = q(Cos[t u] (-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) + (Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) Sin[t u])
      + Sqrt[L^2 - (q(Cos[t u] (Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c]) + (Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) Sin[t u]) - r Cos[w])^2 - (q(Cos[b] Cos[c] Cos[t u] - Cos[b] Sin[c] Sin[t u]) - r Sin[w])^2]
 * Solve for D(t) = D0 + s:
 *   D0 + s = q(Cos[t u] (-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) + (Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) Sin[t u])
      + Sqrt[L^2 - (q(Cos[t u] (Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c]) + (Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) Sin[t u]) - r Cos[w])^2 - (q(Cos[b] Cos[c] Cos[t u] - Cos[b] Sin[c] Sin[t u]) - r Sin[w])^2]
 *
 *   (D0 + s - q*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) - q*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]))^2 = L^2 - (q(Cos[t u] (Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c]) + (Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) Sin[t u]) - r Cos[w])^2 - (q(Cos[b] Cos[c] Cos[t u] - Cos[b] Sin[c] Sin[t u]) - r Sin[w])^2
 *
 * (D0 + s)^2 - 2(D0 + s)q*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) - 2(D0 + s)q*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + 2q^2*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + q^2*Cos^2[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c])^2 = L^2 - (q*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c]) + q*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) - r*Cos[w])^2 - (q*Cos[t u]Cos[b]Cos[c] + q*Sin[t u]Cos[b]Sin[c] - r*Sin[w])^2
 *
 * (D0 + s)^2 - 2(D0 + s)q*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) - 2(D0 + s)q*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + 2q^2*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + q^2*Cos^2[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c])^2 = L^2 - (2q^2*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) - 2*q*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c])*r*Cos[w] - 2*q*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])*r*Cos[w] + q^2*Cos^2[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c])^2 + r^2*Cos^2[w])
 *
 * (D0 + s)^2 - 2(D0 + s)q*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) - 2(D0 + s)q*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + 2q^2*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + q^2*Cos^2[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c])^2 + 2q^2*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) - 2*q*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c])*r*Cos[w] - 2*q*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])*r*Cos[w] + q^2*Cos^2[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c])^2 + r^2*Cos^2[w] = L^2
 *
 * 0 = (D0 + s)^2 - L^2 - 2(D0 + s)q*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c]) - 2(D0 + s)q*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + 2q^2*Cos[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])*Sin[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c]) + q^2*Cos^2[t u](-Cos[a] Cos[c] Sin[b] + Sin[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[c] Sin[a] + Cos[a] Sin[b] Sin[c])^2 + 2q^2*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c]) - 2*q*Sin[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c])*r*Cos[w] - 2*q*Cos[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])*r*Cos[w] + q^2*Cos^2[t u](Cos[c] Sin[a] Sin[b] + Cos[a] Sin[c])^2 + q^2*Sin^2[t u](Cos[a] Cos[c] - Sin[a] Sin[b] Sin[c])^2 + r^2*Cos^2[w]
 *
 * FullExpand from far above gives: 
 * 0 = -D0^2+L^2-2 D0 s-s^2-q^2 Cos[b]^2 Cos[c]^2 Cos[t u]^2-r^2 Cos[w]^2-2 D0 q Cos[a] Cos[c] Cos[t u] Sin[b]-2 q s Cos[a] Cos[c] Cos[t u] Sin[b]+2 q r Cos[c] Cos[t u] Cos[w] Sin[a] Sin[b]-q^2 Cos[a]^2 Cos[c]^2 Cos[t u]^2 Sin[b]^2-q^2 Cos[c]^2 Cos[t u]^2 Sin[a]^2 Sin[b]^2+2 q r Cos[a] Cos[t u] Cos[w] Sin[c]+2 D0 q Cos[t u] Sin[a] Sin[c]+2 q s Cos[t u] Sin[a] Sin[c]-q^2 Cos[a]^2 Cos[t u]^2 Sin[c]^2-q^2 Cos[t u]^2 Sin[a]^2 Sin[c]^2+2 q r Cos[a] Cos[c] Cos[w] Sin[t u]+2 D0 q Cos[c] Sin[a] Sin[t u]+2 q s Cos[c] Sin[a] Sin[t u]-2 q^2 Cos[a]^2 Cos[c] Cos[t u] Sin[c] Sin[t u]+2 q^2 Cos[b]^2 Cos[c] Cos[t u] Sin[c] Sin[t u]-2 q^2 Cos[c] Cos[t u] Sin[a]^2 Sin[c] Sin[t u]+2 D0 q Cos[a] Sin[b] Sin[c] Sin[t u]+2 q s Cos[a] Sin[b] Sin[c] Sin[t u]-2 q r Cos[w] Sin[a] Sin[b] Sin[c] Sin[t u]+2 q^2 Cos[a]^2 Cos[c] Cos[t u] Sin[b]^2 Sin[c] Sin[t u]+2 q^2 Cos[c] Cos[t u] Sin[a]^2 Sin[b]^2 Sin[c] Sin[t u]-q^2 Cos[a]^2 Cos[c]^2 Sin[t u]^2-q^2 Cos[c]^2 Sin[a]^2 Sin[t u]^2-q^2 Cos[b]^2 Sin[c]^2 Sin[t u]^2-q^2 Cos[a]^2 Sin[b]^2 Sin[c]^2 Sin[t u]^2-q^2 Sin[a]^2 Sin[b]^2 Sin[c]^2 Sin[t u]^2+2 q r Cos[b] Cos[c] Cos[t u] Sin[w]-2 q r Cos[b] Sin[c] Sin[t u] Sin[w]-r^2 Sin[w]^2
 *
 */


#ifndef DRIVERS_LINEARDELTASTEPPER_H
#define DRIVERS_LINEARDELTASTEPPER_H

#include "axisstepper.h"
#include "linearstepper.h" //for LinearHomeStepper
#include "endstop.h"

namespace drv {

template <std::size_t AxisIdx, typename CoordMap, unsigned R1000, unsigned L1000, unsigned STEPS_M, typename EndstopT=EndstopNoExist> class LinearDeltaStepper : public AxisStepper {
    private:
        float M0; //initial coordinate of THIS axis.
        int sTotal; //current step offset from M0
        float inv_v2; //1/v^2, where v is the linear speed in cartesian-space
        float vz_over_v2; //vz/v^2, where vz is the 
        float _almostTerm1; //used for caching & reducing computational complexity inside nextStep()
        float _almostRootParam;
        float _almostRootParamV2S;
        static constexpr float r() { return R1000 / 1000.; } //distance from center of build-plate to each axis
        static constexpr float L() { return L1000 / 1000.; } //length of rods connecting the axis carriages to the effector
        static constexpr float STEPS_MM() { return STEPS_M / 1000.; } //# of steps to turn an axis stepper in order to elevate the carriage by 1 mm
        static constexpr float MM_STEPS() { return  1. / STEPS_MM(); }
    public:
        typedef LinearHomeStepper<STEPS_M, EndstopT> HomeStepperT;
        LinearDeltaStepper() {}
        template <std::size_t sz> LinearDeltaStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
            : AxisStepper(idx, curPos, vx, vy, vz, ve),
             M0(curPos[AxisIdx]*MM_STEPS()), 
             sTotal(0),
             //vx(vx), vy(vy), vz(vz),
             //v2(vx*vx + vy*vy + vz*vz), 
             inv_v2(1/(vx*vx + vy*vy + vz*vz)),
             vz_over_v2(vz*inv_v2) {
                static_assert(AxisIdx < 3, "LinearDeltaStepper only supports axis A, B, or C (0, 1, 2)");
                this->time = 0; //this may NOT be zero-initialized by parent.
                float x0, y0, z0, e_;
                //CoordMap::xyzeFromMechanical(curPos, this->x0, this->y0, this->z0, e_);
                std::tie(x0, y0, z0, e_) = CoordMap::xyzeFromMechanical(curPos);
                //precompute as much as possible:
                _almostRootParamV2S = 2*M0 - 2*z0;
                if (AxisIdx == 0) {
                    _almostTerm1 = inv_v2*(r()*vy - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
                    //rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + (M0 + s - z0)*(M0 + s - z0));
                    //rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 + 2*M0*s - 2*M0*z0 + s*s - 2*s*z0 + z0*z0);
                    //rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 - 2*M0*z0 + z0*z0) - v2*s*(2*M0 + s - 2*z0);
                    _almostRootParam = -inv_v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 - 2*M0*z0 + z0*z0);
                    //_almostRootParamV2S = 2*M0 - 2*z0; // (...+s)*-1/v2*s
                } else if (AxisIdx == 1) { 
                    _almostTerm1 = inv_v2*(r()*(sqrt(3)*vx - vy)/2. - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
                    //rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
                    _almostRootParam = -inv_v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + M0*M0 - 2*M0*z0 + z0*z0);
                    //_almostRootParamV2S = 2*M0 - 2*z0;
                } else if (AxisIdx == 2) {
                    _almostTerm1 = inv_v2*(-r()*(sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
                    //rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
                    _almostRootParam = -inv_v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + M0*M0 - 2*M0*z0 + z0*z0);
                    //_almostRootParamV2S = 2*M0 - 2*z0;
                }
            }
        void getTerm1AndRootParam(float &term1, float &rootParam, float s) {
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
        float testDir(float s) {
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
        void _nextStep() {
            //called to set this->time and this->direction; the time (in seconds) and the direction at which the next step should occur for this axis
            //General formula is outlined in comments at the top of this file.
            //First, we test the time at which a forward step (sTotal + 1) should occur given constant cartesian velocity.
            //Then we test that time for a backward step (sTotal - 1).
            //We choose the nearest resulting time as our next step.
            //This is necessary because axis velocity can actually reverse direction during a linear cartesian movement.
            float negTime = testDir((this->sTotal-1)*MM_STEPS()); //get the time at which next steps would occur.
            float posTime = testDir((this->sTotal+1)*MM_STEPS());
            if (negTime < this->time || std::isnan(negTime)) { //negTime is invalid
                if (posTime > this->time) {
                    //LOGV("LinearDeltaStepper<%zu>::chose %f (pos)\n", AxisIdx, posTime);
                    this->time = posTime;
                    this->direction = StepForward;
                    ++this->sTotal;
                } else {
                    this->time = NAN;
                }
            } else if (posTime < this->time || std::isnan(posTime)) { //posTime is invalid
                if (negTime > this->time) {
                    //LOGV("LinearDeltaStepper<%zu>::chose %f (neg)\n", AxisIdx, negTime);
                    this->time = negTime;
                    this->direction = StepBackward;
                    --this->sTotal;
                } else {
                    this->time = NAN;
                }
            } else { //neither time is invalid
                if (negTime < posTime) {
                    //LOGV("LinearDeltaStepper<%zu>::chose %f (neg)\n", AxisIdx, negTime);
                    this->time = negTime;
                    this->direction = StepBackward;
                    --this->sTotal;
                } else {
                    //LOGV("LinearDeltaStepper<%zu>::chose %f (pos)\n", AxisIdx, posTime);
                    this->time = posTime;
                    this->direction = StepForward;
                    ++this->sTotal;
                }
            }
        }
};

}


#endif

