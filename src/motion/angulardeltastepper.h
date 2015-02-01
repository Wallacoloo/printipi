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
 * This is combined with the constraint equation and solved further down in the AngularDeltaStepper::testDir() function
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
 *   This is solved further down in the LinearDeltaArcStepper::testDir() function.  
 *
 * Note: all motion in this file is planned at a constant velocity. 
 *   Cartesian-space acceleration is introduced by a post-transformation of the step times applied elsewhere in the motion planning system. 
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
    
    float M0; //initial coordinate of THIS axis. CW from +y axis
    int sTotal; //current step offset from M0
    
    public:
        template <typename CoordMapT> AngularDeltaStepper(int idx, DeltaAxis axisIdx, const CoordMapT &map, const StepperDriverT &stepper, const iodrv::Endstop *endstop)
         : AxisStepperWithDriver<StepperDriverT>(idx, stepper),
           axisIdx(axisIdx),
           endstop(endstop)  {
              //E axis has to be controlled using a LinearStepper (as if it were cartesian)
              assert(axisIdx == ANGULARDELTA_AXIS_A || axisIdx == ANGULARDELTA_AXIS_B || axisIdx == ANGULARDELTA_AXIS_C);
        }

        //function to initiate a linear (through cartesian space) motion
        template <typename CoordMapT, std::size_t sz> void beginLine(const CoordMapT &map, const std::array<int, sz>& curPos, 
        const Vector4f &vel) {
            this->M0 = map.getAxisPosition(curPos, axisIdx)*map.DEGREES_STEP(axisIdx); 
            this->sTotal = 0;
            this->time = 0;
        }
        //function to initiate a circular arc (through cartesian space) motion
        template <typename CoordMapT, std::size_t sz> void beginArc(const CoordMapT &map, const std::array<int, sz> &curPos, 
        const Vector3f &center, const Vector3f &u, const Vector3f &v,  
        float arcRad, float arcVel, float extVel) {
            //arc motions not yet supported.
            assert(false);
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
                // TODO: calculate the time of the next step
            }
        }
};


}


#endif

