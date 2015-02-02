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

#ifndef MOTION_ANGULARDELTACOORDMAP_H
#define MOTION_ANGULARDELTACOORDMAP_H

#include <array>
#include <tuple>
#include <utility> //for std::move
#include <tuple>

#include "coordmap.h"
#include "common/logging.h"
#include "common/matrix.h"
#include "angulardeltastepper.h"
#include "iodrivers/endstop.h"
#include "motion/motionplanner.h" //for motion::USE_ENDSTOPS

namespace motion {

/* 
 * AngularDeltaCoordMap implements the CoordMap interface for (rail-based) Delta-style robots like the Kossel
 * This class allows for translating mechanical positions to the cartesian x, y, z, e system.
 * It assumes there are 3 legs arranged in a circle (120 degrees between each adjacent pair)
 *   and these legs have carriages a distance d from their base.
 * The leg at (x=0, y=+) is axis A,
 * The leg at (x>0, y<0) is axis B,
 * The leg at (x<0, y<0) is axis C
 * Additionally, the carriages host a free-spinning joint with an arm of length L linked to an end effector,
 * and the carriages are r units apart.
 *
 * R1000 is 'r' (in mm) multiplied by 1000,
 * L1000 is 'L' (in mm) multiplied by 1000
 *
 * The math is described more in /code/proof-of-concept/coordmath.py and coord-math.nb (note: file has been deleted; must view an archived version of printipi on Github to view this documentation)
 */

template <typename Stepper1, typename Stepper2, typename Stepper3, typename Stepper4, typename BedLevelT=Matrix3x3> class AngularDeltaCoordMap : public CoordMap {
    typedef std::tuple<Stepper1, Stepper2, Stepper3, Stepper4> StepperDriverTypes;
    typedef std::tuple<AngularDeltaStepper<Stepper1>, 
                       AngularDeltaStepper<Stepper2>, 
                       AngularDeltaStepper<Stepper3>, 
                       LinearStepper<Stepper4> > _AxisStepperTypes;

    static constexpr float MIN_Z() { return -2; } //useful to be able to go a little under z=0 when tuning.
    float e, f, re, rf, _zoffset, _buildrad;
    float _STEPS_DEGREE, _DEGREES_STEP;
    float _STEPS_MM_EXT, _MM_STEPS_EXT;
    float homeVelocity;
    float homeAngle; //the angle at which the endstops are positioned
    BedLevelT bedLevel;

    std::array<iodrv::Endstop, 4> endstops; //A, B, C and E (E is null)
    StepperDriverTypes stepperDrivers;    
    private:
        //return number of (micro)steps it takes to rotate a motor by 1 degree.
        inline float STEPS_DEGREE() const { return _STEPS_DEGREE; }
        //Return the numner of degrees rotated by advancing a motor by 1 (micro)step
        inline float DEGREES_STEP() const { return _DEGREES_STEP; }
        inline float STEPS_MM_EXT() const { return _STEPS_MM_EXT; }
        inline float MM_STEPS_EXT() const { return _MM_STEPS_EXT; }
    public:
        //define these functions so our AxisSteppers can know a bit about their coordinate system
        inline float STEPS_DEGREE(std::size_t axisIdx) const { return _STEPS_DEGREE; }
        inline float DEGREES_STEP(std::size_t axisIdx) const { return _DEGREES_STEP; }
        //The following are only valid for the extruder:
        inline float STEPS_MM(std::size_t axisIdx)     const { assert(axisIdx==CARTESIAN_AXIS_E); return _STEPS_MM_EXT; }
        inline float MM_STEPS(std::size_t axisIdx)     const { assert(axisIdx==CARTESIAN_AXIS_E); return _MM_STEPS_EXT; }
        inline float buildrad() const { return _buildrad; }

        inline AngularDeltaCoordMap(
		float e,
		float f, 
		float re, 
        float rf,
        float zoffset, 
		float buildrad, 
		float STEPS_DEGREE, 
		float STEPS_MM_EXT, 
		float homeVelocity,
        float homeAngle,

            	Stepper1 &&stepper1, 
		Stepper2 &&stepper2, 
		Stepper3 &&stepper3, 
		Stepper4 &&stepper4,

            	iodrv::Endstop &&endstopA,
		iodrv::Endstop &&endstopB, 
		iodrv::Endstop &&endstopC, 
		const BedLevelT &t)

	        :
		e(e),
        f(f),
        re(re),
        rf(rf),
        _zoffset(zoffset),
		_buildrad(buildrad),
           	_STEPS_DEGREE(STEPS_DEGREE), _DEGREES_STEP(1. / STEPS_DEGREE),
           	_STEPS_MM_EXT(STEPS_MM_EXT), _MM_STEPS_EXT(1./ STEPS_MM_EXT),
           	homeVelocity(homeVelocity),
            homeAngle(homeAngle),
           	bedLevel(t),

           	endstops({{
			std::move(endstopA), 
			std::move(endstopB), 
			std::move(endstopC), 
			std::move(iodrv::Endstop())
			}}),

           	stepperDrivers(
			std::move(stepper1), 
			std::move(stepper2), 
			std::move(stepper3), 
			std::move(stepper4)) {}

	        inline std::tuple<
				Stepper1&, 
				Stepper2&, 
				Stepper3&, 
				Stepper4&,
	 
          	        iodrv::Endstop&, 
			iodrv::Endstop&,
			iodrv::Endstop&>
		
		 getDependentIoDrivers() {
		 return std::tie(
                std::get<0>(stepperDrivers), 
                std::get<1>(stepperDrivers), 
                std::get<2>(stepperDrivers), 
                std::get<3>(stepperDrivers),
                endstops[0],
                endstops[1],
                endstops[2]);
        }
        inline _AxisStepperTypes getAxisSteppers() const {
            return std::make_tuple(
                       AngularDeltaStepper<Stepper1>(0, ANGULARDELTA_AXIS_A, *this, std::get<0>(stepperDrivers), &endstops[0], e, f, re, rf, _zoffset), 
                       AngularDeltaStepper<Stepper2>(1, ANGULARDELTA_AXIS_B, *this, std::get<1>(stepperDrivers), &endstops[1], e, f, re, rf, _zoffset), 
                       AngularDeltaStepper<Stepper3>(2, ANGULARDELTA_AXIS_C, *this, std::get<2>(stepperDrivers), &endstops[2], e, f, re, rf, _zoffset), 
                       LinearStepper<Stepper4>      (3, CARTESIAN_AXIS_E,    *this, std::get<3>(stepperDrivers), &endstops[3])
            );
        }

        inline static constexpr std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        inline int getAxisPosition(const std::array<int, 4> &cur, std::size_t axis) const {
            return cur[axis];
        }
        inline const iodrv::Endstop& getEndstop(std::size_t axis) const {
            return endstops[axis];
        }
        template <std::size_t idx> auto getStepperDriver() const
         -> const typename std::tuple_element<idx, StepperDriverTypes>::type& {
            return std::get<idx>(stepperDrivers);
        }
        template <typename Interface> void executeHomeRoutine(Interface &interface) {
            //must disable buffering so that endstops can be reliably checked
            //interface.setUnbufferedMove(true);
            Vector4f curPos = interface.actualCartesianPosition();
            //try to move to <here> + <height> & will stop along the way if we hit an endstop.
            float veryHighZ = 1000; //just pick any value that will cause us to (try to) move past the endstop position
            Vector4f destPos = curPos + Vector4f(0, 0, veryHighZ, 0);
            interface.moveTo(destPos, homeVelocity, motion::USE_ENDSTOPS | motion::NO_LEVELING | motion::NO_BOUNDING);
            //reset the indexed axis positions:
            auto axesCoords = getHomePosition(interface.axisPositions());
            interface.resetAxisPositions(axesCoords);
            LOG("homed position: %s\n", xyzeFromMechanical(axesCoords).str().c_str());
            //interface.setUnbufferedMove(false);
        }
        inline std::array<int, 4> getHomePosition(const std::array<int, 4> &cur) const {
            //When we're at the home position, we know the angle of each axis.
            //Convert these angles into the microstep positions of the stepper motors and return that.
            //Extruder is unaffected by homing.
            return std::array<int, 4>({{(int)(homeAngle*STEPS_DEGREE()), (int)(homeAngle*STEPS_DEGREE()), (int)(homeAngle*STEPS_DEGREE()), cur[3]}});
        }
        inline Vector3f applyLeveling(const Vector3f &xyz) const {
            return bedLevel.transform(xyz);
        }
        inline Vector4f bound(const Vector4f &xyze) const {
            //implement this function later to make sure we don't try to move to any invalid coordinate.
            return xyze;
        }
    private:
        // Function taken from http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
        // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
        // returned status: 0=OK, -1=non-existing position
        void delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) const {
             // define useful constants
             float pi = M_PI;
             float tan30 = tan(30*pi/180.0);
             float sin30 = sin(30*pi/180.0);
             float tan60 = tan(60*pi/180.0);

             float t = (f-e)*tan30/2;
             float dtr = pi/(float)180.0;
         
             theta1 *= dtr;
             theta2 *= dtr;
             theta3 *= dtr;
         
             float y1 = -(t + rf*cos(theta1));
             float z1 = -rf*sin(theta1);
         
             float y2 = (t + rf*cos(theta2))*sin30;
             float x2 = y2*tan60;
             float z2 = -rf*sin(theta2);
         
             float y3 = (t + rf*cos(theta3))*sin30;
             float x3 = -y3*tan60;
             float z3 = -rf*sin(theta3);
         
             float dnm = (y2-y1)*x3-(y3-y1)*x2;
         
             float w1 = y1*y1 + z1*z1;
             float w2 = x2*x2 + y2*y2 + z2*z2;
             float w3 = x3*x3 + y3*y3 + z3*z3;
             
             // x = (a1*z + b1)/dnm
             float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
             float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
         
             // y = (a2*z + b2)/dnm;
             float a2 = -(z2-z1)*x3+(z3-z1)*x2;
             float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
         
             // a*z^2 + b*z + c = 0
             float a = a1*a1 + a2*a2 + dnm*dnm;
             float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
             float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
          
             // discriminant
             float d = b*b - (float)4.0*a*c;
             assert (d >= 0);
             //if (d < 0) return -1; // non-existing point
         
             z0 = -(float)0.5*(b+sqrt(d))/a;
             x0 = (a1*z0 + b1)/dnm;
             y0 = (a2*z0 + b2)/dnm;
             //return 0;
         }
         
    public:
         // Inside your AngularDeltaCoordMap, your forward kinematics should look like:
         Vector4f xyzeFromMechanical(const std::array<int, 4> &mech) const {
            // The "mech" coordinates given are the locations of each axis *in microsteps*.
            // So convert these into angles for the 3 towers,
            // and millimeters for the extruder:
            float theta1 =   mech[0] * DEGREES_STEP(); 
            float theta2 =   mech[1] * DEGREES_STEP();
            float theta3 =   mech[2] * DEGREES_STEP();
            float extruder = mech[3] * MM_STEPS_EXT();
            
            // Now you can use your previous forward kinematics code:
            float x0, y0, z0;
            delta_calcForward(theta1, theta2, theta3, x0, y0, z0);
            
            //Now return x0, y0, z0, extruder - all coordinates in millimeters:
            //must invert y0 because the trossen derivation uses a left-hand coordinate system.
            return Vector4f(x0, -y0, z0+_zoffset, extruder);
         }

};

}

#endif
