#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include "accelerationprofile.h"
#include "drivers/axisstepper.h"

enum MotionType {
	MotionNone,
	MotionMove,
	MotionHome
};

template <typename Interface, typename AccelProfile=NoAcceleration> class MotionPlanner {
	private:
		typedef typename Interface::CoordMapT CoordMapT;
		typedef typename Interface::AxisStepperTypes AxisStepperTypes;
		AccelProfile _accel;
		std::array<int, CoordMapT::numAxis()> _destMechanicalPos;
		AxisStepperTypes _iters;
		typename drv::AxisStepper::GetHomeStepperTypes<AxisStepperTypes>::HomeStepperTypes _homeIters;
		timespec _baseTime;
		float _duration;
		float _maxVel;
		MotionType _motionType;
		/*template <typename AxisStepperTypes> void scheduleAxisSteppers(const timespec &baseTime, float duration, bool accelerate, float maxVel) {
			//Information on acceleration: http://reprap.org/wiki/Firmware/Linear_Acceleration
			//Current implementation uses instantaneous acceleration, which is physically impossible.
			//The best course *appears* to be an exponential velocity curve.
			//An attempt at that is made in proof-of-concept/acceleration.nb
			//if (Drv::CoordMapT::numAxis() == 0) { 
			//	return; //some of the following logic may assume that there are at least 1 axis.
			//}
			//timespec baseTime = scheduler.lastSchedTime();
			do {
				drv::AxisStepper& s = drv::AxisStepper::getNextTime(_iters);
				LOGV("Next step: %i at %g of %g\n", s.index(), s.time, duration);
				//if (s.time > duration || gmath::ltepsilon(s.time, 0, gmath::NANOSECOND)) { 
				if (s.time > duration || s.time <= 0 || std::isnan(s.time)) { //don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
					break; 
				}
				float transformedTime = accelerate ? transformEventTime(s.time, duration, maxVel) : s.time;
				LOGV("Step transformed time: %f\n", transformedTime);
				Event e = s.getEvent(transformedTime);
				e.offset(baseTime);
				//scheduler.queue(e);
				_destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction);
				s.nextStep(_iters);
			} while (1);
		}*/
	public:
		MotionPlanner() : _accel(), _destMechanicalPos(), _iters(), _homeIters(), _baseTime(), _duration(NAN), _maxVel(0), _motionType(MotionNone) {}
		Event nextStep() {
			if (_motionType == MotionNone) {
				return Event();
			}
			bool isHoming = _motionType == MotionHome;
			drv::AxisStepper& s = isHoming ? drv::AxisStepper::getNextTime(_homeIters) : drv::AxisStepper::getNextTime(_iters);
			//s = drv::AxisStepper::getNextTime(_iters);
			LOGV("MotionPlanner::nextStep() is: %i at %g of %g\n", s.index(), s.time, _duration);
			//if (s.time > duration || gmath::ltepsilon(s.time, 0, gmath::NANOSECOND)) { 
			if (s.time > _duration || s.time <= 0 || std::isnan(s.time)) { //don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
				//break; 
				if (isHoming) {
					_destMechanicalPos = CoordMapT::getHomePosition(_destMechanicalPos);
				}
				_motionType = MotionNone; //motion is over.
				return Event();
			}
			//float transformedTime = accelerate ? transformEventTime(s.time, duration, maxVel) : s.time;
			//float transformedTime = s.time;
			float transformedTime = _accel.transform(s.time, _duration, _maxVel);
			LOGV("Step transformed time: %f\n", transformedTime);
			Event e = s.getEvent(transformedTime);
			e.offset(_baseTime);
			//scheduler.queue(e);
			_destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction);
			if (isHoming) {
				s.nextStep(_homeIters);
			} else {
				s.nextStep(_iters);
			}
			return e;
		}
		void moveTo(const timespec &baseTime, float x, float y, float z, float e, float maxVelXyz) {
			this->_baseTime = baseTime;
			float curX, curY, curZ, curE;
			std::tie(curX, curY, curZ, curE) = CoordMapT::xyzeFromMechanical(_destMechanicalPos);
			//_destXPrimitive = x;
			//_destYPrimitive = y;
			//_destZPrimitive = z;
			//_destEPrimitive = e;
			//float maxVelXyz = destMoveRatePrimitive(); //the maximum velocity this path will be coded for.
			float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
			float dist = sqrt(distSq);
			float minDuration = dist/maxVelXyz; //duration, should there be no acceleration
			float velE = (e-curE)/minDuration;
			//float newVelE = this->driver.clampExtrusionRate(velE);
			/*float newVelE = velE;
			if (velE != newVelE) { //in the case that newXYZ = currentXYZ, but extrusion is different, regulate that.
				velE = newVelE;
				minDuration = (e-curE)/newVelE; //L/(L/t) = t
				maxVelXyz = dist/minDuration;
			}*/
			float vx = (x-curX)/minDuration;
			float vy = (y-curY)/minDuration;
			float vz = (z-curZ)/minDuration;
			LOGD("MotionPlanner::moveTo (%f, %f, %f, %f) -> (%f, %f, %f, %f)\n", curX, curY, curZ, curE, x, y, z, e);
			LOGD("MotionPlanner::moveTo _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
			LOGD("MotionPlanner::moveTo V:%f, vx:%f, vy:%f, vz:%f, ve:%f dur:%f\n", maxVelXyz, vx, vy, vz, velE, minDuration);
			drv::AxisStepper::initAxisSteppers(_iters, _destMechanicalPos, vx, vy, vz, velE);
			this->_maxVel = maxVelXyz;
			this->_duration = minDuration;
			this->_motionType = MotionMove;
			//this->scheduleAxisSteppers(baseTime, _iters, minDuration, true, maxVelXyz);
			//std::tie(curX, curY, curZ, curE) = Drv::CoordMapT::xyzeFromMechanical(_destMechanicalPos);
			//LOGD("MotionPlanner::moveTo wanted (%f, %f, %f, %f) got (%f, %f, %f, %f)\n", x, y, z, e, curX, curY, curZ, curE);
			//LOGD("MotionPlanner::moveTo _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
		}

		void homeEndstops(const timespec &baseTime, float maxVelXyz) {
			//typename Drv::AxisStepperTypes iters;
			//typename drv::AxisStepper::GetHomeStepperTypes<typename Drv::AxisStepperTypes>::HomeStepperTypes iters;
			//drv::AxisStepper::initAxisHomeSteppers(iters, this->driver.clampHomeRate(destMoveRatePrimitive()));
			this->_baseTime = baseTime;
			this->_maxVel = maxVelXyz;
			//drv::AxisStepper::initAxisHomeSteppers(_homeIters, this->driver.clampHomeRate(maxVelXyz));
			drv::AxisStepper::initAxisHomeSteppers(_homeIters, maxVelXyz);
			//auto b = this->scheduler.getBufferSize();
			//this->scheduler.setBufferSize(this->scheduler.numActivePwmChannels()+1); //todo: what happens when another PWM channel is enabled during scheduling?
			this->_duration = NAN;
			this->_motionType = MotionHome;
			//this->scheduleAxisSteppers(baseTime, _homeIters, NAN, false);
			//this->scheduler.setBufferSize(b);
			//_destMechanicalPos = Drv::CoordMapT::getHomePosition(_destMechanicalPos);
		}
};


#endif
