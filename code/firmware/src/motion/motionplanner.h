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
	public:
		MotionPlanner() : _accel(), _destMechanicalPos(), _iters(), _homeIters(), _baseTime(), _duration(NAN), _maxVel(0), _motionType(MotionNone) {}
		/* isReadyForNextMove: returns true if a call to moveTo() or homeEndstops() wouldn't hang, false if it would hang (or cause other problems) */
		bool readyForNextMove() const {
			//Note: for now, there isn't actually buffering.
			return _motionType == MotionNone;
		}
		Event nextStep() {
			if (_motionType == MotionNone) {
				return Event();
			}
			bool isHoming = _motionType == MotionHome;
			drv::AxisStepper& s = isHoming ? drv::AxisStepper::getNextTime(_homeIters) : drv::AxisStepper::getNextTime(_iters);
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
			float transformedTime = _accel.transform(s.time, _duration, _maxVel);
			LOGV("Step transformed time: %f\n", transformedTime);
			Event e = s.getEvent(transformedTime);
			e.offset(_baseTime);
			_destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction);
			if (isHoming) {
				s.nextStep(_homeIters);
			} else {
				s.nextStep(_iters);
			}
			return e;
		}
		void moveTo(const timespec &baseTime, float x, float y, float z, float e, float maxVelXyz, float minVelE, float maxVelE) {
			this->_baseTime = baseTime;
			float curX, curY, curZ, curE;
			std::tie(curX, curY, curZ, curE) = CoordMapT::xyzeFromMechanical(_destMechanicalPos);
			
			float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
			float dist = sqrt(distSq);
			float minDuration = dist/maxVelXyz; //duration, should there be no acceleration
			float velE = (e-curE)/minDuration;
			//float newVelE = this->driver.clampExtrusionRate(velE);
			float newVelE = std::max(minVelE, std::min(maxVelE, velE));
			if (velE != newVelE) { //in the case that newXYZ = currentXYZ, but extrusion is different, regulate that.
				velE = newVelE;
				minDuration = (e-curE)/newVelE; //L/(L/t) = t
				maxVelXyz = dist/minDuration;
			}
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
			drv::AxisStepper::initAxisHomeSteppers(_homeIters, maxVelXyz);
			this->_baseTime = baseTime;
			this->_maxVel = maxVelXyz;
			this->_duration = NAN;
			this->_motionType = MotionHome;
		}
};


#endif
