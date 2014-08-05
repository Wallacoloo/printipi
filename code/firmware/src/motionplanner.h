#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include "drivers/axisstepper.h"

template <typename Drv> class MotionPlanner {
	private:
		std::array<int, Drv::CoordMapT::numAxis()> _destMechanicalPos;
		typename Drv::AxisStepperTypes _iters;
		typename drv::AxisStepper::GetHomeStepperTypes<typename Drv::AxisStepperTypes>::HomeStepperTypes _homeIters;
		timespec _baseTime;
		float _duration;
		bool _isHoming;
		float transformEventTime(float time, float moveDuration, float Vmax) {
			//Note: it is assumed that the original path is already coded for constant velocity = Vmax.
			return time;
			/*float Amax = this->driver.maxAccel();
			float V0 = std::min(0.5*Vmax, 0.1); //c becomes invalid if V0 >= Vmax
			float k = 4*Amax/Vmax;
			float c = V0 / (Vmax-V0);
			if (time > 0.5*moveDuration) {
				return 2*transformEventTime(0.5*moveDuration, moveDuration, Vmax) - transformEventTime(moveDuration-time, moveDuration, Vmax);
			} else { //take advantage of the fact that comparisons against NaN always compare false to allow for indefinite movements:
				//the problem with the below equation is that it can return infinity if k/Vmax*time is sufficiently large.
				//return 1./k * log(1./c * ((1. + c)*exp(k/Vmax*time) - 1.));
				//aka: 1./k*( log(1./c) + log((1. + c)*exp(k/Vmax*time) - 1.))
				//simplify: 1./k*log(e^x-1) ~=~ 1./k*x at x = Log[1 - E^(-k*.001)], at which point it is only .001 off (however, 1ms is significant! Would rather use a smaller value.
				auto logparam = (1. + c)*exp(k*time) - 1;
				if (std::isfinite(logparam)) {
					return 1./k*( log(1./c) + log(logparam));
				} else { //use the approximation:
					return 1./k*(log(1./c) + log(1. + c) + k*time);
				}
			}*/
		}
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
		Event nextStep() {
			drv::AxisStepper& s = _isHoming ? drv::AxisStepper::getNextTime(_homeIters) : drv::AxisStepper::getNextTime(_iters);
			//s = drv::AxisStepper::getNextTime(_iters);
			LOGV("Next step: %i at %g of %g\n", s.index(), s.time, _duration);
			//if (s.time > duration || gmath::ltepsilon(s.time, 0, gmath::NANOSECOND)) { 
			if (s.time > _duration || s.time <= 0 || std::isnan(s.time)) { //don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
				//break; 
				if (_isHoming) {
					_destMechanicalPos = Drv::CoordMapT::getHomePosition(_destMechanicalPos);
				}
				return Event();
			}
			//float transformedTime = accelerate ? transformEventTime(s.time, duration, maxVel) : s.time;
			float transformedTime = s.time;
			LOGV("Step transformed time: %f\n", transformedTime);
			Event e = s.getEvent(transformedTime);
			e.offset(_baseTime);
			//scheduler.queue(e);
			_destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction);
			if (_isHoming) {
				s.nextStep(_homeIters);
			} else {
				s.nextStep(_iters);
			}
			return e;
		}
		void moveTo(const timespec &baseTime, float x, float y, float z, float e, float maxVelXyz) {
			_baseTime = baseTime;
			float curX, curY, curZ, curE;
			std::tie(curX, curY, curZ, curE) = Drv::CoordMapT::xyzeFromMechanical(_destMechanicalPos);
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
			float newVelE = velE;
			if (velE != newVelE) { //in the case that newXYZ = currentXYZ, but extrusion is different, regulate that.
				velE = newVelE;
				minDuration = (e-curE)/newVelE; //L/(L/t) = t
				maxVelXyz = dist/minDuration;
			}
			float vx = (x-curX)/minDuration;
			float vy = (y-curY)/minDuration;
			float vz = (z-curZ)/minDuration;
			LOGD("State::queueMovement (%f, %f, %f, %f) -> (%f, %f, %f, %f)\n", curX, curY, curZ, curE, x, y, z, e);
			LOGD("State::queueMovement _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
			LOGD("State::queueMovement V:%f, vx:%f, vy:%f, vz:%f, ve:%f dur:%f\n", maxVelXyz, vx, vy, vz, velE, minDuration);
			//typename Drv::AxisStepperTypes iters;
			drv::AxisStepper::initAxisSteppers(_iters, _destMechanicalPos, vx, vy, vz, velE);
			this->_duration = minDuration;
			this->_isHoming = false;
			//this->scheduleAxisSteppers(baseTime, _iters, minDuration, true, maxVelXyz);
			//std::tie(curX, curY, curZ, curE) = Drv::CoordMapT::xyzeFromMechanical(_destMechanicalPos);
			//LOGD("State::queueMovement wanted (%f, %f, %f, %f) got (%f, %f, %f, %f)\n", x, y, z, e, curX, curY, curZ, curE);
			//LOGD("State::queueMovement _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
		}

		void homeEndstops(const timespec &baseTime, float maxVelXyz) {
			//typename Drv::AxisStepperTypes iters;
			//typename drv::AxisStepper::GetHomeStepperTypes<typename Drv::AxisStepperTypes>::HomeStepperTypes iters;
			//drv::AxisStepper::initAxisHomeSteppers(iters, this->driver.clampHomeRate(destMoveRatePrimitive()));
			_baseTime = baseTime;
			//drv::AxisStepper::initAxisHomeSteppers(_homeIters, this->driver.clampHomeRate(maxVelXyz));
			drv::AxisStepper::initAxisHomeSteppers(_homeIters, maxVelXyz);
			//auto b = this->scheduler.getBufferSize();
			//this->scheduler.setBufferSize(this->scheduler.numActivePwmChannels()+1); //todo: what happens when another PWM channel is enabled during scheduling?
			this->_duration = NAN;
			this->_isHoming = true;
			//this->scheduleAxisSteppers(baseTime, _homeIters, NAN, false);
			//this->scheduler.setBufferSize(b);
			//_destMechanicalPos = Drv::CoordMapT::getHomePosition(_destMechanicalPos);
		}
};


#endif
