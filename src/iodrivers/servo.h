#ifndef IODRIVERS_SERVO_H
#define IODRIVERS_SERVO_H

//for std::move
#include <utility>
#include <chrono>

#include "iodriver.h"
#include "iopin.h"
#include "outputevent.h"
#include "platforms/auto/chronoclock.h"
#include "common/logging.h"

namespace iodrv {

//Controls a Servo motor.
//
//Servos are controlled by periodically sending a pulse of a specified length.
//The length of that pulse determines the position at which the servo should be placed, and the servo will attempt to stay at that location until the next command.
//Typical pulse length varies from 1ms to 2ms for the full control range,
// while the pulses must occur between 40-200 times per second.
//
//Not yet ready for use; see discussion @ https://github.com/Wallacoloo/printipi/issues/62
class Servo : public IODriver {
	//constants:
	IoPin pin;
	EventClockT::duration cycleLength;
	std::pair<EventClockT::duration, EventClockT::duration> minMaxOnTime;
	std::pair<float, float> minMaxAngle;
	//state:
	EventClockT::time_point lastEventTime;
	EventClockT::duration highTime;
	bool curState;
	public:
		Servo(IoPin &&pin, EventClockT::duration cycleLength, std::pair<EventClockT::duration, EventClockT::duration> minMaxOnTime,
			std::pair<float, float> minMaxAngle, float initialAngle=0)
		 : pin(std::move(pin)), cycleLength(cycleLength), minMaxOnTime(minMaxOnTime), minMaxAngle(minMaxAngle),
		   lastEventTime(EventClockT::now()), highTime(getOnTime(initialAngle)), curState(false) {}
		inline bool isServo() const {
			return true;
		}
		inline void setServoAngle(float angle) {
			highTime = getOnTime(angle);
		}
		inline OutputEvent peekNextEvent() const {
			bool nextState = !curState;
			EventClockT::time_point nextEventTime = lastEventTime + (nextState ? highTime : (cycleLength-highTime));
			return OutputEvent(nextEventTime, pin, nextState);
		}
		inline void consumeNextEvent() {
			OutputEvent evt = peekNextEvent();
			curState = evt.state();
			lastEventTime = evt.time();
		}
	private:
		inline EventClockT::duration getOnTime(float angle) {
			//clamp the angle
			angle = std::min(minMaxAngle.second, std::max(minMaxAngle.first, angle));
			//calculate proportion, p, such that angle = angleMin + p*(angleMax-angleMin)
			float proportion = (angle-minMaxAngle.first) / (minMaxAngle.second - minMaxAngle.first);
			//This proportion now nicely maps to the linear scale, [minOnTime, maxOnTime]
			float fsec = std::chrono::duration<float>(minMaxOnTime.first).count() + proportion*std::chrono::duration<float>(minMaxOnTime.second - minMaxOnTime.first).count();
			return std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(fsec));
		}
};

}

#endif