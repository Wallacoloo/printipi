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
#include "common/mathutil.h"

namespace iodrv {

//Controls a Servo motor.
//
//Servos are controlled by periodically sending a pulse of a specified length.
//The length of that pulse determines the position at which the servo should be placed, and the servo will attempt to stay at that location until the next command.
//Typical pulse length varies from 1ms to 2ms for the full control range,
// while the pulses must occur between 40-200 times per second.
class Servo : public IODriver {
	//constants:
	IoPin pin;
	EventClockT::duration cycleLength;
	std::pair<EventClockT::duration, EventClockT::duration> minMaxOnTime;
	std::pair<float, float> minMaxAngle;
	//internal state:
	EventClockT::time_point lastEventTime;
	EventClockT::duration highTime;
	bool curState;
	public:
		//Instantiate a Servo controlled by PWM on pin @pin
		//
		//A Servo operates using pwm of a fixed frequency, and adjusting only the duty cycle to control angle.
		//@cycleLength the length of one PWM cycle. Most servos can handle 40-200 Hz, so this should be anywhere between 5-25 ms.
		//@minMaxOnTime a pair of <minOnTime, maxOnTime> where minOnTime is the PWM duty cycle that would send the Servo to the lowest angle it can handle,
		//   and maxOnTime corresponds with the Servo's maximum angle.
		//@minMaxAngle a pair of <minAngle, maxAngle> where minAngle is the minimum angle the Servo can be set to, in degrees, and 
		//   maxAngle is the maximum angle the Servo can be set to, in degrees.
		//@initialAngle the angle the Servo should be set to at startup.
		inline Servo(IoPin &&pin, EventClockT::duration cycleLength, std::pair<EventClockT::duration, EventClockT::duration> minMaxOnTime,
			std::pair<float, float> minMaxAngle=std::pair<float, float>(0, 360), float initialAngle=0)
		 : pin(std::move(pin)), cycleLength(cycleLength), minMaxOnTime(minMaxOnTime), minMaxAngle(minMaxAngle),
		   lastEventTime(EventClockT::now()), highTime(getOnTime(initialAngle)), curState(false) {}

		inline bool isServo() const {
			return true;
		}
		inline void setServoAngleDegrees(float angle) {
			highTime = getOnTime(angle);
		}
		inline OutputEvent peekNextEvent() const {
			bool nextState = !curState;
			EventClockT::time_point nextEventTime = lastEventTime + (nextState ? highTime : (cycleLength-highTime));
			return OutputEvent(nextEventTime, pin, nextState);
		}
		inline void consumeNextEvent() {
			OutputEvent evt = peekNextEvent();
			this->curState = evt.state();
			this->lastEventTime = evt.time();
		}
	private:
		inline EventClockT::duration getOnTime(float angle) {
			//clamp the angle
			//angle = std::min(minMaxAngle.second, std::max(minMaxAngle.first, angle));
			angle = mathutil::clamp(angle, minMaxAngle.first, minMaxAngle.second);
			//calculate proportion, p, such that angle = angleMin + p*(angleMax-angleMin)
			float proportion = (angle-minMaxAngle.first) / (minMaxAngle.second - minMaxAngle.first);
			//This proportion now nicely maps to the linear scale, [minOnTime, maxOnTime]
			float fsec = std::chrono::duration<float>(minMaxOnTime.first).count() + proportion*std::chrono::duration<float>(minMaxOnTime.second - minMaxOnTime.first).count();
			return std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(fsec));
		}
};

}

#endif