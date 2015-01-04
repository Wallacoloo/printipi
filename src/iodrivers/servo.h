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
	EventClockT::duration minOnTime, maxOnTime;
	float minAngle, maxAngle;
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
		 : pin(std::move(pin)) , cycleLength(cycleLength), minOnTime(minMaxOnTime.first), maxOnTime(minMaxOnTime.second), 
		   minAngle(minMaxAngle.first), maxAngle(minMaxAngle.second),
		   lastEventTime(EventClockT::now()), highTime(getOnTime(initialAngle)), curState(false) {}

		inline bool isServo() const {
			return true;
		}
		void setServoAngleDegrees(float angle);
		//Used by State for event scheduling.
		//Queries information about the next time this Servo needs to toggle an IoPin
		OutputEvent peekNextEvent() const;
		//Used by State for event scheduling.
		//When the State calls this, it is an acknowledgement that the last event returned from peekNextEvent() has been scheduled.
		void consumeNextEvent();
	private:
		EventClockT::duration getOnTime(float angle);
};

}

#endif