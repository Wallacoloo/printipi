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

#include "servo.h"

namespace iodrv {

void Servo::setServoAngleDegrees(float angle) {
	highTime = getOnTime(angle);
}

OutputEvent Servo::peekNextEvent() const {
	bool nextState = !curState;
	EventClockT::time_point nextEventTime = lastEventTime + (nextState ? highTime : (cycleLength-highTime));
	return OutputEvent(nextEventTime, pin, nextState);
}

void Servo::consumeNextEvent() {
	OutputEvent evt = peekNextEvent();
	this->curState = evt.state();
	this->lastEventTime = evt.time();
}

EventClockT::duration Servo::getOnTime(float angle) {
	//clamp the angle
	//angle = std::min(minMaxAngle.second, std::max(minMaxAngle.first, angle));
	angle = mathutil::clamp(angle, minMaxAngle.first, minMaxAngle.second);
	//calculate proportion, p, such that angle = angleMin + p*(angleMax-angleMin)
	float proportion = (angle-minMaxAngle.first) / (minMaxAngle.second - minMaxAngle.first);
	//This proportion now nicely maps to the linear scale, [minOnTime, maxOnTime]
	float fsec = std::chrono::duration<float>(minMaxOnTime.first).count() + proportion*std::chrono::duration<float>(minMaxOnTime.second - minMaxOnTime.first).count();
	return std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(fsec));
}

}