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
#include "testhelper.h"

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
	angle = mathutil::clamp(angle, minAngle, maxAngle);
	//calculate proportion, p, such that angle = angleMin + p*(angleMax-angleMin)
	float proportion = (angle-minAngle) / (maxAngle - minAngle);
	//This proportion now nicely maps to the linear scale, [minOnTime, maxOnTime]
	float fsec = std::chrono::duration<float>(minOnTime).count() + proportion*std::chrono::duration<float>(maxOnTime - minOnTime).count();
	return std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(fsec));
}

struct ServoTester {
	ServoTester() {
		GIVEN("A TestHelper & IoDrivers tuple") {
			//create the ioDrivers
	    	auto ioDrivers = std::make_tuple(
	    	    iodrv::Servo(iodrv::IoPin(iodrv::NO_INVERSIONS, iodrv::IoPin::null().primitiveIoPin()), 
				  			 std::chrono::milliseconds(100), //cycle length
				  			 std::make_pair(std::chrono::milliseconds(1), std::chrono::milliseconds(5)), //min/max duty cycle
				  			 std::make_pair(0, 360) //min/max angle
				),
				iodrv::IODriver(),
				iodrv::IODriver(),
				iodrv::Servo(iodrv::IoPin(iodrv::INVERT_WRITES, iodrv::IoPin::null().primitiveIoPin()), 
				  			 std::chrono::milliseconds(100), //cycle length
				  			 std::make_pair(std::chrono::milliseconds(2), std::chrono::milliseconds(4)), //min/max duty cycle
				  			 std::make_pair(90, 180) //min/max angle
				),
				iodrv::IODriver()
	    	);
			//only give the State references to the ioDrivers so that we can track changes without private member access
			auto getIoDrivers = [&]() {
				return std::tie(std::get<0>(ioDrivers));
			};
	    	auto helper = makeTestHelper(makeTestMachine(getIoDrivers));

	    	WHEN("Servo0 is set to 90 degrees") {
	    		helper.sendCommand("M280 P0 S90.0", "ok");
	    		THEN("Its highTime should be 2 us (25% interpolation of 1, 5)") {
		    		helper.requireTimesApproxEqual(std::get<0>(ioDrivers).highTime, std::chrono::milliseconds(2));
		    	}
	    	}
		}
	}
};

SCENARIO("Servo will behave correctly", "[servo]") {
    ServoTester();
}

}