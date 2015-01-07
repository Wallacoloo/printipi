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
	EventClockT::time_point nextEventTime = lastEventTime + (curState ? highTime : (cycleLength-highTime));
	return OutputEvent(nextEventTime, pin, nextState);
}

void Servo::consumeNextEvent() {
	OutputEvent evt = peekNextEvent();
	this->curState = !curState; //Note: avoid evt.state(), as that has potentially been level-inverted for the pin inversions
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
		GIVEN("A tuple of Servos and IoDrivers") {
			//create the ioDrivers
	    	auto ioDrivers = std::make_tuple(
	    	    iodrv::Servo(iodrv::IoPin(iodrv::NO_INVERSIONS, iodrv::IoPin::null().primitiveIoPin()), 
				  			 std::chrono::milliseconds(100), //cycle length
				  			 std::make_pair(std::chrono::milliseconds(1), std::chrono::milliseconds(5)), //min/max duty cycle
				  			 std::make_pair(0, 360), //min/max angle
				  			 0 //default angle
				),
				iodrv::IODriver(),
				iodrv::IODriver(),
				iodrv::Servo(iodrv::IoPin(iodrv::INVERT_WRITES, iodrv::IoPin::null().primitiveIoPin()), 
				  			 std::chrono::milliseconds(100), //cycle length
				  			 std::make_pair(std::chrono::milliseconds(2), std::chrono::milliseconds(3)), //min/max duty cycle
				  			 std::make_pair(90, 180), //min/max angle
				  			 180 //default angle
				),
				iodrv::IODriver()
	    	);
	    	iodrv::Servo &servo0 = std::get<0>(ioDrivers);
	    	iodrv::Servo &servo1 = std::get<3>(ioDrivers);
    		THEN("Two consecutive calls to peekNextEvent should return the same event") {
    			REQUIRE(servo0.peekNextEvent() == servo0.peekNextEvent());
    		}
    		THEN("consecutive consumeNextEvent calls should have appropriate spacing") {
    			//advance such that the next call to peekNextEvent() will return the OFF state.
    			//avoid (servo0.peekNextEvent().state() == IoHigh) due to erroneous valgrind warning
    			if (servo0.curState == IoLow) {
    				servo0.consumeNextEvent();
    			}
    			OutputEvent prevLow = servo0.peekNextEvent();
    			servo0.consumeNextEvent();
    			//now at the ON state.
    			//test 3 iterations
    			for (int i=0; i<3; ++i) {
    				OutputEvent curHigh = servo0.peekNextEvent();
    				servo0.consumeNextEvent();
    				OutputEvent curLow = servo0.peekNextEvent();
    				servo0.consumeNextEvent();
    				//Require pin state to be alternating.
    				REQUIRE(curHigh.state() == IoHigh);
    				REQUIRE(curLow.state() == IoLow);
    				//configured for 1 ms on, 99 ms off.
    				TestHelper<>::requireTimesApproxEqual(curHigh.time(), prevLow.time() + std::chrono::milliseconds(99));
    				TestHelper<>::requireTimesApproxEqual(curLow.time(), curHigh.time() + std::chrono::milliseconds(1));
    				prevLow = curLow;
    			}
    		}
	    	GIVEN("A TestHelper that owns references to those IoDrivers") {
				//only give the State references to the ioDrivers so that we can track changes without private member access
				auto getIoDrivers = [&]() {
					return std::tie(std::get<0>(ioDrivers), 
									std::get<1>(ioDrivers), 
									std::get<2>(ioDrivers), 
									std::get<3>(ioDrivers),
									std::get<4>(ioDrivers));
				};
		    	auto helper = makeTestHelper(makeTestMachine(getIoDrivers));

		    	WHEN("Servo0 is set to 90 degrees") {
		    		helper.sendCommand("M280 P0 S90.0", "ok");
		    		THEN("Its highTime should be 2 ms (25% interpolation of 1, 5) and other servos should not have been affected") {
			    		helper.requireDurationsApproxEqual(servo0.highTime, std::chrono::milliseconds(2));
			    		helper.requireDurationsApproxEqual(servo1.highTime, std::chrono::milliseconds(3));
			    	}
		    	}
		    	WHEN("Servo1 is set to 135 degrees") {
		    		helper.sendCommand("M280 P1 S135", "ok");
		    		THEN("Its highTime should be 2.5 ms (50% interpolation of 2, 3) and other servos should not have been affected") {
		    			helper.requireDurationsApproxEqual(servo0.highTime, std::chrono::milliseconds(1));
		    			helper.requireDurationsApproxEqual(servo1.highTime, std::chrono::microseconds(2500));
		    		}
		    	}
		    	WHEN("Servo1 is set to 270 degrees") {
		    		helper.sendCommand("M280 P1 S270", "ok");
		    		THEN("It should be clamped to its limit, 180 degrees, and other servos should be unaffected") {
		    			helper.requireDurationsApproxEqual(servo0.highTime, std::chrono::milliseconds(1));
		    			helper.requireDurationsApproxEqual(servo1.highTime, std::chrono::milliseconds(3));
		    		}
		    	}
		    }
		}
	}
};

SCENARIO("Servo will behave correctly", "[servo]") {
    ServoTester();
}

}