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

#include "iopin.h"

#include <cassert>

#include "schedulerbase.h" //for SchedulerBase::registerExitHandler
#include "common/logging.h"
//for the testsuite
#include "catch.hpp"

namespace iodrv {

std::set<IoPin*> IoPin::livingPins; //allocate storage for static variables.

void IoPin::deactivateAll() {
    LOG("IoPin::deactivateAll()\n");
    for (auto p : livingPins) {
        p->setToDefault();
    }
}

void IoPin::registerExitHandler() {
    //install exit handler to leave pins in a safe state post-execution.
    static bool doOnce(SchedulerBase::registerExitHandler((void(*)())&deactivateAll, SCHED_IO_EXIT_LEVEL));
    (void)doOnce; //destroy 'unused' warning
}

IoPin::~IoPin() {
    setToDefault();
    livingPins.erase(this);
}

//move constructor:
IoPin::IoPin(IoPin &&other) : _pin(PrimitiveIoPin::null()) {
	*this = std::move(other);
}

IoPin& IoPin::operator=(IoPin &&other) {
	_invertReads = other._invertReads;
    _invertWrites = other._invertWrites;
    _defaultState = other._defaultState;
    #ifndef NDEBUG
        _currentMode = other._currentMode;
    #endif
	_pin = other._pin;
	other._pin = IoPin::null()._pin; //PrimitiveIoPin::null()
	livingPins.insert(this);
    livingPins.erase(&other);
    return *this;
}

void IoPin::setDefaultState(DefaultIoState state) {
    _defaultState = state;
}

bool IoPin::isNull() const { 
    return _pin.isNull(); 
}
IoLevel IoPin::translateWriteToPrimitive(IoLevel lev) const { 
    return _invertWrites ? !lev : lev; 
}
float IoPin::translateDutyCycleToPrimitive(float pwm) const {
    return _invertWrites ? 1-pwm : pwm;
}
const PrimitiveIoPin& IoPin::primitiveIoPin() const { 
    return _pin; 
}
//wrapper functions that take the burden of inversions, etc off the platform-specific drivers:

void IoPin::makeDigitalOutput(IoLevel lev) {
    #ifndef NDEBUG
        _currentMode = IOPIN_MODE_OUTPUT;
    #endif
    //relay the call to the real pin, performing any inversions necessary
    _pin.makeDigitalOutput(translateWriteToPrimitive(lev));
}
void IoPin::makeDigitalInput() {
    #ifndef NDEBUG
        _currentMode = IOPIN_MODE_INPUT;
    #endif
    _pin.makeDigitalInput();
}
void IoPin::makePwmOutput(float duty, float desiredPeriod) {
    #ifndef NDEBUG
        _currentMode = IOPIN_MODE_PWM;
    #endif
    _pin.makePwmOutput(translateDutyCycleToPrimitive(duty), desiredPeriod);
}
IoLevel IoPin::digitalRead() const {
    #ifndef NDEBUG
        assert(_currentMode == IOPIN_MODE_INPUT);
    #endif
    //relay the call to the real pin, performing any inversions necessary
    return _invertReads ? !_pin.digitalRead() : _pin.digitalRead();
}
void IoPin::digitalWrite(IoLevel lev) {
    #ifndef NDEBUG
        assert(_currentMode == IOPIN_MODE_OUTPUT);
    #endif
    //relay the call to the real pin, performing any inversions necessary
    _pin.digitalWrite(translateWriteToPrimitive(lev));
}
void IoPin::pwmWrite(float duty, float desiredPeriod) {
    #ifndef NDEBUG
        assert(_currentMode == IOPIN_MODE_PWM);
    #endif
    //relay the call to the real pin, performing any inversions necessary
    _pin.pwmWrite(translateDutyCycleToPrimitive(duty), desiredPeriod);
}

void IoPin::setToDefault() {
    //set the pin to a "safe" default state:
    if (!_pin.isNull()) {
        if (_defaultState == IO_DEFAULT_LOW) {
            makeDigitalOutput(IoLow);
        } else if (_defaultState == IO_DEFAULT_HIGH) {
            makeDigitalOutput(IoHigh);
        } else if (_defaultState == IO_DEFAULT_HIGH_IMPEDANCE) {
            makeDigitalInput();
        }
    }
}

}

TEST_CASE("IoPins will correctly invert writes", "[iopin]") {
    SECTION("Inverted reads won't invert writes") {
        iodrv::IoPin p(iodrv::INVERT_READS, PrimitiveIoPin::null());
        REQUIRE(p.translateWriteToPrimitive(IoLow) == IoLow);
        REQUIRE(p.translateWriteToPrimitive(IoHigh) == IoHigh);
        REQUIRE(p.translateDutyCycleToPrimitive(0.2) == Approx(0.2));
    }
    SECTION("Inverted pins will invert writes") {
        iodrv::IoPin p(iodrv::INVERT_WRITES, PrimitiveIoPin::null());
        REQUIRE(p.translateWriteToPrimitive(IoLow) == IoHigh);
        REQUIRE(p.translateWriteToPrimitive(IoHigh) == IoLow);
        REQUIRE(p.translateDutyCycleToPrimitive(0.2) == Approx(0.8));
    }
}