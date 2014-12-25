#include "iopin.h"

#include "schedulerbase.h" //for SchedulerBase::registerExitHandler
#include "common/logging.h"
#include "catch.hpp" //for the testsuite

namespace iodrv {

std::set<IoPin*> IoPin::livingPins; //allocate storage for static variables.
IoPin::null IoPin::null::_null;

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
	_pin = other._pin;
	other._pin = IoPin::null::ref()._pin;
	livingPins.insert(this);
    livingPins.erase(&other);
    return *this;
}

}

TEST_CASE("IoPins will correctly invert writes", "[iopin]") {
    SECTION("Inverted reads won't invert writes") {
        iodrv::IoPin p(iodrv::INVERT_READS, IoDefaultLow, PrimitiveIoPin::null());
        REQUIRE(p.translateWriteToPrimitive(IoLow) == IoLow);
        REQUIRE(p.translateWriteToPrimitive(IoHigh) == IoHigh);
        REQUIRE(p.translateDutyCycleToPrimitive(0.2) == Approx(0.2));
    }
    SECTION("Inverted pins will invert writes") {
        iodrv::IoPin p(iodrv::INVERT_WRITES, IoDefaultLow, PrimitiveIoPin::null());
        REQUIRE(p.translateWriteToPrimitive(IoLow) == IoHigh);
        REQUIRE(p.translateWriteToPrimitive(IoHigh) == IoLow);
        REQUIRE(p.translateDutyCycleToPrimitive(0.2) == Approx(0.8));
    }
}