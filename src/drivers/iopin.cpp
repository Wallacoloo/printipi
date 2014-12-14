#include "iopin.h"

#include "schedulerbase.h" //for SchedulerBase::registerExitHandler
#include "common/logging.h"

namespace drv {

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
    LOG("IoPin::registerExitHandler()\n");
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