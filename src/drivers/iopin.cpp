#include "iopin.h"

namespace drv {

std::set<IoPin*> IoPin::livingPins; //allocate storage for static variables.
IoPin::null IoPin::null::_null;

//move constructor:
IoPin::IoPin(IoPin &&other) : _pin(PrimitiveIoPin::null()) {
	*this = std::move(other);
}

IoPin& IoPin::operator=(IoPin &&other) {
	_pin = other._pin;
	other._pin = IoPin::null::ref()._pin;
	livingPins.insert(this);
    livingPins.erase(&other);
    return *this;
}

}