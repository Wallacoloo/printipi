#include "iopin.h"

namespace drv {

std::set<IoPin*> IoPin::livingPins; //allocate storage for static variables.
IoPin::null IoPin::null::_null;

//move constructor:
IoPin::IoPin(IoPin &&other)  : _pin(std::move(other._pin)), 
      _invertReads(std::move(other._invertReads)), _invertWrites(std::move(other._invertWrites)),
      _defaultState(std::move(other._defaultState)) {
        other._pin = IoPin::null::ref()._pin; //set the other pin to null to prevent it from deactivating
        livingPins.insert(this);
        livingPins.erase(&other);
    }

}