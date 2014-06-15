#include "state.h"

namespace gparse {

const std::string State::OP_G90  = "G90";
const std::string State::OP_G91  = "G91";
const std::string State::OP_M21  = "M21";
const std::string State::OP_M105 = "M105";
const std::string State::OP_M110 = "M110";

void State::setPositionMode(PositionMode mode) {
	this->positionMode = mode; 
}

}
