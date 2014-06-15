#include "state.h"

namespace gparse {

const std::string State::OP_G1   = "G1";
const std::string State::OP_G20  = "G20";
const std::string State::OP_G21  = "G21";
const std::string State::OP_G90  = "G90";
const std::string State::OP_G91  = "G91";
const std::string State::OP_M21  = "M21";
const std::string State::OP_M105 = "M105";
const std::string State::OP_M110 = "M110";

void State::setPositionMode(PositionMode mode) {
	this->positionMode = mode; 
}

void State::setUnitMode(LengthUnit mode) {
	this->unitMode = mode;
}

}
