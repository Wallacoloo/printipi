#include "state.h"
#include "math.h"

namespace gparse {

const std::string State::OP_G1   = "G1";
const std::string State::OP_G20  = "G20";
const std::string State::OP_G21  = "G21";
const std::string State::OP_G90  = "G90";
const std::string State::OP_G91  = "G91";
const std::string State::OP_M21  = "M21";
const std::string State::OP_M105 = "M105";
const std::string State::OP_M110 = "M110";

State::State(const drv::Driver &drv) {
	this->setDestMoveRatePrimitive(drv.defaultMoveRate());
	this->setDestFeedRatePrimitive(drv.defaultFeedRate());
}

void State::setPositionMode(PositionMode mode) {
	this->positionMode = mode; 
}

void State::setUnitMode(LengthUnit mode) {
	this->unitMode = mode;
}

float State::xUnitToAbsolute(float posUnit) const {
	switch (this->positionMode) {
		case POS_RELATIVE:
			posUnit += this->_destXPrimitive;
			break;
		case POS_ABSOLUTE:
		default:
			break; //no transformation needed.
	}
	return posUnit;
}
float State::yUnitToAbsolute(float posUnit) const {
	switch (this->positionMode) {
		case POS_RELATIVE:
			posUnit += this->_destYPrimitive;
			break;
		case POS_ABSOLUTE:
		default:
			break; //no transformation needed.
	}
	return posUnit;
}
float State::zUnitToAbsolute(float posUnit) const {
	switch (this->positionMode) {
		case POS_RELATIVE:
			posUnit += this->_destZPrimitive;
			break;
		case POS_ABSOLUTE:
		default:
			break; //no transformation needed.
	}
	return posUnit;
}
float State::eUnitToAbsolute(float posUnit) const {
	switch (this->positionMode) {
		case POS_RELATIVE:
			posUnit += this->_destEPrimitive;
			break;
		case POS_ABSOLUTE:
		default:
			break; //no transformation needed.
	}
	return posUnit;
}
float State::posUnitToMM(float posUnit) const {
	switch (this->unitMode) {
		case UNIT_IN:
			return gparse::math::MM_PER_IN * posUnit;
		case UNIT_MM:
		default: //impossible case.
			return posUnit;
	}
}

float State::xUnitToPrimitive(float posUnit) const {
	return posUnitToMM(xUnitToAbsolute(posUnit));
}
float State::yUnitToPrimitive(float posUnit) const {
	return posUnitToMM(yUnitToAbsolute(posUnit));
}
float State::zUnitToPrimitive(float posUnit) const {
	return posUnitToMM(zUnitToAbsolute(posUnit));
}
float State::eUnitToPrimitive(float posUnit) const {
	return posUnitToMM(eUnitToAbsolute(posUnit));
}
float State::fUnitToPrimitive(float posUnit) const {
	return posUnitToMM(posUnit); //feed rate is always relative, so no need to call toAbsolute
}

float State::destXPrimitive() const {
	return this->_destXPrimitive;
}
float State::destYPrimitive() const {
	return this->_destYPrimitive;
}
float State::destZPrimitive() const {
	return this->_destZPrimitive;
}
float State::destEPrimitive() const {
	return this->_destEPrimitive;
}
float State::destMoveRatePrimitive() const {
	return this->_destMoveRatePrimitive;
}
void State::setDestMoveRatePrimitive(float f) {
	this->_destMoveRatePrimitive = f;
}
float State::destFeedRatePrimitive() const {
	return this->_destFeedRatePrimitive;
}
void State::setDestFeedRatePrimitive(float f) {
	this->_destFeedRatePrimitive = f;
}

}
