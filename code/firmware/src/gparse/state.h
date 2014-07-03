#ifndef GPARSE_STATE_H
#define GPARSE_STATE_H

//Gcode documentation can be found:
//  http://reprap.org/wiki/G-code
//  or (with implementation): https://github.com/Traumflug/Teacup_Firmware/blob/master/gcode_process.c
//  Marlin-specific: http://www.ctheroux.com/2012/11/g-code-commands-supported-by-marlin/

#include <string>
#include <cstddef> //for size_t
#include <stdexcept> //for runtime_error
//#include <memory> //for unique_ptr
#include <utility> //for std::pair
#include "command.h"
#include "event.h"
#include "scheduler.h"
#include "../drivers/driver.h"
#include "math.h" //note: relative import

namespace gparse {

enum PositionMode {
	POS_ABSOLUTE,
	POS_RELATIVE
};

enum LengthUnit {
	UNIT_MM,
	UNIT_IN
};

template <typename Drv> class State {
	PositionMode positionMode; // = POS_ABSOLUTE;
	//PositionMode extruderPosMode = POS_RELATIVE; //set via M82 and M83
	LengthUnit unitMode; // = UNIT_MM;
	Scheduler scheduler;
	//float _destXPrimitive=0, _destYPrimitive=0, _destZPrimitive=0;
	//float _destEPrimitive=0;
	float _destXPrimitive, _destYPrimitive, _destZPrimitive;
	float _destEPrimitive;
	float _destMoveRatePrimitive; //varies accross drivers
	float _destFeedRatePrimitive;
	public:
	    //so-called "Primitive" units represent a cartesian coordinate from the origin, using some primitive unit (mm)
		static const int DEFAULT_HOTEND_TEMP = -300;
		static const int DEFAULT_BED_TEMP = -300;
		static const std::string OP_G1  ;//  = "G1" ;
		static const std::string OP_G20 ;//  = "G20";
		static const std::string OP_G21 ;//  = "G21";
		static const std::string OP_G90 ;//  = "G90";
		static const std::string OP_G91 ;//  = "G91";
		static const std::string OP_M21 ;//  = "M21";
		static const std::string OP_M105;// = "M105";
		static const std::string OP_M110;// = "M110";
		State(const drv::Driver &drv);
		void setPositionMode(PositionMode mode);
		void setUnitMode(LengthUnit mode);
		float xUnitToAbsolute(float posUnit) const;
		float yUnitToAbsolute(float posUnit) const;
		float zUnitToAbsolute(float posUnit) const;
		float eUnitToAbsolute(float posUnit) const;
		float posUnitToMM(float posUnit) const;
		float xUnitToPrimitive(float posUnit) const;
		float yUnitToPrimitive(float posUnit) const;
		float zUnitToPrimitive(float posUnit) const;
		float eUnitToPrimitive(float posUnit) const;
		float fUnitToPrimitive(float posUnit) const;
		float destXPrimitive() const; //the last queued position (X, Y, Z, E) and feed rate. Future queued commands may depend on this.
		float destYPrimitive() const;
		float destZPrimitive() const;
		float destEPrimitive() const;
		float destMoveRatePrimitive() const;
		void setDestMoveRatePrimitive(float f);
		float destFeedRatePrimitive() const;
		void setDestFeedRatePrimitive(float f);
		
		//void queueMovement(float curX, float curY, float curZ, float curE, float x, float y, float z, float e, float velSpace, float velExt);
		/*execute the GCode on a Driver object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		//template <typename Drv> Command execute(Command const& cmd, Drv &driver) {
		Command execute(Command const& cmd, Drv &driver);
		
		//template <typename Drv> void queueMovement(const Drv &driver, float curX, float curY, float curZ, float curE, float x, float y, float z, float e, float velXYZ, float velE) {
		void queueMovement(const Drv &driver, float curX, float curY, float curZ, float curE, float x, float y, float z, float e, float velXYZ, float velE);
};

template <typename Drv> const std::string State<Drv>::OP_G1   = "G1";
template <typename Drv> const std::string State<Drv>::OP_G20  = "G20";
template <typename Drv> const std::string State<Drv>::OP_G21  = "G21";
template <typename Drv> const std::string State<Drv>::OP_G90  = "G90";
template <typename Drv> const std::string State<Drv>::OP_G91  = "G91";
template <typename Drv> const std::string State<Drv>::OP_M21  = "M21";
template <typename Drv> const std::string State<Drv>::OP_M105 = "M105";
template <typename Drv> const std::string State<Drv>::OP_M110 = "M110";

template <typename Drv> State<Drv>::State(const drv::Driver &drv) : positionMode(POS_ABSOLUTE), unitMode(UNIT_MM),
	_destXPrimitive(0), _destYPrimitive(0), _destZPrimitive(0), _destEPrimitive(0) {
	this->setDestMoveRatePrimitive(drv.defaultMoveRate());
	this->setDestFeedRatePrimitive(drv.defaultFeedRate());
}

template <typename Drv> void State<Drv>::setPositionMode(PositionMode mode) {
	this->positionMode = mode; 
}

template <typename Drv> void State<Drv>::setUnitMode(LengthUnit mode) {
	this->unitMode = mode;
}

template <typename Drv> float State<Drv>::xUnitToAbsolute(float posUnit) const {
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
template <typename Drv> float State<Drv>::yUnitToAbsolute(float posUnit) const {
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
template <typename Drv> float State<Drv>::zUnitToAbsolute(float posUnit) const {
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
template <typename Drv> float State<Drv>::eUnitToAbsolute(float posUnit) const {
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
template <typename Drv> float State<Drv>::posUnitToMM(float posUnit) const {
	switch (this->unitMode) {
		case UNIT_IN:
			return gparse::math::MM_PER_IN * posUnit;
		case UNIT_MM:
		default: //impossible case.
			return posUnit;
	}
}

template <typename Drv> float State<Drv>::xUnitToPrimitive(float posUnit) const {
	return posUnitToMM(xUnitToAbsolute(posUnit));
}
template <typename Drv> float State<Drv>::yUnitToPrimitive(float posUnit) const {
	return posUnitToMM(yUnitToAbsolute(posUnit));
}
template <typename Drv> float State<Drv>::zUnitToPrimitive(float posUnit) const {
	return posUnitToMM(zUnitToAbsolute(posUnit));
}
template <typename Drv> float State<Drv>::eUnitToPrimitive(float posUnit) const {
	return posUnitToMM(eUnitToAbsolute(posUnit));
}
template <typename Drv> float State<Drv>::fUnitToPrimitive(float posUnit) const {
	return posUnitToMM(posUnit); //feed rate is always relative, so no need to call toAbsolute
}

template <typename Drv> float State<Drv>::destXPrimitive() const {
	return this->_destXPrimitive;
}
template <typename Drv> float State<Drv>::destYPrimitive() const {
	return this->_destYPrimitive;
}
template <typename Drv> float State<Drv>::destZPrimitive() const {
	return this->_destZPrimitive;
}
template <typename Drv> float State<Drv>::destEPrimitive() const {
	return this->_destEPrimitive;
}
template <typename Drv> float State<Drv>::destMoveRatePrimitive() const {
	return this->_destMoveRatePrimitive;
}
template <typename Drv> void State<Drv>::setDestMoveRatePrimitive(float f) {
	this->_destMoveRatePrimitive = f;
}
template <typename Drv> float State<Drv>::destFeedRatePrimitive() const {
	return this->_destFeedRatePrimitive;
}
template <typename Drv> void State<Drv>::setDestFeedRatePrimitive(float f) {
	this->_destFeedRatePrimitive = f;
}

template <typename Drv> Command State<Drv>::execute(Command const& cmd, Drv &driver) {
	std::string opcode = cmd.getOpcode();
	Command resp;
	if (opcode == OP_G1) { //controlled (linear) movement.
	    bool hasX, hasY, hasZ, hasE, hasF;
	    float curX = destXPrimitive();
	    float curY = destYPrimitive();
	    float curZ = destZPrimitive();
	    float curE = destEPrimitive();
		float x = cmd.getX(hasX); //new x-coordinate.
		float y = cmd.getY(hasY); //new y-coordinate.
		float z = cmd.getZ(hasZ); //new z-coordinate.
		float e = cmd.getE(hasE); //extrusion amount.
		float f = cmd.getF(hasF); //feed-rate.
		x = hasX ? xUnitToPrimitive(x) : curX;
		y = hasY ? yUnitToPrimitive(y) : curY;
		z = hasZ ? zUnitToPrimitive(z) : curZ;
		if (hasF) {
			this->setDestFeedRatePrimitive(fUnitToPrimitive(f));
		}
		//TODO: calculate future e based on feedrate.
		this->queueMovement(driver, curX, curY, curZ, curE, x, y, z, e, destMoveRatePrimitive(), destFeedRatePrimitive());
	} else if (opcode == OP_G20) { //g-code coordinates will now be interpreted as inches
		setUnitMode(UNIT_IN);
		resp = Command::OK;
	} else if (opcode == OP_G21) { //g-code coordinates will now be interpreted as millimeters.
		setUnitMode(UNIT_MM);
		resp = Command::OK;
	} else if (opcode == OP_G90) { //set g-code coordinates to absolute
		setPositionMode(POS_ABSOLUTE);
		resp = Command::OK;
	} else if (opcode == OP_G91) { //set g-code coordinates to relative
		setPositionMode(POS_RELATIVE);
		resp = Command::OK;
	} else if (opcode == OP_M21) { //initialize SD card (nothing to do).
		resp = Command::OK;
	} else if (opcode == OP_M105) { //get temperature, in C
		int t=DEFAULT_HOTEND_TEMP, b=DEFAULT_BED_TEMP; //a temperature < absolute zero means no reading available.
		driver.getTemperature(t, b);
		resp = Command("ok T:" + std::to_string(t) + " B:" + std::to_string(b));
	} else if (opcode == OP_M110) { //set current line number
		resp = Command::OK;
	} else {
		throw new std::runtime_error("unrecognized gcode opcode");
	}
	return resp;
}
		
template <typename Drv> void State<Drv>::queueMovement(const Drv &driver, float curX, float curY, float curZ, float curE, float x, float y, float z, float e, float velXYZ, float velE) {
	float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
	float dist = sqrt(distSq);
	float vx = (x-curX)/dist * velXYZ;
	float vy = (y-curY)/dist * velXYZ;
	float vz = (z-curZ)/dist * velXYZ;
	float durationXYZ = dist/velXYZ;
	float durationE = abs(e-curE)/velE;
	float duration = std::min(durationXYZ, durationE);
	//this->_queueMovement(driver, curX, curY, curZ, curE, vx, vy, vz, velE, durationXYZ, durationE);
	constexpr std::size_t numAxis = Drv::numAxis(); //driver.numAxis();
	if (numAxis == 0) { 
		return; //some of the following logic may assume that there are at least 1 axis.
	}
	//std::unique_ptr<float[]> times(new float[numAxis]); //no size penalty vs new/delete using -Os and -flto
	//std::unique_ptr<std::pair<float, gparse::StepDirection>[] > times(new std::pair<float, gparse::StepDirection>[numAxis]);
	std::pair<float, gparse::StepDirection> times[numAxis];
	for (int i=0; i<numAxis; ++i) { //initialize
		times[i].first = driver.relativeTimeOfNextStep(i, times[i].second, curX, curY, curZ, curE, vx, vy, vz, velE);
	}
	
	int minIdx = 0;
	do {
		for (int i=1; i<numAxis; ++i) {
			if (times[i].first < times[minIdx].first) {
				minIdx = i;
			} 
		}
		float t = times[minIdx].second;
		scheduler.queue(Event::StepperEvent(t, minIdx, times[minIdx].second));
		float txyz = std::min(t, duration); //need to know how much time has been spent traveling for XYZ or Extruder.
		float te = std::min(t, durationE);
		float tempX = curX + txyz*vx; //get the current X/Y/Z for which we want to find the next step.
		float tempY = curY + txyz*vy;
		float tempZ = curZ + txyz*vz;
		float tempE = curE + te*velE;
		//Calculate the next time to trigger this motor.
		times[minIdx].first = driver.relativeTimeOfNextStep(minIdx, times[minIdx].second, curX, curY, curZ, curE, vx, vy, vz, velE);
	} while (times[minIdx].first < duration);
}

}
#endif
