#ifndef STATE_H
#define STATE_H

//Gcode documentation can be found:
//  http://reprap.org/wiki/G-code
//  or (with implementation): https://github.com/Traumflug/Teacup_Firmware/blob/master/gcode_process.c
//  Marlin-specific: http://www.ctheroux.com/2012/11/g-code-commands-supported-by-marlin/

#include <string>
#include <cstddef> //for size_t
#include <stdexcept> //for runtime_error
#include <array>
//#include <memory> //for unique_ptr
#include <utility> //for std::pair
#include "logging.h"
#include "gparse/command.h"
#include "event.h"
#include "scheduler.h"
#include "drivers/driver.h"
#include "gmath.h" //note: relative import
#include "drivers/axisstepper.h"
#include "typesettings.h"

template <typename Drv> class State {
	PositionMode _positionMode; // = POS_ABSOLUTE;
	PositionMode _extruderPosMode; // = POS_RELATIVE; //set via M82 and M83
	LengthUnit unitMode; // = UNIT_MM;
	Scheduler scheduler;
	float _destXPrimitive, _destYPrimitive, _destZPrimitive;
	float _destEPrimitive;
	float _destMoveRatePrimitive; //varies accross drivers
	float _destFeedRatePrimitive;
	float _hostZeroX, _hostZeroY, _hostZeroZ, _hostZeroE; //the host can set any arbitrary point to be referenced as 0.
	std::array<int, Drv::numAxis()> _destMechanicalPos; //number of steps for each stepper motor.
	const Drv &driver;
	public:
	    //so-called "Primitive" units represent a cartesian coordinate from the origin, using some primitive unit (mm)
		static const int DEFAULT_HOTEND_TEMP = -300;
		static const int DEFAULT_BED_TEMP = -300;
		State(const Drv &drv);
		/* Control interpretation of positions from the host as relative or absolute */
		PositionMode positionMode() const;
		void setPositionMode(PositionMode mode);
		/* Control interpretation of *extruder* positions from the host as relative or absolute.
		 *If not explicitly set, it will default to the XYZ position mode. */
		PositionMode extruderPosMode() const;
		void setExtruderPosMode(PositionMode mode);
		/* Control interpretation of distances sent by the host as inches or millimeters */
		void setUnitMode(LengthUnit mode);
		/* Convert an x/y/z/e value sent from the host to its absolute value, in the case that the host is sending relative positions */
		float xUnitToAbsolute(float posUnit) const;
		float yUnitToAbsolute(float posUnit) const;
		float zUnitToAbsolute(float posUnit) const;
		float eUnitToAbsolute(float posUnit) const;
		/* Convert an x/y/z/e value sent from the host to MM, in the case that the host is sending inches */
		float posUnitToMM(float posUnit) const;
		/* Convert an x/y/z/e value sent from the host to whatever primitive value we're using internally
		 * Acts similarly as a shortcut for posUnitToMM(xUnitToAbsolute(x)), though it may apply transformations in the future.*/
		float xUnitToPrimitive(float posUnit) const;
		float yUnitToPrimitive(float posUnit) const;
		float zUnitToPrimitive(float posUnit) const;
		float eUnitToPrimitive(float posUnit) const;
		float fUnitToPrimitive(float posUnit) const;
		/* Get the last queued position (X, Y, Z, E). Future queued commands may depend on this */
		float destXPrimitive() const; 
		float destYPrimitive() const;
		float destZPrimitive() const;
		float destEPrimitive() const;
		/* Control the feed and move rate */
		float destMoveRatePrimitive() const;
		void setDestMoveRatePrimitive(float f);
		float destFeedRatePrimitive() const;
		void setDestFeedRatePrimitive(float f);
		/* The host can set any arbitrary point to be a reference to 0 */
		void setHostZeroPos(float x, float y, float z, float e);
		
		/*execute the GCode on a Driver object that supports a well-defined interface.
		 *returns a Command to send back to the host.*/
		gparse::Command execute(gparse::Command const& cmd);
		void queueMovement(float curX, float curY, float curZ, float curE, float x, float y, float z, float e, float velXYZ, float velE);
};


template <typename Drv> State<Drv>::State(const Drv &drv) : _positionMode(POS_ABSOLUTE), _extruderPosMode(POS_UNDEFINED),  
	unitMode(UNIT_MM), 
	_destXPrimitive(0), _destYPrimitive(0), _destZPrimitive(0), _destEPrimitive(0),
	_hostZeroX(0), _hostZeroY(0), _hostZeroZ(0), _hostZeroE(0),
	_destMechanicalPos(), 
	driver(drv) {
	this->setDestMoveRatePrimitive(drv.defaultMoveRate());
	this->setDestFeedRatePrimitive(drv.defaultFeedRate());
}

template <typename Drv> PositionMode State<Drv>::positionMode() const {
	return this->_positionMode;
}
template <typename Drv> void State<Drv>::setPositionMode(PositionMode mode) {
	this->_positionMode = mode; 
}

template <typename Drv> PositionMode State<Drv>::extruderPosMode() const {
	return this->_extruderPosMode == POS_UNDEFINED ? positionMode() : this->_extruderPosMode;
}
template <typename Drv> void State<Drv>::setExtruderPosMode(PositionMode mode) {
	this->_extruderPosMode = mode;
}

template <typename Drv> void State<Drv>::setUnitMode(LengthUnit mode) {
	this->unitMode = mode;
}

template <typename Drv> float State<Drv>::xUnitToAbsolute(float posUnit) const {
	switch (this->positionMode()) {
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
	switch (this->positionMode()) {
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
	switch (this->positionMode()) {
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
	switch (this->extruderPosMode()) {
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
			return gmath::MM_PER_IN * posUnit;
		case UNIT_MM:
		default: //impossible case.
			return posUnit;
	}
}

template <typename Drv> float State<Drv>::xUnitToPrimitive(float posUnit) const {
	return posUnitToMM(xUnitToAbsolute(posUnit)) + this->_hostZeroX;
}
template <typename Drv> float State<Drv>::yUnitToPrimitive(float posUnit) const {
	return posUnitToMM(yUnitToAbsolute(posUnit)) + this->_hostZeroY;
}
template <typename Drv> float State<Drv>::zUnitToPrimitive(float posUnit) const {
	return posUnitToMM(zUnitToAbsolute(posUnit)) + this->_hostZeroZ;
}
template <typename Drv> float State<Drv>::eUnitToPrimitive(float posUnit) const {
	return posUnitToMM(eUnitToAbsolute(posUnit)) + this->_hostZeroE;
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

template <typename Drv> void State<Drv>::setHostZeroPos(float x, float y, float z, float e) {
	_hostZeroX = x;
	_hostZeroY = y;
	_hostZeroZ = z;
	_hostZeroE = e;
}

template <typename Drv> gparse::Command State<Drv>::execute(gparse::Command const& cmd) {
	std::string opcode = cmd.getOpcode();
	gparse::Command resp;
	if (cmd.isG0() || cmd.isG1()) { //rapid movement / controlled (linear) movement (currently uses same code)
		LOG("Warning (gparse/state.h): OP_G0/1 (linear movement) not fully implemented - notably extrusion\n");
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
		this->queueMovement(curX, curY, curZ, curE, x, y, z, e, destMoveRatePrimitive(), destFeedRatePrimitive());
		resp = gparse::Command::OK;
	} else if (cmd.isG20()) { //g-code coordinates will now be interpreted as inches
		setUnitMode(UNIT_IN);
		resp = gparse::Command::OK;
	} else if (cmd.isG21()) { //g-code coordinates will now be interpreted as millimeters.
		setUnitMode(UNIT_MM);
		resp = gparse::Command::OK;
	} else if (cmd.isG28()) { //home to end-stops
		LOG("Warning (gparse/state.h): OP_G28 (home to end-stops) not fully implemented\n");
		bool homeX = cmd.hasX(); //can optionally specify specific axis to home.
		bool homeY = cmd.hasY();
		bool homeZ = cmd.hasZ();
		if (!homeX && !homeY && !homeZ) { //if no axis are passed, then home ALL axis.
			homeX = homeY = homeZ = true;
		}
		float curX = destXPrimitive();
		float curY = destYPrimitive();
		float curZ = destZPrimitive();
		float curE = destEPrimitive();
		float newX = homeX ? 0 : curX;
		float newY = homeY ? 0 : curY;
		float newZ = homeZ ? 0 : curZ;
		this->queueMovement(curX, curY, curZ, curE, newX, newY, newZ, curE, destMoveRatePrimitive(), destFeedRatePrimitive());
		resp = gparse::Command::OK;
	} else if (cmd.isG90()) { //set g-code coordinates to absolute
		setPositionMode(POS_ABSOLUTE);
		resp = gparse::Command::OK;
	} else if (cmd.isG91()) { //set g-code coordinates to relative
		setPositionMode(POS_RELATIVE);
		resp = gparse::Command::OK;
	} else if (cmd.isG92()) { //set current position = 0
		//LOG("Warning (gparse/state.h): OP_G92 (set current position as reference to zero) not tested\n");
		float actualX, actualY, actualZ, actualE;
		bool hasXYZE = cmd.hasAnyXYZEParam();
		if (!hasXYZE) { //make current position (0, 0, 0, 0)
			actualX = actualY = actualZ = actualE = posUnitToMM(0);
		} else {
			actualX = cmd.hasX() ? posUnitToMM(cmd.getX()) : _hostZeroX;
			actualY = cmd.hasY() ? posUnitToMM(cmd.getY()) : _hostZeroY;
			actualZ = cmd.hasZ() ? posUnitToMM(cmd.getZ()) : _hostZeroZ;
			actualE = cmd.hasE() ? posUnitToMM(cmd.getE()) : _hostZeroE;
		}
		setHostZeroPos(actualX, actualY, actualZ, actualE);
		resp = gparse::Command::OK;
	} else if (cmd.isM21()) { //initialize SD card (nothing to do).
		resp = gparse::Command::OK;
	} else if (cmd.isM82()) { //set extruder absolute mode
		setExtruderPosMode(POS_ABSOLUTE);
		resp = gparse::Command::OK;
	} else if (cmd.isM83()) { //set extruder relative mode
		setExtruderPosMode(POS_RELATIVE);
		resp = gparse::Command::OK;
	} else if (cmd.isM104()) { //set hotend temperature and return immediately.
		LOG("Warning (gparse/state.h): OP_M104 (set hotend temp) not implemented\n");
		resp = gparse::Command::OK;
	} else if (cmd.isM105()) { //get temperature, in C
		int t=DEFAULT_HOTEND_TEMP, b=DEFAULT_BED_TEMP; //a temperature < absolute zero means no reading available.
		driver.getTemperature(t, b);
		resp = gparse::Command("ok T:" + std::to_string(t) + " B:" + std::to_string(b));
	} else if (cmd.isM106()) { //set fan speed. Takes parameter S. Can be 0-255 (PWM) or in some implementations, 0.0-1.0
		LOG("Warning (gparse/state.h): OP_M106 (set fan speed) not implemented\n");
		resp = gparse::Command::OK;
	} else if (cmd.isM107()) { //set fan = off.
		LOG("Warning (gparse/state.h): OP_M106 (set fan off) not implemented\n");
		resp = gparse::Command::OK;
	} else if (cmd.isM109()) { //set extruder temperature to S param and wait.
		LOG("Warning (gparse/state.h): OP_M109 (set extruder temperature and wait) not implemented\n");
		resp = gparse::Command::OK;
	} else if (cmd.isM110()) { //set current line number
		resp = gparse::Command::OK;
	} else if (cmd.isM117()) { //print message
		resp = gparse::Command::OK;
	} else if (cmd.isM140()) { //set BED temp and return immediately.
		LOG("Warning (gparse/state.h): OP_M140 (set bed temp) not implemented\n");
		resp = gparse::Command::OK;
	} else if (cmd.isTxxx()) { //set tool number
		LOG("Warning (gparse/state.h): OP_T[n] (set tool number) not implemented\n");
		resp = gparse::Command::OK;
	} else {
		throw new std::runtime_error("unrecognized gcode opcode");
	}
	return resp;
}
		
template <typename Drv> void State<Drv>::queueMovement(float curX, float curY, float curZ, float curE, float x, float y, float z, float e, float velXYZ, float velE) {
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
	typename Drv::AxisSteppers iters;
	drv::AxisStepper::initAxisSteppers(iters, _destMechanicalPos, vx, vy, vz, velE);
	//initialize iterators...
	do {
		drv::AxisStepper& s = drv::AxisStepper::getNextTime<typename Drv::AxisSteppers>(iters);
		LOG("Next step: %i at %f of %f. Is s.time <= 0? %i. Is vy == 0? %i\n", s.index(), s.time, duration, s.time <= 0, vy == 0);
		if (s.time > duration || s.time <= 0) { 
			break; 
		}
		scheduler.queue(s.getEvent());
		s.nextStep(iters);
	} while (1);
}

#endif
