#ifndef STATE_H
#define STATE_H

/* 
 * Printipi/state.h
 * (c) 2014 Colin Wallace
 *
 * State handles as much driver-mutual functionality as possible, including mapping Gcodes to specific functions,
 *   tracking unit mode and axis position, and interfacing with the scheduler.
 * State controls the communications channel, the scheduler, and the underlying driver.
 */

//Gcode documentation can be found:
//  http://reprap.org/wiki/G-code
//  or (with implementation): https://github.com/Traumflug/Teacup_Firmware/blob/master/gcode_process.c
//  Marlin-specific: http://www.ctheroux.com/2012/11/g-code-commands-supported-by-marlin/
//  Clarification of E and F: http://forums.reprap.org/read.php?263,208245
//    E is the extruder coordinate. F is the "feed rate", which is really the rate at which X, Y, Z moves.

#include <string>
#include <cstddef> //for size_t
#include <stdexcept> //for runtime_error
#include <cmath> //for isnan
#include <array>
//#include <atomic>
//#include <memory> //for unique_ptr
//#include <utility> //for std::pair
#include <functional>
//#include <thread>
#include "common/logging.h"
#include "gparse/command.h"
#include "gparse/com.h"
#include "event.h"
#include "scheduler.h"
#include "motion/motionplanner.h"
#include "motion/exponentialacceleration.h"
#include "drivers/driver.h"
#include "common/mathutil.h"
#include "drivers/axisstepper.h"
#include "drivers/iodriver.h"
#include "common/typesettings.h"
#include "common/tupleutil.h"

template <typename Drv> class State {
	//The scheduler needs to have certain callback functions, so we expose them without exposing the entire State:
	struct SchedInterface {
		State<Drv>& _state;
		SchedInterface(State<Drv> &state) : _state(state) {}
		void onEvent(const Event& evt) {
			_state.handleEvent(evt);
		}
		bool onIdleCpu() {
			return _state.satisfyIOs();
		}
		static constexpr std::size_t numIoDrivers() {
			return std::tuple_size<typename Drv::IODriverTypes>::value;
		}
	};
	//The MotionInterface needs certain information about the physical machine, so we provide that without exposing all of Drv:
	struct MotionInterface {
		typedef typename Drv::CoordMapT CoordMapT;
		typedef typename Drv::AxisStepperTypes AxisStepperTypes;
	};
	typedef Scheduler<SchedInterface> SchedType;
	//std::atomic<bool> _isDeadOrDying; //for thread destruction upon death.
	PositionMode _positionMode; // = POS_ABSOLUTE;
	PositionMode _extruderPosMode; // = POS_RELATIVE; //set via M82 and M83
	LengthUnit unitMode; // = UNIT_MM;
	float _destXPrimitive, _destYPrimitive, _destZPrimitive;
	float _destEPrimitive;
	float _destMoveRatePrimitive;
	float _hostZeroX, _hostZeroY, _hostZeroZ, _hostZeroE; //the host can set any arbitrary point to be referenced as 0.
	//std::array<int, Drv::CoordMapT::numAxis()> _destMechanicalPos; //number of steps for each stepper motor.
	gparse::Com &com;
	SchedType scheduler;
	MotionPlanner<MotionInterface, typename Drv::AccelerationProfileT> motionPlanner;
	Drv &driver;
	typename Drv::IODriverTypes ioDrivers;
	//bool _isExecutingGCode; //cannot schedule two movements simultaneously, so this serves as a lock
	//std::thread schedthread;
	public:
	    //so-called "Primitive" units represent a cartesian coordinate from the origin, using some primitive unit (mm)
		static constexpr CelciusType DEFAULT_HOTEND_TEMP() { return -300; } // < absolute 0
		static constexpr CelciusType DEFAULT_BED_TEMP() { return -300; }
		State(Drv &drv, gparse::Com &com);
		/* Control interpretation of positions from the host as relative or absolute */
		PositionMode positionMode() const;
		void setPositionMode(PositionMode mode);
		/* Control interpretation of *extruder* positions from the host as relative or absolute.
		 *If not explicitly set, it will default to the same as the XYZ position mode. */
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
		/* Control the move rate (AKA "feed rate") */
		float destMoveRatePrimitive() const;
		void setDestMoveRatePrimitive(float f);
		/* The host can set the current physical position to be a reference to an arbitrary point (like 0) */
		void setHostZeroPos(float x, float y, float z, float e);
		/* Processes the event immediately, eg stepping a stepper motor */
		void handleEvent(const Event &evt);
		/* Reads inputs of any IODrivers, and possible does something with the value (eg feedback loop between thermistor and hotend PWM control */
		bool satisfyIOs();
		void eventLoop();
		/* execute the GCode on a Driver object that supports a well-defined interface.
		 * returns a Command to send back to the host. */
		gparse::Command execute(gparse::Command const& cmd);
		/* Calculate and schedule a movement to absolute-valued x, y, z, e coords from the last queued position */
		void queueMovement(float x, float y, float z, float e);
		/* Home to the endstops. Does not return until endstops have been reached. */
		void homeEndstops();
		/* Set the hotend fan to a duty cycle between 0.0 and 1.0 */
		void setFanRate(float rate);
	private:
		/* Used internally to communicate step event times with the scheduler when moving or homing */
		//float transformEventTime(float time, float moveDuration, float Vmax);
		//template <typename AxisStepperTypes> void scheduleAxisSteppers(AxisStepperTypes &iters, float duration, bool accelerate, float maxVel=NAN);
};


template <typename Drv> State<Drv>::State(Drv &drv, gparse::Com &com)// : _isDeadOrDying(false), 
	: _positionMode(POS_ABSOLUTE), _extruderPosMode(POS_UNDEFINED),  
	unitMode(UNIT_MM), 
	_destXPrimitive(0), _destYPrimitive(0), _destZPrimitive(0), _destEPrimitive(0),
	_hostZeroX(0), _hostZeroY(0), _hostZeroZ(0), _hostZeroE(0),
	//_destMechanicalPos(), 
	com(com), 
	scheduler(SchedInterface(*this)),
	driver(drv)
	//_isExecutingGCode(false)
	{
	this->setDestMoveRatePrimitive(this->driver.defaultMoveRate());
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
			return mathutil::MM_PER_IN * posUnit;
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
	return posUnitToMM(posUnit/60); //feed rate is always relative, so no need to call toAbsolute. It is also given in mm/minute
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
	this->_destMoveRatePrimitive = this->driver.clampMoveRate(f);
}

template <typename Drv> void State<Drv>::setHostZeroPos(float x, float y, float z, float e) {
	//want it such that xUnitToPrimitive(x) (new) == _destXPrimitive (old)
	//note that x, y, z, e are already in mm.
	//thus, x + _hostZeroX (new) == _destXPrimitive
	//so, _hostZeroX = _destXPrimitive - x
	_hostZeroX = destXPrimitive() - x;
	_hostZeroY = destYPrimitive() - y;
	_hostZeroZ = destZPrimitive() - z;
	_hostZeroE = destEPrimitive() - e;
	//What x value makes _hostZeroX (new) == _hostZeroX (old) ?
	//_destXPrimitive - x = _hostZeroX
	//x = _destXPrimitive - _hostZeroX;
}

template <typename Drv> void State<Drv>::handleEvent(const Event &evt) {
	//handle an event from the scheduler.
	LOGV("State::handleEvent(time, idx, dir): %ld, %i, %i\n", evt.time().time_since_epoch().count(), evt.stepperId(), evt.direction()==StepForward);
	if (evt.direction() == StepForward) {
		drv::IODriver::selectAndStepForward(this->ioDrivers, evt.stepperId());
	} else {
		drv::IODriver::selectAndStepBackward(this->ioDrivers, evt.stepperId());
	}
}
template <typename Drv> bool State<Drv>::satisfyIOs() {
	/*if (!_isExecutingGCode && com.tendCom()) {
		_isExecutingGCode = true;
		com.reply(execute(com.getCommand()));
		_isExecutingGCode = false;
	}*/
	if (com.tendCom()) {
		//note: may want to optimize this; once there is a pending command, this involves a lot of extra work.
		gparse::Command resp = execute(com.getCommand());
		if (!resp.empty()) { //returning Command::Null means we're not ready to handle the command.
			com.reply(resp);
		}
	}
	bool motionNeedsCpu = false;
	if (scheduler.isRoomInBuffer()) {
		//LOGV("State::satisfyIOs, sched has buffer room\n");
		Event evt;
		if (!(evt = motionPlanner.nextStep()).isNull()) {
			this->scheduler.queue(evt);
			motionNeedsCpu = scheduler.isRoomInBuffer();
		} else {
			this->scheduler.setBufferSizeToDefault();
		}
	}
	bool driversNeedCpu = drv::IODriver::callIdleCpuHandlers<typename Drv::IODriverTypes, SchedType&>(this->ioDrivers, this->scheduler);
	return motionNeedsCpu || driversNeedCpu;
}

template <typename Drv> void State<Drv>::eventLoop() {
	this->scheduler.initSchedThread();
	this->scheduler.eventLoop();
}

template <typename Drv> gparse::Command State<Drv>::execute(gparse::Command const& cmd) {
	std::string opcode = cmd.getOpcode();
	//gparse::Command resp;
	if (cmd.isG0() || cmd.isG1()) { //rapid movement / controlled (linear) movement (currently uses same code)
		//LOGW("Warning (gparse/state.h): OP_G0/1 (linear movement) not fully implemented - notably extrusion\n");
		if (!motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
			return gparse::Command::Null;
		}
	    bool hasX, hasY, hasZ, hasE;
	    bool hasF;
	    float curX = destXPrimitive();
	    float curY = destYPrimitive();
	    float curZ = destZPrimitive();
	    float curE = destEPrimitive();
		float x = cmd.getX(hasX); //new x-coordinate.
		float y = cmd.getY(hasY); //new y-coordinate.
		float z = cmd.getZ(hasZ); //new z-coordinate.
		float e = cmd.getE(hasE); //extrusion amount.
		float f = cmd.getF(hasF); //feed-rate (XYZ move speed)
		x = hasX ? xUnitToPrimitive(x) : curX;
		y = hasY ? yUnitToPrimitive(y) : curY;
		z = hasZ ? zUnitToPrimitive(z) : curZ;
		e = hasE ? eUnitToPrimitive(e) : curE;
		if (hasF) {
			//this->setDestFeedRatePrimitive(fUnitToPrimitive(f));
			this->setDestMoveRatePrimitive(fUnitToPrimitive(f));
		}
		this->queueMovement(x, y, z, e);
		return gparse::Command::OK;
	} else if (cmd.isG20()) { //g-code coordinates will now be interpreted as inches
		setUnitMode(UNIT_IN);
		return gparse::Command::OK;
	} else if (cmd.isG21()) { //g-code coordinates will now be interpreted as millimeters.
		setUnitMode(UNIT_MM);
		return gparse::Command::OK;
	} else if (cmd.isG28()) { //home to end-stops / zero coordinates
		if (!motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
			return gparse::Command::Null;
		}
		this->homeEndstops();
		/*bool homeX = cmd.hasX(); //can optionally specify specific axis to home.
		bool homeY = cmd.hasY();
		bool homeZ = cmd.hasZ();
		if (!homeX && !homeY && !homeZ) { //if no axis are passed, then home ALL axis.
			homeX = homeY = homeZ = true;
		}
		float newX = homeX ? 0 : destXPrimitive();
		float newY = homeY ? 0 : destYPrimitive();
		float newZ = homeZ ? 0 : destZPrimitive();
		float curE = destEPrimitive();
		//this->queueMovement(newX, newY, newZ, curE);*/
		return gparse::Command::OK;
	} else if (cmd.isG90()) { //set g-code coordinates to absolute
		setPositionMode(POS_ABSOLUTE);
		return gparse::Command::OK;
	} else if (cmd.isG91()) { //set g-code coordinates to relative
		setPositionMode(POS_RELATIVE);
		return gparse::Command::OK;
	} else if (cmd.isG92()) { //set current position = 0
		//LOG("Warning (gparse/state.h): OP_G92 (set current position as reference to zero) not tested\n");
		float actualX, actualY, actualZ, actualE;
		bool hasXYZE = cmd.hasAnyXYZEParam();
		if (!hasXYZE) { //make current position (0, 0, 0, 0)
			actualX = actualY = actualZ = actualE = posUnitToMM(0);
		} else {
			actualX = cmd.hasX() ? posUnitToMM(cmd.getX()) : destXPrimitive() - _hostZeroX; //_hostZeroX;
			actualY = cmd.hasY() ? posUnitToMM(cmd.getY()) : destYPrimitive() - _hostZeroY; //_hostZeroY;
			actualZ = cmd.hasZ() ? posUnitToMM(cmd.getZ()) : destZPrimitive() - _hostZeroZ; //_hostZeroZ;
			actualE = cmd.hasE() ? posUnitToMM(cmd.getE()) : destEPrimitive() - _hostZeroE; //_hostZeroE;
		}
		setHostZeroPos(actualX, actualY, actualZ, actualE);
		return gparse::Command::OK;
	} else if (cmd.isM17()) { //enable all stepper motors
		LOGW("Warning (gparse/state.h): OP_M17 (enable stepper motors) not tested\n");
		drv::IODriver::lockAllAxis(this->ioDrivers);
		return gparse::Command::OK;
	} else if (cmd.isM18()) { //allow stepper motors to move 'freely'
		LOGW("Warning (gparse/state.h): OP_M18 (disable stepper motors) not tested\n");
		drv::IODriver::unlockAllAxis(this->ioDrivers);
		return gparse::Command::OK;
	} else if (cmd.isM21()) { //initialize SD card (nothing to do).
		return gparse::Command::OK;
	} else if (cmd.isM82()) { //set extruder absolute mode
		setExtruderPosMode(POS_ABSOLUTE);
		return gparse::Command::OK;
	} else if (cmd.isM83()) { //set extruder relative mode
		setExtruderPosMode(POS_RELATIVE);
		return gparse::Command::OK;
	} else if (cmd.isM84()) { //stop idle hold: relax all motors.
		LOGW("Warning (gparse/state.h): OP_M84 (stop idle hold) not implemented\n");
		return gparse::Command::OK;
	} else if (cmd.isM104()) { //set hotend temperature and return immediately.
		bool hasS;
		float t = cmd.getS(hasS);
		if (hasS) {
			drv::IODriver::setHotendTemp(ioDrivers, t);
		}
		return gparse::Command::OK;
	} else if (cmd.isM105()) { //get temperature, in C
		//CelciusType t=DEFAULT_HOTEND_TEMP(), b=DEFAULT_BED_TEMP(); //a temperature < absolute zero means no reading available.
		//driver.getTemperature(t, b);
		CelciusType t, b;
		//std::tie(t, b) = driver.getTemperature();
		t = drv::IODriver::getHotendTemp(ioDrivers);
		b = drv::IODriver::getBedTemp(ioDrivers);
		return gparse::Command("ok T:" + std::to_string(t) + " B:" + std::to_string(b));
	} else if (cmd.isM106()) { //set fan speed. Takes parameter S. Can be 0-255 (PWM) or in some implementations, 0.0-1.0
		float s = cmd.getS(1.0); //PWM duty cycle
		if (s > 1) { //host thinks we're working from 0 to 255
			s = s/256.0; //TODO: move this logic into cmd.getSNorm()
		}
		setFanRate(s);
		return gparse::Command::OK;
	} else if (cmd.isM107()) { //set fan = off.
		setFanRate(0);
		return gparse::Command::OK;
	} else if (cmd.isM109()) { //set extruder temperature to S param and wait.
		LOGW("Warning (gparse/state.h): OP_M109 (set extruder temperature and wait) not implemented\n");
		return gparse::Command::OK;
	} else if (cmd.isM110()) { //set current line number
		return gparse::Command::OK;
	} else if (cmd.isM117()) { //print message
		return gparse::Command::OK;
	} else if (cmd.isM140()) { //set BED temp and return immediately.
		LOGW("Warning (gparse/state.h): OP_M140 (set bed temp) is untested\n");
		bool hasS;
		float t = cmd.getS(hasS);
		if (hasS) {
			drv::IODriver::setBedTemp(ioDrivers, t);
		}
		return gparse::Command::OK;
	} else if (cmd.isTxxx()) { //set tool number
		LOGW("Warning (gparse/state.h): OP_T[n] (set tool number) not implemented\n");
		return gparse::Command::OK;
	} else {
		throw std::runtime_error(std::string("unrecognized gcode opcode: '") + cmd.getOpcode() + "'");
	}
}

/*template <typename Drv> float State<Drv>::transformEventTime(float time, float moveDuration, float Vmax) {
	//Note: it is assumed that the original path is already coded for constant velocity = Vmax.
	float Amax = this->driver.maxAccel();
	float V0 = std::min(0.5*Vmax, 0.1); //c becomes invalid if V0 >= Vmax
	float k = 4*Amax/Vmax;
	float c = V0 / (Vmax-V0);
	if (time > 0.5*moveDuration) {
		return 2*transformEventTime(0.5*moveDuration, moveDuration, Vmax) - transformEventTime(moveDuration-time, moveDuration, Vmax);
	} else { //take advantage of the fact that comparisons against NaN always compare false to allow for indefinite movements:
		//the problem with the below equation is that it can return infinity if k/Vmax*time is sufficiently large.
		//return 1./k * log(1./c * ((1. + c)*exp(k/Vmax*time) - 1.));
		//aka: 1./k*( log(1./c) + log((1. + c)*exp(k/Vmax*time) - 1.))
		//simplify: 1./k*log(e^x-1) ~=~ 1./k*x at x = Log[1 - E^(-k*.001)], at which point it is only .001 off (however, 1ms is significant! Would rather use a smaller value.
		auto logparam = (1. + c)*exp(k*time) - 1;
		if (std::isfinite(logparam)) {
			return 1./k*( log(1./c) + log(logparam));
		} else { //use the approximation:
			return 1./k*(log(1./c) + log(1. + c) + k*time);
		}
	}
}

template <typename Drv> template <typename AxisStepperTypes> void State<Drv>::scheduleAxisSteppers(AxisStepperTypes &iters, float duration, bool accelerate, float maxVel) {
	//Information on acceleration: http://reprap.org/wiki/Firmware/Linear_Acceleration
	//Current implementation uses instantaneous acceleration, which is physically impossible.
	//The best course *appears* to be an exponential velocity curve.
	//An attempt at that is made in proof-of-concept/acceleration.nb
	if (Drv::CoordMapT::numAxis() == 0) { 
		return; //some of the following logic may assume that there are at least 1 axis.
	}
	timespec baseTime = scheduler.lastSchedTime();
	do {
		drv::AxisStepper& s = drv::AxisStepper::getNextTime(iters);
		LOGV("Next step: %i at %g of %g\n", s.index(), s.time, duration);
		//if (s.time > duration || gmath::ltepsilon(s.time, 0, gmath::NANOSECOND)) { 
		if (s.time > duration || s.time <= 0 || std::isnan(s.time)) { //don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
			break; 
		}
		float transformedTime = accelerate ? transformEventTime(s.time, duration, maxVel) : s.time;
		LOGV("Step transformed time: %f\n", transformedTime);
		Event e = s.getEvent(transformedTime);
		e.offset(baseTime);
		scheduler.queue(e);
		_destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction);
		s.nextStep(iters);
	} while (1);
}*/
		
template <typename Drv> void State<Drv>::queueMovement(float x, float y, float z, float e) {
	/*float curX, curY, curZ, curE; 
	curX = destXPrimitive(); //could add motionPlanner::realXyze() for slightly increased accuracy, but that adds more interdependencies.
	curY = destYPrimitive();
	curZ = destZPrimitive();
	curE = destEPrimitive();*/
	_destXPrimitive = x;
	_destYPrimitive = y;
	_destZPrimitive = z;
	_destEPrimitive = e;
	//now determine the velocity (must ensure xyz velocity doesn't cause too much E velocity):
	float velXyz = destMoveRatePrimitive();
	/*if (curE != e) { //This if-statement avoids a division-by-zero caused by when velE == 0
		float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
		float distXyz = sqrt(distSq);
		float durationXyz = distXyz / velXyz;
		float velE = (e-curE)/durationXyz;
		float newVelE = this->driver.clampExtrusionRate(velE);
		velXyz *= newVelE / velE;
	}*/
	float minExtRate = -this->driver.maxRetractRate();
	float maxExtRate = this->driver.maxExtrudeRate();
	motionPlanner.moveTo(scheduler.lastSchedTime(), x, y, z, e, velXyz, minExtRate, maxExtRate);
	/*float curX, curY, curZ, curE;
	std::tie(curX, curY, curZ, curE) = Drv::CoordMapT::xyzeFromMechanical(_destMechanicalPos);
	_destXPrimitive = x;
	_destYPrimitive = y;
	_destZPrimitive = z;
	_destEPrimitive = e;
	float maxVelXyz = destMoveRatePrimitive(); //the maximum velocity this path will be coded for.
	float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
	float dist = sqrt(distSq);
	float minDuration = dist/maxVelXyz; //duration, should there be no acceleration
	float velE = (e-curE)/minDuration;
	float newVelE = this->driver.clampExtrusionRate(velE);
	if (velE != newVelE) { //in the case that newXYZ = currentXYZ, but extrusion is different, regulate that.
		velE = newVelE;
		minDuration = (e-curE)/newVelE; //L/(L/t) = t
		maxVelXyz = dist/minDuration;
	}
	float vx = (x-curX)/minDuration;
	float vy = (y-curY)/minDuration;
	float vz = (z-curZ)/minDuration;
	LOGD("State::queueMovement (%f, %f, %f, %f) -> (%f, %f, %f, %f)\n", curX, curY, curZ, curE, x, y, z, e);
	LOGD("State::queueMovement _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
	LOGD("State::queueMovement V:%f, vx:%f, vy:%f, vz:%f, ve:%f dur:%f\n", maxVelXyz, vx, vy, vz, velE, minDuration);
	typename Drv::AxisStepperTypes iters;
	drv::AxisStepper::initAxisSteppers(iters, _destMechanicalPos, vx, vy, vz, velE);
	this->scheduleAxisSteppers(iters, minDuration, true, maxVelXyz);
	std::tie(curX, curY, curZ, curE) = Drv::CoordMapT::xyzeFromMechanical(_destMechanicalPos);
	LOGD("State::queueMovement wanted (%f, %f, %f, %f) got (%f, %f, %f, %f)\n", x, y, z, e, curX, curY, curZ, curE);
	LOGD("State::queueMovement _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);*/
}

template <typename Drv> void State<Drv>::homeEndstops() {
	this->scheduler.setBufferSize(this->scheduler.numActivePwmChannels()+1);
	motionPlanner.homeEndstops(scheduler.lastSchedTime(), this->driver.clampHomeRate(destMoveRatePrimitive()));
}

/* State utility class for setting the fan rate (State::setFanRate).
Note: could be replaced with a generic lambda in C++14 (gcc-4.9) */
template <typename SchedT> struct State_setFanRate {
	SchedT &sched;
	float rate;
	//State_setFanRate() {}
	State_setFanRate(SchedT &s, float rate) : sched(s), rate(rate) {}
	template <typename T> void operator()(std::size_t index, const T &f) {
		if (f.isFan()) {
			sched.schedPwm(index, PwmInfo(rate, f.fanPwmPeriod()));
		}
	}
};

template <typename Drv> void State<Drv>::setFanRate(float rate) {
	callOnAll(ioDrivers, State_setFanRate<SchedType>(scheduler, rate));
}

#endif
