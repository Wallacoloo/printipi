/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* 
 * Printipi/state.h
 *
 * State handles as much driver-mutual functionality as possible, including mapping Gcodes to specific functions,
 *   tracking unit mode and axis position, and interfacing with the scheduler.
 * State controls the communications channel, the scheduler, and the underlying driver.
 * Motion planning is offloaded to src/motion/MotionPlanner
 */

#ifndef STATE_H
#define STATE_H

//Gcode documentation can be found:
//  http://www.nist.gov/customcf/get_pdf.cfm?pub_id=823374
//  http://reprap.org/wiki/G-code
//  or (with implementation): https://github.com/Traumflug/Teacup_Firmware/blob/master/gcode_process.c
//  Marlin-specific: http://www.ctheroux.com/2012/11/g-code-commands-supported-by-marlin/
//  Clarification of E and F: http://forums.reprap.org/read.php?263,208245
//    E is the extruder coordinate. F is the "feed rate", which is really the rate at which X, Y, Z moves.

#include <string>
#include <cstddef> //for size_t
#include <stdexcept> //for runtime_error
#include <cmath> //for isnan
#include <utility> //for std::declval
#include <array>
#include <stack>
#include "common/logging.h"
#include "gparse/command.h"
#include "gparse/com.h"
#include "gparse/response.h"
#include "event.h"
#include "scheduler.h"
#include "motion/motionplanner.h"
#include "common/mathutil.h"
#include "drivers/iodriver.h"
#include "drivers/auto/chronoclock.h" //for EventClockT
#include "drivers/auto/hardwarescheduler.h" //for SchedInterfaceHardwareScheduler
#include "compileflags.h" //for CelciusType
#include "common/tupleutil.h"
#include "filesystem.h"
#include "outputevent.h"

enum PositionMode {
    POS_ABSOLUTE,
    POS_RELATIVE
};

enum LengthUnit {
    UNIT_MM,
    UNIT_IN
};

template <typename Drv> class State {
    //The scheduler needs to have certain callback functions, so we expose them without exposing the entire State by defining a SchedInterface object:
    struct SchedInterface {
        private:
            State<Drv>& _state;
            SchedInterfaceHardwareScheduler _hardwareScheduler; //configured in typesettings.h
        public:
            SchedInterface(State<Drv> &state) : _state(state) {}
            bool onIdleCpu(OnIdleCpuIntervalT interval) {
                //relay onIdleCpu event to hardware scheduler & state.
                //return true if either one requests more cpu time. 
                bool hwNeedsCpu = _hardwareScheduler.onIdleCpu(interval);
                bool stateNeedsCpu = _state.onIdleCpu(interval);
                return hwNeedsCpu || stateNeedsCpu;
            }
            struct __iterPwmPins {
                template <typename T, typename Func> void operator()(T &driver, float dutyCycle, Func &f) {
                    auto p = driver.getPwmPin();
                    f(p.id(), p.areWritesInverted() ? 1-dutyCycle : dutyCycle);
                }
            };
            template <typename Func> void iterPwmPins(AxisIdType axis, float dutyCycle, Func f) {
                //call a function, f, for each pin owned by the driver stored at index 'axis' in the ioDrivers object
                return tupleCallOnIndex(_state.ioDrivers, __iterPwmPins(), axis, dutyCycle, f);
            }
            inline void queue(const OutputEvent &evt) {
                //schedule an event to happen at some time in the future (relay message to hardware scheduler)
                _hardwareScheduler.queue(evt);
            }
            inline void queuePwm(int pin, float duty, float maxPeriod) {
                //configure hardware pin with id 'pin' for PWM, and ensure the PWM period is < maxPeriod (if possible)
                _hardwareScheduler.queuePwm(pin, duty, maxPeriod);
            }
            EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
                //if an event is to occur at evtTime, then return the soonest that we are capable of scheduling it in hardware (we may have limited buffers, etc).
                return _hardwareScheduler.schedTime(evtTime);
            }
    };
    //The MotionPlanner needs certain information about the physical machine, so we provide that without exposing all of Drv:
    class MotionInterface {
        State<Drv> &state;
        public:
            //Derive the types for various Machine-specific subtypes:
            //Do this by applying decltype on the functions that create these types.
            //Note: In order to avoid any assumptions about the Machine's constructor, but still be able to access its member functions,
            //  we use declval<Drv>() to create a dummy instance of the Machine.
            typedef decltype(std::declval<Drv>().getCoordMap()) CoordMapT;
            typedef decltype(std::declval<Drv>().getAxisSteppers()) AxisStepperTypes;
            typedef decltype(std::declval<Drv>().getHomeSteppers()) AxisHomeStepperTypes;
            typedef decltype(std::declval<Drv>().getArcSteppers()) AxisArcStepperTypes;
            typedef decltype(std::declval<Drv>().getAccelerationProfile()) AccelerationProfileT;
            MotionInterface(State<Drv> &state) : state(state) {}
            AccelerationProfileT getAccelerationProfile() const {
                return state.driver.getAccelerationProfile();
            }
            CoordMapT getCoordMap() const {
                return state.driver.getCoordMap();
            }
            AxisStepperTypes getAxisSteppers() const {
                return state.driver.getAxisSteppers();
            }
            AxisHomeStepperTypes getHomeSteppers() const {
                return state.driver.getHomeSteppers();
            }
            AxisArcStepperTypes getArcSteppers() const {
                return state.driver.getArcSteppers();
            }
    };
    typedef Scheduler<SchedInterface> SchedType;
    typedef decltype(std::declval<Drv>().getIoDrivers()) IODriverTypes;
    PositionMode _positionMode; // = POS_ABSOLUTE;
    PositionMode _extruderPosMode; // = POS_RELATIVE; //set via M82 and M83
    LengthUnit unitMode; // = UNIT_MM;
    float _destXPrimitive, _destYPrimitive, _destZPrimitive;
    float _destEPrimitive;
    float _destMoveRatePrimitive;
    float _hostZeroX, _hostZeroY, _hostZeroZ, _hostZeroE; //the host can set any arbitrary point to be referenced as 0.
    bool _isHomed; //Need to know if our absolute coordinates are accurate, which changes when power is lost or stepper motors are deactivated.
    bool _isWaitingForHotend;
    EventClockT::time_point _lastMotionPlannedTime;
    gparse::Com com;
    //M32 allows a gcode file to call subroutines, essentially.
    //  These subroutines can then call more subroutines, so what we have is essentially a call stack.
    //  We only read the top file on the stack, until it's done, and then pop it and return to the next one.
    //BUT, we still need to maintain a com channel to the host (especially for emergency stop, etc).
    //Thus, we need a root com ("com") & an additional file stack ("gcodeFileStack").
    std::stack<gparse::Com> gcodeFileStack;
    SchedType scheduler;
    MotionPlanner<MotionInterface> motionPlanner;
    Drv &driver;
    FileSystem &filesystem;
    IODriverTypes ioDrivers;
    public:
        //so-called "Primitive" units represent a cartesian coordinate from the origin, using some primitive unit (mm)
        static constexpr CelciusType DEFAULT_HOTEND_TEMP() { return -300; } // < absolute 0
        static constexpr CelciusType DEFAULT_BED_TEMP() { return -300; }
        //Initialize the state:
        //  Needs a driver object (drv), a communications channel (com), and needs to know whether or not the com channel must be persistent
        //  M32 command allows branching to another, local gcode file. By default, this will PAUSE reading/writing from the previous com channel.
        //  But if we want to continue reading from that original com channel while simultaneously reading from the new gcode file, then 'needPersistentCom' should be set to true.
        //  This is normally only relevant for communication with a host, like Octoprint, where we want temperature reading, emergency stop, etc to still work.
        State(Drv &drv, FileSystem &fs, gparse::Com com, bool needPersistentCom);
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
        //void handleEvent(const Event &evt);
        /* Reads inputs of any IODrivers, and possible does something with the value (eg feedback loop between thermistor and hotend PWM control */
        bool onIdleCpu(OnIdleCpuIntervalT interval);
        void eventLoop();
        void tendComChannel(gparse::Com &com);
        /* execute the GCode on a Driver object that supports a well-defined interface.
         * returns a Command to send back to the host. */
        gparse::Response execute(gparse::Command const& cmd, gparse::Com &com);
        // make an arc from the current position to (x, y, z), maintaining a constant distance from (cX, cY, cZ)
        void queueArc(float x, float y, float z, float e, float cX, float cY, float cZ, bool isCW=false);
        /* Calculate and schedule a movement to absolute-valued x, y, z, e coords from the last queued position */
        void queueMovement(float x, float y, float z, float e);
        /* Home to the endstops. */
        void homeEndstops();
        //Check if M109 (set temperature and wait until reached) has been satisfied.
        bool isHotendReady();
        /* Set the hotend fan to a duty cycle between 0.0 and 1.0 */
        void setFanRate(float rate);
};


template <typename Drv> State<Drv>::State(Drv &drv, FileSystem &fs, gparse::Com com, bool needPersistentCom)
    : _positionMode(POS_ABSOLUTE), _extruderPosMode(POS_ABSOLUTE),  
    unitMode(UNIT_MM), 
    _destXPrimitive(0), _destYPrimitive(0), _destZPrimitive(0), _destEPrimitive(0),
    _hostZeroX(0), _hostZeroY(0), _hostZeroZ(0), _hostZeroE(0),
    _isHomed(false),
    _isWaitingForHotend(false),
    _lastMotionPlannedTime(std::chrono::seconds(0)), 
    scheduler(SchedInterface(*this)),
    motionPlanner(MotionInterface(*this)),
    driver(drv),
    filesystem(fs),
    ioDrivers(drv.getIoDrivers())
    {
    this->setDestMoveRatePrimitive(this->driver.defaultMoveRate());
    if (needPersistentCom) {
        this->com = com;
    } else {
        this->gcodeFileStack.push(com);
    }
}


template <typename Drv> PositionMode State<Drv>::positionMode() const {
    return this->_positionMode;
}
template <typename Drv> void State<Drv>::setPositionMode(PositionMode mode) {
    this->_positionMode = mode; 
}

template <typename Drv> PositionMode State<Drv>::extruderPosMode() const {
    return this->_extruderPosMode;
}
template <typename Drv> void State<Drv>::setExtruderPosMode(PositionMode mode) {
    this->_extruderPosMode = mode;
}

template <typename Drv> void State<Drv>::setUnitMode(LengthUnit mode) {
    this->unitMode = mode;
}

template <typename Drv> float State<Drv>::xUnitToAbsolute(float posUnit) const {
    //if we're set to interpret coordinates as relative, then translate to absolute:
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
    //if we're set to interpret coordinates as relative, then translate to absolute:
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
    //if we're set to interpret coordinates as relative, then translate to absolute:
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
    //if we're set to interpret coordinates as relative, then translate to absolute:
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
    //If we're set to interpret coordinates as inches, then convert to mm:
    switch (this->unitMode) {
        case UNIT_IN:
            return mathutil::MM_PER_IN * posUnit;
        case UNIT_MM:
        default: //impossible case.
            return posUnit;
    }
}

template <typename Drv> float State<Drv>::xUnitToPrimitive(float posUnit) const {
    //Convert the gcode unit to absolute MM, compensated for the zero setpoint
    return posUnitToMM(xUnitToAbsolute(posUnit)) + this->_hostZeroX;
}
template <typename Drv> float State<Drv>::yUnitToPrimitive(float posUnit) const {
    //Convert the gcode unit to absolute MM, compensated for the zero setpoint
    return posUnitToMM(yUnitToAbsolute(posUnit)) + this->_hostZeroY;
}
template <typename Drv> float State<Drv>::zUnitToPrimitive(float posUnit) const {
    //Convert the gcode unit to absolute MM, compensated for the zero setpoint
    return posUnitToMM(zUnitToAbsolute(posUnit)) + this->_hostZeroZ;
}
template <typename Drv> float State<Drv>::eUnitToPrimitive(float posUnit) const {
    //Convert the gcode unit to absolute MM, compensated for the zero setpoint
    return posUnitToMM(eUnitToAbsolute(posUnit)) + this->_hostZeroE;
}
template <typename Drv> float State<Drv>::fUnitToPrimitive(float posUnit) const {
    return posUnitToMM(posUnit/60); //feed rate is given in mm/minute, or in/minute.
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

struct __iterEventOutputSequence {
    template <typename T, typename Func> void operator()(T &driver, const Event &evt, Func &f) {
        auto a = driver.getEventOutputSequence(evt); //get the output events for this events
        for (auto &&outputEvt : a) {
            f(outputEvt); //apply to f.
        }
    }
};

template <typename Drv> bool State<Drv>::onIdleCpu(OnIdleCpuIntervalT interval) {
    //Only check the communications periodically because calling execute(com.getCommand()) DOES add up.
    //One could swap the interval check with the com.tendCom() if running on a system incapable of buffering a full line of g-code.
    if (interval == OnIdleCpuIntervalWide) {
        tendComChannel(com);
        if (!gcodeFileStack.empty()) {
            LOGV("Tending gcodeFileStack top\n");
            tendComChannel(gcodeFileStack.top());
        }
    }
    bool motionNeedsCpu = false;
    if (scheduler.isRoomInBuffer()) { 
        Event evt; //check to see if motionPlanner has another event ready
        if (!motionPlanner.isHoming() || _lastMotionPlannedTime <= EventClockT::now()) { //if we're homing, we don't want to queue the next step until the current one has actually completed.
            if (!(evt = motionPlanner.nextStep()).isNull()) {
                tupleCallOnIndex(this->ioDrivers, __iterEventOutputSequence(), evt.stepperId(), evt, [this](const OutputEvent &out) { this->scheduler.queue(out); });
                _lastMotionPlannedTime = evt.time();
                motionNeedsCpu = scheduler.isRoomInBuffer();
            } else { //undo buffer-length changes set in homing
                this->scheduler.setDefaultMaxSleep();
            }
        }
    }
    bool driversNeedCpu = drv::IODriver::callIdleCpuHandlers<IODriverTypes, SchedType&>(this->ioDrivers, this->scheduler);
    return motionNeedsCpu || driversNeedCpu;
}

template <typename Drv> void State<Drv>::eventLoop() {
    this->scheduler.initSchedThread();
    this->scheduler.eventLoop();
}

template <typename Drv> void State<Drv>::tendComChannel(gparse::Com &com) {
    if (com.tendCom()) {
        //note: may want to optimize this; once there is a pending command, this involves a lot of extra work.
        auto cmd = com.getCommand();
        gparse::Response resp = execute(cmd, com);
        if (!resp.isNull()) { //returning Command::Null means we're not ready to handle the command.
            if (!NO_LOG_M105 || !cmd.isM105()) {
                LOG("command: %s\n", cmd.toGCode().c_str());
                LOG("response: %s", resp.toString().c_str());
            }
            com.reply(resp);
        }
    }
}

template <typename Drv> gparse::Response State<Drv>::execute(gparse::Command const &cmd, gparse::Com &com) {
    //process a gcode command received on the given communications channel and return an appropriate response
    std::string opcode = cmd.getOpcode();
    if (cmd.isG0() || cmd.isG1()) { //rapid movement / controlled (linear) movement (currently uses same code)
        if (!motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
            return gparse::Response::Null;
        }
        if (!isHotendReady()) { //make sure that a call to M109 doesn't allow movements until it's complete.
            return gparse::Response::Null;
        }
        if (!_isHomed && driver.doHomeBeforeFirstMovement()) {
            this->homeEndstops();
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
            this->setDestMoveRatePrimitive(fUnitToPrimitive(f));
        }
        this->queueMovement(x, y, z, e);
        return gparse::Response::Ok;
    //} else if (cmd.isG2()) { //clockwise arc from current position to (x, y, z) with center at (i, j, k)
    //    LOG("Clockwise arcs not supported; use G3 (CCW arc)\n");
    //    return gparse::Response::Ok;
    } else if (cmd.isG2() || cmd.isG3()) {
        LOGW("Warning: G3 is experimental\n");
        //first, get the end coordinate and optional feed-rate:
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
            this->setDestMoveRatePrimitive(fUnitToPrimitive(f));
        }
        //Now get the center-point coordinate:
        bool hasK; //center-z is optional.
        float i = xUnitToPrimitive(cmd.getI());
        float j = yUnitToPrimitive(cmd.getJ());
        float k = cmd.getK(hasK);
        k = hasK ? zUnitToPrimitive(k) : curZ;
        this->queueArc(x, y, z, e, i, j, k, cmd.isG2());
        return gparse::Response::Ok;
    } else if (cmd.isG20()) { //g-code coordinates will now be interpreted as inches
        setUnitMode(UNIT_IN);
        return gparse::Response::Ok;
    } else if (cmd.isG21()) { //g-code coordinates will now be interpreted as millimeters.
        setUnitMode(UNIT_MM);
        return gparse::Response::Ok;
    } else if (cmd.isG28()) { //home to end-stops / zero coordinates
        if (!motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
            return gparse::Response::Null;
        }
        if (!isHotendReady()) { //make sure that a call to M109 doesn't allow movements until it's complete.
            return gparse::Response::Null;
        }
        this->homeEndstops();
        return gparse::Response::Ok;
    } else if (cmd.isG90()) { //set g-code coordinates to absolute
        setPositionMode(POS_ABSOLUTE);
        setExtruderPosMode(POS_ABSOLUTE);
        return gparse::Response::Ok;
    } else if (cmd.isG91()) { //set g-code coordinates to relative
        setPositionMode(POS_RELATIVE);
        setExtruderPosMode(POS_RELATIVE);
        return gparse::Response::Ok;
    } else if (cmd.isG92()) { //set current position = 0
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
        return gparse::Response::Ok;
    } else if (cmd.isM0()) { //Stop; empty move buffer & exit cleanly
        LOG("recieved M0 command: exiting\n");
        exit(0);
        return gparse::Response::Ok;
    } else if (cmd.isM17()) { //enable all stepper motors
        drv::IODriver::lockAllAxis(this->ioDrivers);
        return gparse::Response::Ok;
    } else if (cmd.isM18()) { //allow stepper motors to move 'freely'
        drv::IODriver::unlockAllAxis(this->ioDrivers);
        return gparse::Response::Ok;
    } else if (cmd.isM21()) { //initialize SD card (nothing to do).
        return gparse::Response::Ok;
    } else if (cmd.isM32()) { //select file on SD card and print:
        LOGD("loading gcode: %s\n", cmd.getFilepathParam().c_str());
        gcodeFileStack.push(gparse::Com(filesystem.relGcodePathToAbs(cmd.getFilepathParam())));
        return gparse::Response::Ok;
    } else if (cmd.isM82()) { //set extruder absolute mode
        setExtruderPosMode(POS_ABSOLUTE);
        return gparse::Response::Ok;
    } else if (cmd.isM83()) { //set extruder relative mode
        setExtruderPosMode(POS_RELATIVE);
        return gparse::Response::Ok;
    } else if (cmd.isM84()) { //stop idle hold: relax all motors.
        LOGW("Warning (gparse/state.h): OP_M84 (stop idle hold) not implemented\n");
        return gparse::Response::Ok;
    } else if (cmd.isM99()) { //return from macro/subprogram
        LOGW("Warning (state.h): OP_M99 (return) not tested\n");
        //note: can't simply pop the top file, because then that causes memory access errors when trying to send it a reply.
        //Need to check if com channel that received this command is the top one. If yes, then pop it and return Response::Null so that no response will be sent.
        //  else, pop it and return Response::Ok.
        if (gcodeFileStack.empty()) { //return from the main I/O routine = kill program
            exit(0);
            return gparse::Response::Null;
        } else {
            if (&gcodeFileStack.top() == &com) { //popping the com channel that sent this = cannot reply
                //Note: MUST compare com to .top() before popping, otherwise com will become an invalid reference.
                //We can get away with comparing just the pointers, because com objects are only ever stored in one place.
                gcodeFileStack.pop();
                return gparse::Response::Null;
            } else { //popping a different com channel than the one that sent this request.
                gcodeFileStack.pop();
                return gparse::Response::Ok;
            }
        }
    } else if (cmd.isM104()) { //set hotend temperature and return immediately.
        bool hasS;
        float t = cmd.getS(hasS);
        if (hasS) {
            drv::IODriver::setHotendTemp(ioDrivers, t);
        }
        return gparse::Response::Ok;
    } else if (cmd.isM105()) { //get temperature, in C
        CelciusType t, b;
        t = drv::IODriver::getHotendTemp(ioDrivers);
        b = drv::IODriver::getBedTemp(ioDrivers);
        return gparse::Response(gparse::ResponseOk, "T:" + std::to_string(t) + " B:" + std::to_string(b));
    } else if (cmd.isM106()) { //set fan speed. Takes parameter S. Can be 0-255 (PWM) or in some implementations, 0.0-1.0
        float s = cmd.getS(1.0); //PWM duty cycle
        if (s > 1) { //host thinks we're working from 0 to 255
            s = s/256.0; //TODO: move this logic into cmd.getSNorm()
        }
        setFanRate(s);
        return gparse::Response::Ok;
    } else if (cmd.isM107()) { //set fan = off.
        setFanRate(0);
        return gparse::Response::Ok;
    } else if (cmd.isM109()) { //set extruder temperature to S param and wait.
        LOGW("Warning (gparse/state.h): OP_M109 (set extruder temperature and wait) not fully implemented\n");
        bool hasS;
        float t = cmd.getS(hasS);
        if (hasS) {
            drv::IODriver::setHotendTemp(ioDrivers, t);
        }
        _isWaitingForHotend = true;
        return gparse::Response::Ok;
    } else if (cmd.isM110()) { //set current line number
        LOGW("Warning (state.h): OP_M110 (set current line number) not implemented\n");
        return gparse::Response::Ok;
    } else if (cmd.isM112()) { //emergency stop
        exit(1);
        return gparse::Response::Ok;
    } else if (cmd.isM116()) { //Wait for all heaters (and slow moving variables) to reach target
        _isWaitingForHotend = true;
        return gparse::Response::Ok;
    } else if (cmd.isM117()) { //print message
        return gparse::Response::Ok;
    } else if (cmd.isM140()) { //set BED temp and return immediately.
        LOGW("Warning (gparse/state.h): OP_M140 (set bed temp) is untested\n");
        bool hasS;
        float t = cmd.getS(hasS);
        if (hasS) {
            drv::IODriver::setBedTemp(ioDrivers, t);
        }
        return gparse::Response::Ok;
    } else if (cmd.isTxxx()) { //set tool number
        LOGW("Warning (gparse/state.h): OP_T[n] (set tool number) not implemented\n");
        return gparse::Response::Ok;
    } else {
        throw std::runtime_error(std::string("unrecognized gcode opcode: '") + cmd.getOpcode() + "'");
    }
}

template <typename Drv> void State<Drv>::queueArc(float x, float y, float z, float e, float cX, float cY, float cZ, bool isCW) {
    //track the desired position to minimize drift over time caused by relative movements when we can't precisely reach the given coordinates:
    _destXPrimitive = x;
    _destYPrimitive = y;
    _destZPrimitive = z;
    _destEPrimitive = e;
    //now determine the velocity of the move. We just calculate limits & relay the info to the motionPlanner
    float velXyz = destMoveRatePrimitive();
    float minExtRate = -this->driver.maxRetractRate();
    float maxExtRate = this->driver.maxExtrudeRate();
    //start the next move at the time that the previous move is scheduled to complete, unless that time is in the past
    auto startTime = std::max(_lastMotionPlannedTime, EventClockT::now());
    motionPlanner.arcTo(startTime, x, y, z, e, cX, cY, cZ, velXyz, minExtRate, maxExtRate, isCW);
}
        
template <typename Drv> void State<Drv>::queueMovement(float x, float y, float z, float e) {
    //track the desired position to minimize drift over time caused by relative movements when we can't precisely reach the given coordinates:
    _destXPrimitive = x;
    _destYPrimitive = y;
    _destZPrimitive = z;
    _destEPrimitive = e;
    //now determine the velocity of the move. We just calculate limits & relay the info to the motionPlanner
    float velXyz = destMoveRatePrimitive();
    float minExtRate = -this->driver.maxRetractRate();
    float maxExtRate = this->driver.maxExtrudeRate();
    //start the next move at the time that the previous move is scheduled to complete, unless that time is in the past
    auto startTime = std::max(_lastMotionPlannedTime, EventClockT::now());
    motionPlanner.moveTo(startTime, x, y, z, e, velXyz, minExtRate, maxExtRate);
}

template <typename Drv> void State<Drv>::homeEndstops() {
    //homing is done with real-time scheduling (can't plan far ahead), so instruct the Scheduler to avoid long sleeps
    this->scheduler.setMaxSleep(std::chrono::milliseconds(1));
    motionPlanner.homeEndstops(std::max(_lastMotionPlannedTime, EventClockT::now()), this->driver.clampHomeRate(destMoveRatePrimitive()));
    this->_isHomed = true;
}

template <typename Drv> bool State<Drv>::isHotendReady() {
    if (_isWaitingForHotend) {
        //TODO: check ALL heaters, not just hotend.
        CelciusType current = drv::IODriver::getHotendTemp(ioDrivers);
        CelciusType target = drv::IODriver::getHotendTargetTemp(ioDrivers);
        _isWaitingForHotend = current < target;
    }
    return !_isWaitingForHotend;
}
/* State utility class for setting the fan rate (State::setFanRate).
Note: could be replaced with a generic lambda in C++14 (gcc-4.9) */
template <typename SchedT> struct State_setFanRate {
    SchedT &sched;
    float rate;
    State_setFanRate(SchedT &s, float rate) : sched(s), rate(rate) {}
    template <typename T> void operator()(std::size_t index, const T &f) {
        if (f.isFan()) {
            sched.schedPwm(index, rate, f.fanPwmPeriod());
        }
    }
};

template <typename Drv> void State<Drv>::setFanRate(float rate) {
    callOnAll(ioDrivers, State_setFanRate<SchedType>(scheduler, rate));
}

#endif
