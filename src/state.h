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
#include <vector>
#include "common/logging.h"
#include "gparse/command.h"
#include "gparse/com.h"
#include "gparse/response.h"
#include "scheduler.h"
#include "motion/motionplanner.h"
#include "common/mathutil.h"
#include "iodrivers/iodriver.h"
#include "iodrivers/iodrivers.h"
#include "platforms/auto/chronoclock.h" //for EventClockT
#include "platforms/auto/hardwarescheduler.h" //for HardwareScheduler
#include "iodrivers/iopin.h"
#include "compileflags.h" //for CelciusType
#include "common/tupleutil.h"
#include "filesystem.h"
#include "outputevent.h"
#include "common/vector4.h"
#include "common/optionalarg.h"

//g-code coordinates can either be interpreted as absolute or relative to the last coordinates received
enum PositionMode {
    POS_ABSOLUTE,
    POS_RELATIVE
};

//g-code coordinates can either be interpreted as having units of millimeters or units of inches
enum LengthUnit {
    UNIT_MM,
    UNIT_IN
};

/* 
 * State handles as much driver-mutual functionality as possible, including mapping Gcodes to specific functions,
 *   tracking unit mode and axis position, and interfacing with the scheduler.
 * State controls the communications channel, the scheduler, and the underlying driver.
 * Motion planning is offloaded to src/motion/MotionPlanner
 */
template <typename Drv> class State {
    friend struct TestClass;
    //Derive the types for various Machine-specific subtypes:
    //Do this by applying decltype on the functions that create these types.
    //Note: In order to avoid any assumptions about the Machine's constructor, but still be able to access its member functions,
    //  we use declval<Drv>() to create a dummy instance of the Machine.
    typedef decltype(std::declval<Drv>().getCoordMap()) CoordMapT;
    typedef decltype(std::declval<Drv>().getAccelerationProfile()) AccelerationProfileT;
    //The scheduler needs to have certain callback functions, so we expose them without exposing the entire State by defining a SchedInterface object:
    struct SchedInterface {
        private:
            State<Drv>& _state;
            HardwareScheduler _hardwareScheduler; 
        public:
            SchedInterface(State<Drv> &state) : _state(state) {}
            bool onIdleCpu(OnIdleCpuIntervalT interval) {
                //relay onIdleCpu event to hardware scheduler & state.
                //return true if either one requests more cpu time. 
                IntervalTimer timer;
                timer.clock();
                bool hwNeedsCpu = _hardwareScheduler.onIdleCpu(interval);
                LOGV("Time spent in _hardwareScheduler:onIdleCpu: %" PRId64 ", %i, ret %i\n", (int64_t)timer.clockDiff().count(), interval, hwNeedsCpu);
                bool stateNeedsCpu = _state.onIdleCpu(interval);
                LOGV("Time spent in state.h:onIdleCpu: %" PRId64 ", %i, ret %i\n", (int64_t)timer.clockDiff().count(), interval, stateNeedsCpu);
                return hwNeedsCpu || stateNeedsCpu;
            }
            inline void queue(const OutputEvent &evt) {
                //schedule an event to happen at some time in the future (relay message to hardware scheduler)
                _hardwareScheduler.queue(evt);
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
            //expose CoordMapT typedef to <MotionPlanner>
            typedef State<Drv>::CoordMapT CoordMapT;
            //expose AccelerationProfileT typedef to <MotionPlanner>
            typedef State<Drv>::AccelerationProfileT AccelerationProfileT;
            MotionInterface(State<Drv> &state) : state(state) {}
            AccelerationProfileT getAccelerationProfile() const {
                return state.driver.getAccelerationProfile();
            }
            CoordMapT getCoordMap() const {
                return state.driver.getCoordMap();
            }
    };
    //The CoordMap needs extra information when homing
    class CoordMapInterface {
        State<Drv> &state;
        public:
            CoordMapInterface(State<Drv> &state) : state(state) {}
            Vector4f actualCartesianPosition() const {
                return state.motionPlanner().actualCartesianPosition();
            }
            //blocking moveTo (linear cartesian movement with acceleration) function
            void moveTo(const Vector4f &position, OptionalArg<float> velXyz=OptionalArg<float>::NotPresent, const motion::MotionFlags flags=motion::MOTIONFLAGS_DEFAULT) {
                state.queueMovement(position, velXyz, flags);
                //retain control until the movement is complete.
                state._doExitEventLoopAfterMoveCompletes = true;
                state.eventLoop();
            }
            const std::array<int, CoordMapT::numAxis()> & axisPositions() const {
                return state.motionPlanner().axisPositions();
            }
            void resetAxisPositions(const std::array<int, CoordMapT::numAxis()> &pos) {
                state._motionPlanner.resetAxisPositions(pos);
            }
    };
    typedef Scheduler<SchedInterface> SchedType;
    //The ioDrivers are a combination of the ones used by the coordmap and the miscellaneous ones from the machine
    typedef iodrv::IODrivers<decltype(std::tuple_cat(
        std::declval<Drv>().getCoordMap().getDependentIoDrivers(), 
        std::declval<Drv>().getIoDrivers()))> IODriverTypes;

    //flag set by M0. Indicates that the machine should shut down after any current moves complete.
    bool _doShutdownAfterMoveCompletes;
    //used in recursive eventLoops in order to do a synchronous movement
    bool _doExitEventLoopAfterMoveCompletes;
    //when we're homing, for example, we need to check endstops before each step, which means no buffering of movements.
    bool _doBufferMoves;
    //Are g-code coordinates to be interpreted as relative or absolute? Default: POS_ABSOLUTE
    PositionMode _positionMode;
    //Are g-code extruder coordinates to be interpreted as relative or absolute? Default: POS_RELATIVE. Set via M82 or M83
    PositionMode _extruderPosMode;
    //Are g-code coordinates to be interpreted as inches or mm? Default: UNIT_MM
    LengthUnit unitMode;
    //absolute (x, y, z, e) destination in millimeters. Doesn't take into account leveling. Purpose of this variable is to allow for relative movements.
    Vector4f _destMm;
    float _destMoveRatePrimitive;
    //the host can set any arbitrary point to be referenced as 0.
    Vector4f _hostZeroOffset;
    //Flag will be true if in the homing routine (in which case we don't want to allow any G1's, etc, to be run)
    bool _isHoming;
    //Need to know if our absolute coordinates are accurate, which changes when power is lost or stepper motors are deactivated.
    bool _isHomed; 
    bool _isWaitingForHotend;
    EventClockT::time_point _lastMotionPlannedTime;
    //M32 allows a gcode file to call subroutines, essentially.
    //  These subroutines can then call more subroutines, so what we have is essentially a call stack.
    //  We only read the top file on the stack, until it's done, and then pop it and return to the next one.
    //BUT, we still need to maintain a com channel to the host (especially for emergency stop, etc).
    //so we store Com channels in a vector & include a flag that tells us whether the root one should act as a special always-active host com
    bool _isRootComPersistent;
    std::vector<gparse::Com> gcodeFileStack;
    SchedType scheduler;
    motion::MotionPlanner<MotionInterface> _motionPlanner;
    Drv driver;
    FileSystem filesystem;
    IODriverTypes ioDrivers;
    public:
        //Initialize the state:
        //@drv Machine instance to take ownership of
        //@fs FileSystem object that describes where to look when asked to open a gcode file
        //@needPersistentCom flag to always keep the first communication channel (e.g. stdio) open
        //  M32 command allows branching to another, local gcode file. By default, this will PAUSE reading/writing from the previous com channel.
        //  But if we want to continue reading from that original com channel while simultaneously reading from the new gcode file, then 'needPersistentCom' should be set to true.
        //  This is normally only relevant for communication with a host, like Octoprint, where we want temperature reading, emergency stop, etc to still work.
        State(Drv &&drv=std::move(Drv()), const FileSystem &fs=FileSystem(), bool needPersistentCom=true);
        //Continually service communication channels & execute received commands until we receive a command to exit
        void eventLoop();
        //return a read-only reference to the interal MotionPlanner object
        //
        //Useful only for introspection (e.g. when running automated tests)
        const motion::MotionPlanner<MotionInterface>& motionPlanner() const {
            return _motionPlanner;
        }
        //if set to false, then running a subprogram will cause the main communication channel to not be serviced until the subprogram returns
        void setPersistentHostCom(bool persistence) {
            _isRootComPersistent = persistence;
        }
        void addComChannel(gparse::Com &&ch) {
            gcodeFileStack.push_back(std::move(ch));
        }
    private:
        void setMoveBuffering(bool doBufferMoves);
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
        Vector4f coordToAbsolute(const Vector4f &coord) const;
        /* Convert an x/y/z/e value sent from the host to MM, in the case that the host is sending inches */
        Vector4f coordToMm(const Vector4f &coord) const;
        /* Convert an x/y/z/e value sent from the host to whatever primitive value we're using internally
         * Acts similarly as a shortcut for posUnitToMM(xUnitToAbsolute(x)), though it may apply transformations in the future.*/
        Vector4f coordToPrimitive(const Vector4f &coord) const;
        float fUnitToPrimitive(float posUnit) const;
        /* Get the last queued position (X, Y, Z, E). Future queued commands may depend on this */
        Vector4f destMm() const;
        /* Control the move rate (AKA "feed rate") */
        float destMoveRatePrimitive() const;
        void setDestMoveRatePrimitive(float f);
        /* The host can set the current physical position to be a reference to an arbitrary point (like 0) */
        void setHostZeroPos(float x, float y, float z, float e);
        /* Reads inputs of any IODrivers, and possible does something with the value (eg feedback loop between thermistor and hotend PWM control */
        bool onIdleCpu(OnIdleCpuIntervalT interval);
        void tendComChannel(gparse::Com &com);
        /* execute the GCode on a Driver object that supports a well-defined interface.
         * returns a Command to send back to the host. */
        template <typename ReplyFunc> void execute(gparse::Command const& cmd, ReplyFunc replyFunc);
        // make an arc from the current position to (x, y, z), maintaining a constant distance from (cX, cY, cZ)
        void queueArc(const Vector4f &dest, const Vector3f &center, bool isCW=false);
        /* Calculate and schedule a movement to absolute-valued x, y, z, e coords from the last queued position */
        void queueMovement(const Vector4f &dest, OptionalArg<float> velXyz=OptionalArg<float>::NotPresent, const motion::MotionFlags flags=motion::MOTIONFLAGS_DEFAULT);
        /* Home to the endstops. */
        void homeEndstops();
        //Check if M109 (set temperature and wait until reached) has been satisfied.
        bool areHeatersReady();
        std::string getEndstopStatusString();
};


template <typename Drv> State<Drv>::State(Drv &&drv, const FileSystem &fs, bool needPersistentCom)
    : _doShutdownAfterMoveCompletes(false),
    _doExitEventLoopAfterMoveCompletes(false), 
    _doBufferMoves(true), 
    _positionMode(POS_ABSOLUTE), _extruderPosMode(POS_ABSOLUTE),  
    unitMode(UNIT_MM), 
    _destMm(0, 0, 0, 0),
    _hostZeroOffset(0, 0, 0, 0),
    _isHoming(false),
    _isHomed(false),
    _isWaitingForHotend(false),
    _lastMotionPlannedTime(std::chrono::seconds(0)), 
    _isRootComPersistent(needPersistentCom),
    scheduler(SchedInterface(*this)),
    _motionPlanner(MotionInterface(*this)),
    driver(std::move(drv)),
    filesystem(fs),
    ioDrivers(std::tuple_cat(_motionPlanner.coordMap().getDependentIoDrivers(), drv.getIoDrivers()))
    {
    this->setDestMoveRatePrimitive(this->driver.defaultMoveRate());
}

template <typename Drv> void State<Drv>::setMoveBuffering(bool doBufferMoves) {
    _doBufferMoves = doBufferMoves;
    if (doBufferMoves) {
        this->scheduler.setDefaultMaxSleep();
    } else {
        this->scheduler.setMaxSleep(std::chrono::milliseconds(1));
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

template <typename Drv> Vector4f State<Drv>::coordToAbsolute(const Vector4f &posUnit) const {
    switch (this->positionMode()) {
        case POS_RELATIVE:
            return posUnit + destMm();
        case POS_ABSOLUTE:
        default:
            return posUnit; //no transformation needed.
    }
}

template <typename Drv> Vector4f State<Drv>::coordToMm(const Vector4f &coord) const {
    //If we're set to interpret coordinates as inches, then convert to mm:
    switch (this->unitMode) {
        case UNIT_IN:
            return coord * mathutil::MM_PER_IN;
        case UNIT_MM:
        default: //impossible case.
            return coord;
    }
}

template <typename Drv> Vector4f State<Drv>::coordToPrimitive(const Vector4f &coord) const {
    return coordToMm(coordToAbsolute(coord)) + _hostZeroOffset;
}

template <typename Drv> float State<Drv>::fUnitToPrimitive(float posUnit) const {
    return coordToMm(Vector4f(posUnit/60, 0, 0, 0)).x(); //feed rate is given in mm/minute, or in/minute.
}

template <typename Drv> Vector4f State<Drv>::destMm() const {
    return this->_destMm;
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
    _hostZeroOffset = _destMm - Vector4f(x, y, z, e);
    //What x value makes _hostZeroX (new) == _hostZeroX (old) ?
    //_destXPrimitive - x = _hostZeroX
    //x = _destXPrimitive - _hostZeroX;
}

template <typename Drv> bool State<Drv>::onIdleCpu(OnIdleCpuIntervalT interval) {
    bool motionNeedsCpu = false;
    if (scheduler.isRoomInBuffer()) { 
        auto ioDriverIterEvtPair = ioDrivers.peekNextEvent();
        auto ioDriverEvtIter = ioDriverIterEvtPair.first;
        OutputEvent ioDriverEvt = ioDriverIterEvtPair.second;
        OutputEvent motionEvt = _motionPlanner.peekNextEvent();

        //LOG("Next IoDriverEvt at %lu, state: %i\n", ioDriverEvt.time().time_since_epoch().count(), ioDriverEvt.state());
        //bool doServiceMotion =   !motionEvt.isNull()   && (ioDriverEvt.isNull() || motionEvt.time() <= ioDriverEvt.time());
        bool doServiceIoDriver = !ioDriverEvt.isNull() && (motionEvt.isNull()   || ioDriverEvt.time() <= motionEvt.time());
        //LOG("doServiceIoDriver: %i. ioDriverEvt.time: %lu, motionEvt.time: %lu\n", doServiceIoDriver, 
        //    ioDriverEvt.time().time_since_epoch().count(), motionEvt.time().time_since_epoch().count());
        if (doServiceIoDriver) {
            //IoDriver event occurs first, so queue it & consume it.
            this->scheduler.queue(ioDriverEvt);
            ioDriverEvtIter.consumeNextEvent();
        } else if (_doBufferMoves || _lastMotionPlannedTime <= EventClockT::now()) { 
            //if we're homing (_doBufferMoves==false), we don't want to queue the next step until the current one has actually completed.
            //Although the IoDriver event does not occur first, that doesn't mean there is necessarily a motion event.
            if (!motionEvt.isNull()) {
                _motionPlanner.consumeNextEvent();
                this->scheduler.queue(motionEvt);
                _lastMotionPlannedTime = motionEvt.time();
                motionNeedsCpu = scheduler.isRoomInBuffer();
            }
        }
        if (_motionPlanner.peekNextEvent().isNull()) {
            //LOG("State::onIdleCpu() motionEvt is null; signals end of move\n");
            //check if we have received a command to exit after the current move is complete
            //if that command has been received, and the current move has been completed, then exit the event loop.
            if ((_doShutdownAfterMoveCompletes || _doExitEventLoopAfterMoveCompletes) && !motionNeedsCpu) {
                //reset the event loop exit flag (but not the shutdown flag!)
                _doExitEventLoopAfterMoveCompletes = false;
                scheduler.exitEventLoop();
                //It would be best to return now rather than tend the com channel
                //As we don't want to risk the homing routine being interrupted.
                return false;
            }
        }
    }

    //Only check the communications periodically because calling execute(com.getCommand()) DOES add up.
    if (interval == OnIdleCpuIntervalWide) {
        if (!gcodeFileStack.empty()) {
            if (_isRootComPersistent) {
                tendComChannel(gcodeFileStack.front());
            }
            //LOGV("Tending gcodeFileStack top\n");
            //now tend the top channel, although it's possible that it's been popped and there are no more com channels
            if (!gcodeFileStack.empty()) {
                //it's OK if we tend the same com channel twice.
                tendComChannel(gcodeFileStack.back());
                //Remove all gcode files that have been fully read
                while (!gcodeFileStack.empty() && gcodeFileStack.back().isAtEof()) {
                    gcodeFileStack.pop_back();
                }
            }
        }
    }

    bool driversNeedCpu = this->ioDrivers.onIdleCpu(interval);
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
        
        execute(cmd, [&](const gparse::Response &resp) {
            if (!NO_LOG_M105 || !cmd.isM105()) {
                LOG("command: %s\n", cmd.toGCode().c_str());
                LOG("response: %s\n", resp.toString().c_str());
            }
            com.reply(resp);
        });
        //if the above callback isn't called (because the command isn't ready to be serviced), 
        // then a future call to com.getCommand() will return the same command we just read (as opposed to the next line)
    }
}

template <typename Drv> template <typename ReplyFunc> void State<Drv>::execute(gparse::Command const &cmd, ReplyFunc reply) {
    //process a gcode command received on the given communications channel and return an appropriate response
    if (cmd.isG0() || cmd.isG1()) { //rapid movement / controlled (linear) movement (currently uses same code)
        if (!_motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
            return;
        }
        if (!areHeatersReady()) { //make sure that a call to M109 doesn't allow movements until it's complete.
            return;
        }
        if (_isHoming) {
            return;
        }
        if (!_isHomed && _motionPlanner.doHomeBeforeFirstMovement()) {
            this->homeEndstops();
        }
        
        float curX, curY, curZ, curE;
        std::tie(curX, curY, curZ, curE) = destMm().tuple();
        Vector4f cmdDest(cmd.getX(), cmd.getY(), cmd.getZ(), cmd.getE());

        cmdDest = coordToPrimitive(cmdDest);
        Vector4f trueDest = Vector4f(cmd.hasX() ? cmdDest.x() : curX, cmd.hasY() ? cmdDest.y() : curY, cmd.hasZ() ? cmdDest.z() : curZ, cmd.hasE() ? cmdDest.e() : curE);
        if (cmd.hasF()) {
            this->setDestMoveRatePrimitive(fUnitToPrimitive(cmd.getF()));
        }
        this->queueMovement(trueDest);
        reply(gparse::Response::Ok);
    } else if (cmd.isG2() || cmd.isG3()) {
        if (!_motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
            return;
        }
        if (!areHeatersReady()) { //make sure that a call to M109 doesn't allow movements until it's complete.
            return;
        }
        if (_isHoming) {
            return;
        }
        if (!_isHomed && _motionPlanner.doHomeBeforeFirstMovement()) {
            this->homeEndstops();
        }
        LOGW("Warning: G2/G3 is experimental\n");
        //first, get the end coordinate and optional feed-rate:
        float curX, curY, curZ, curE;
        std::tie(curX, curY, curZ, curE) = destMm().tuple();
        Vector4f cmdDest(cmd.getX(), cmd.getY(), cmd.getZ(), cmd.getE());

        cmdDest = coordToPrimitive(cmdDest);
        Vector4f trueDest = Vector4f(cmd.hasX() ? cmdDest.x() : curX, cmd.hasY() ? cmdDest.y() : curY, cmd.hasZ() ? cmdDest.z() : curZ, cmd.hasE() ? cmdDest.e() : curE);
        if (cmd.hasF()) {
            this->setDestMoveRatePrimitive(fUnitToPrimitive(cmd.getF()));
        }
        //Now get the center-point coordinate:
        float i = cmd.getI();
        float j = cmd.getJ();
        float k = cmd.getK();
        Vector3f center = coordToPrimitive(Vector4f(i, j, k, 0)).xyz();

        // if the center z is not explicitly set, make it the midpoint between current z and end z
        center = center.withZ(cmd.hasK() ? center.z() : 0.5f*(curZ+trueDest.z()));

        this->queueArc(trueDest, center, cmd.isG2());
        reply(gparse::Response::Ok);
    } else if (cmd.isG20()) { //g-code coordinates will now be interpreted as inches
        setUnitMode(UNIT_IN);
        reply(gparse::Response::Ok);
    } else if (cmd.isG21()) { //g-code coordinates will now be interpreted as millimeters.
        setUnitMode(UNIT_MM);
        reply(gparse::Response::Ok);
    } else if (cmd.isG28()) { //home to end-stops / zero coordinates
        if (!_motionPlanner.readyForNextMove()) { //don't queue another command unless we have the memory for it.
            return;
        }
        if (!areHeatersReady()) { //make sure that a call to M109 doesn't allow movements until it's complete.
            return;
        }
        if (_isHoming) {
            return;
        }
        //reply before homing, because homing may hang.
        reply(gparse::Response::Ok);
        this->homeEndstops();
    } else if (cmd.isG90()) { //set g-code coordinates to absolute
        setPositionMode(POS_ABSOLUTE);
        setExtruderPosMode(POS_ABSOLUTE);
        reply(gparse::Response::Ok);
    } else if (cmd.isG91()) { //set g-code coordinates to relative
        setPositionMode(POS_RELATIVE);
        setExtruderPosMode(POS_RELATIVE);
        reply(gparse::Response::Ok);
    } else if (cmd.isG92()) { //set current position = 0
        float actualX, actualY, actualZ, actualE;
        if (!cmd.hasAnyXYZEParam()) {
            //make current position (0, 0, 0, 0)
            actualX = actualY = actualZ = actualE = 0; 
        } else {
            Vector4f cmdPosMm = coordToMm(Vector4f(cmd.getX(), cmd.getY(), cmd.getZ(), cmd.getE()));
            Vector4f curZeroPos = destMm() - _hostZeroOffset;
            actualX = cmd.hasX() ? cmdPosMm.x() : curZeroPos.x();
            actualY = cmd.hasY() ? cmdPosMm.y() : curZeroPos.y();
            actualZ = cmd.hasZ() ? cmdPosMm.z() : curZeroPos.z();
            actualE = cmd.hasE() ? cmdPosMm.e() : curZeroPos.e();
        }
        setHostZeroPos(actualX, actualY, actualZ, actualE);
        reply(gparse::Response::Ok);
    } else if (cmd.isM0()) { //Stop; empty move buffer & exit cleanly
        LOGD("recieved M0 command: finishing moves, then exiting\n");
        _doShutdownAfterMoveCompletes = true;
        reply(gparse::Response::Ok);
    } else if (cmd.isM17()) { //enable all stepper motors
        ioDrivers.lockAllAxes();
        //iodrv::IODriver::lockAllAxes(this->ioDrivers.tuple());
        reply(gparse::Response::Ok);
    } else if (cmd.isM18()) { //allow stepper motors to move 'freely'
        ioDrivers.unlockAllAxes();
        //iodrv::IODriver::unlockAllAxes(this->ioDrivers.tuple());
        reply(gparse::Response::Ok);
    } else if (cmd.isM21()) { //initialize SD card (nothing to do).
        reply(gparse::Response::Ok);
    } else if (cmd.isM22()) {
        //"release SD card" (nothing to do).
        reply(gparse::Response::Ok);
    } else if (cmd.isM32()) { //select file on SD card and print:
        LOGD("loading gcode: %s\n", cmd.getSpecialStringParam().c_str());
        reply(gparse::Response::Ok);
        //create another Communications channel for reading from the gcode file.
        gcodeFileStack.push_back(gparse::Com(filesystem.relGcodePathToAbs(cmd.getSpecialStringParam()), nullptr, true));
    } else if (cmd.isM82()) { //set extruder absolute mode
        setExtruderPosMode(POS_ABSOLUTE);
        reply(gparse::Response::Ok);
    } else if (cmd.isM83()) { //set extruder relative mode
        setExtruderPosMode(POS_RELATIVE);
        reply(gparse::Response::Ok);
    } else if (cmd.isM84()) { //stop idle hold: relax all motors (same as M18)
        ioDrivers.unlockAllAxes();
        //iodrv::IODriver::unlockAllAxes(this->ioDrivers.tuple());
        reply(gparse::Response::Ok);
    } else if (cmd.isM99()) { //return from macro/subprogram
        //note: can't simply pop the top file, because then that causes memory access errors when trying to send it a reply.
        //Need to check if com channel that received this command is the top one. If yes, then pop it and return Response::Null so that no response will be sent.
        //  else, pop it and return Response::Ok.
        if (gcodeFileStack.empty()) { //return from the main I/O routine = kill program
            LOGW("M99 received, but not in a macro/subprogam; exiting\n");
            _doShutdownAfterMoveCompletes = true;
            reply(gparse::Response::Ok);
        } else {
            reply(gparse::Response::Ok);
            gcodeFileStack.pop_back();
        }
    } else if (cmd.isM104()) { //set hotend temperature and return immediately.
        if (cmd.hasS()) {
            ioDrivers.setHotendTemp(cmd.getS());
        } else {
            reply(gparse::Response(gparse::ResponseWarning, "No temperature given"));
        }
        reply(gparse::Response::Ok);
    } else if (cmd.isM105()) { //get temperature, in C
        CelciusType t = mathutil::ABSOLUTE_ZERO_CELCIUS;
        CelciusType b = mathutil::ABSOLUTE_ZERO_CELCIUS;
        if (ioDrivers.hotends().length()) {
            t = ioDrivers.hotends()[0].getMeasuredTemperature();
        }
        if (ioDrivers.heatedBeds().length()) {
            b = ioDrivers.heatedBeds()[0].getMeasuredTemperature();
        }
        reply(gparse::Response(gparse::ResponseOk, {
            std::make_pair("T", std::to_string(t)),
            std::make_pair("B", std::to_string(b))
        }));
    } else if (cmd.isM106() || cmd.isM107()) { //set fan speed. Takes parameter S. Can be 0-255 (PWM) or in some implementations, 0.0-1.0
        if (cmd.isM107()) {
            LOGW("M107 is deprecated. Use M106 with S=0 instead.\n");
        }

        float s = cmd.isM107() ? 0.0f : cmd.getNormalizedS(1.0f); //PWM duty cycle

        if (cmd.hasP()) {
            // set only the speed of the fan at the specified index, 'P'
            int index = cmd.getP();
            if (index >= 0 && (unsigned)index < ioDrivers.fans().length()) {
                this->ioDrivers.fans()[index].setFanDutyCycle(s);
            }  else {
                reply(gparse::Response(gparse::ResponseWarning, "Invalid fan index"));
            }
        } else {
            // set the speed of ALL fans to 's'
            this->ioDrivers.setFanDutyCycle(s);
        }
        reply(gparse::Response::Ok);
    } else if (cmd.isM109()) { //set extruder temperature to S param and wait.
        LOGW("(state.h): OP_M109 (set extruder temperature and wait) not fully implemented\n");
        if (cmd.hasS()) {
            ioDrivers.setHotendTemp(cmd.getS());
        } else {
            reply(gparse::Response(gparse::ResponseWarning, "No temperature given"));
        }
        _isWaitingForHotend = true;
        reply(gparse::Response::Ok);
    } else if (cmd.isM110()) { //set current line number
        LOGW("(state.h): OP_M110 (set current line number) not implemented\n");
        reply(gparse::Response::Ok);
    } else if (cmd.isM111()) {
        // set debug info.
        // the S parameter is a bitfield indicating the log levels to enable. bit 0 = verbose, bit 1 = debug, bit 2 = info+errors
        // e.g. S3 = 0b011 = enable verbose + debug logging, but not info/errors.
        int bitfield = cmd.getS(0);
        logging::enableVerbose(bitfield & 1);
        logging::enableDebug(bitfield & 2);
        logging::enableInfo(bitfield & 4);
        reply(gparse::Response::Ok);
    } else if (cmd.isM112()) { //emergency stop
        reply(gparse::Response::Ok);
        exit(1);
    } else if (cmd.isM115()) {
        // get firmware info
        // if you derive from Printipi and reimplement this method to show the name of your firmware,
        //   please consider adding "(based on printipi)"
        // this will aid hosts that tailore their communications based on the detected firmware
        // and will also help track the evolution of the software.
        reply(gparse::Response(gparse::ResponseOk, {
            std::make_pair("FIRMWARE_NAME", "printipi"),
            std::make_pair("FIRMWARE_URL", "https%3A//github.com/Wallacoloo/printipi")
        }));
    } else if (cmd.isM116()) { //Wait for all heaters (and slow moving variables) to reach target
        _isWaitingForHotend = true;
        reply(gparse::Response::Ok);
    } else if (cmd.isM117()) { //print message
        LOG("M117 message: '%s'\n", cmd.getSpecialStringParam().c_str());
        reply(gparse::Response::Ok);
    } else if (cmd.isM119()) {
        //get endstop status
        reply(gparse::Response(gparse::ResponseOk, getEndstopStatusString()));
    } else if (cmd.isM140()) { //set BED temp and return immediately.
        LOGW("(gparse/state.h): OP_M140 (set bed temp) is untested\n");
        if (cmd.hasS()) {
            ioDrivers.setBedTemp(cmd.getS());
        } else {
            reply(gparse::Response(gparse::ResponseWarning, "No temperature given"));
        }
        reply(gparse::Response::Ok);
    } else if(cmd.isM280()) {
        //set servo angle. P=servo index, S=angle (in degrees, presumably)
        int index = cmd.getP(-1);
        float angleDeg = cmd.getS(0);
        if (index >= 0 && (unsigned)index < ioDrivers.servos().length()) {
            ioDrivers.servos()[index].setServoAngleDegrees(angleDeg);
        } else {
            reply(gparse::Response(gparse::ResponseWarning, "Invalid servo index"));
        }
        reply(gparse::Response::Ok);
    } else if (cmd.isM999()) {
        //Restart after being stopped by error.
        //I think this gets sent whenever Octoprint temporarily loses communication with the program
        //  (e.g. you kill the firmware with ctrl+c, then restart it).
        reply(gparse::Response::Ok);
    } else if (cmd.isTxxx()) { //set tool number
        LOGW("(gparse/state.h): OP_T[n] (set tool number) not implemented\n");
        reply(gparse::Response::Ok);
    } else {
        throw std::runtime_error(std::string("unrecognized gcode opcode: '") + cmd.getOpcode() + "'");
    }
}

template <typename Drv> void State<Drv>::queueArc(const Vector4f &dest, const Vector3f &center, bool isCW) {
    //track the desired position to minimize drift over time caused by relative movements when we can't precisely reach the given coordinates:
    _destMm = dest;
    //now determine the velocity of the move. We just calculate limits & relay the info to the motionPlanner
    float velXyz = destMoveRatePrimitive();
    float minExtRate = -this->driver.maxRetractRate();
    float maxExtRate = this->driver.maxExtrudeRate();
    //start the next move at the time that the previous move is scheduled to complete, unless that time is in the past
    auto startTime = std::max(_lastMotionPlannedTime, EventClockT::now());
    _motionPlanner.arcTo(startTime, dest, center, velXyz, minExtRate, maxExtRate, isCW);
}
        
template <typename Drv> void State<Drv>::queueMovement(const Vector4f &dest, OptionalArg<float> velXyz, const motion::MotionFlags flags) {
    //track the desired position to minimize drift over time caused by relative movements when we can't precisely reach the given coordinates:
    _destMm = dest;
    //now determine the velocity limits & relay the info to the motionPlanner
    float minExtRate = -this->driver.maxRetractRate();
    float maxExtRate = this->driver.maxExtrudeRate();
    //start the next move at the time that the previous move is scheduled to complete, unless that time is in the past
    auto startTime = std::max(_lastMotionPlannedTime, EventClockT::now());
    _motionPlanner.moveTo(startTime, dest, velXyz.get(destMoveRatePrimitive()), minExtRate, maxExtRate, flags);
}

template <typename Drv> void State<Drv>::homeEndstops() {
    //we need to keep track of the fact we're homing, so as to ignore remote movement commands until homing is complete
    this->_isHoming = true;
    bool restoreMoveBuffering = _doBufferMoves;
    setMoveBuffering(false);

    CoordMapInterface interface(*this);
    _motionPlanner.coordMap().executeHomeRoutine(interface);

    setMoveBuffering(restoreMoveBuffering);
    this->_isHomed = true;
    this->_isHoming = false;
}

template <typename Drv> bool State<Drv>::areHeatersReady() {
    if (_isWaitingForHotend) {
        // check if ALL heaters have reached their targets
        bool isReady = ioDrivers.heaters().all([](typename IODriverTypes::iteratorbase &d) {
            return d.getMeasuredTemperature() > d.getTargetTemperature();
        });
        _isWaitingForHotend = !isReady;
    }
    return !_isWaitingForHotend;
}

template <typename Drv> std::string State<Drv>::getEndstopStatusString() {
    //give a useful message if the machine doesn't have any endstops
    if (ioDrivers.endstops().empty()) {
        return "(no endstops)";
    }
    //return the endstop status strings joined by spaces, eg "triggered open triggered"
    std::ostringstream imploded;
    bool first = true;
    for (auto& driver : ioDrivers.endstops()) {
        if (first) {
            first = false;
        } else {
            imploded << ' ';
        }
        imploded << (driver.isEndstopTriggered() ? "triggered" : "open");
    }
    return imploded.str();
}

#endif
