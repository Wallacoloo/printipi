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
 * Printipi/motion/motionplanner.h
 *
 * MotionPlanner takes commands from the State (mainly those caused by G1 and G28) and resolves the move into a path via interfacing with a CoordMap, AxisSteppers, and an AccelerationProfile.
 * Once a path is planned, State can call MotionPlanner.nextStep() and be given data in the form of an Event, which can be passed on to a Scheduler.
 * 
 * Interface must have 2 public typedefs: CoordMapT and AxisStepperTypes. These are often provided by the machine driver.
 */
#ifndef MOTION_MOTIONPLANNER_H
#define MOTION_MOTIONPLANNER_H

#include <array>
#include "accelerationprofile.h"
#include "drivers/axisstepper.h"
#include "event.h"

enum MotionType {
    MotionNone,
    MotionLinear,
    MotionHome
};

template <typename Interface, typename AccelProfile=NoAcceleration> class MotionPlanner {
    private:
        typedef typename Interface::CoordMapT CoordMapT;
        typedef typename Interface::AxisStepperTypes AxisStepperTypes;
        typedef typename drv::AxisStepper::GetHomeStepperTypes<AxisStepperTypes>::HomeStepperTypes HomeStepperTypes;
        CoordMapT _coordMapper; //object that maps from (x, y, z) to mechanical coords (eg A, B, C for a kossel)
        AccelProfile _accel; //transforms the constant-velocity motion stream into one that considers acceleration
        std::array<int, CoordMapT::numAxis()> _destMechanicalPos; //the mechanical position of the last step that was scheduled
        AxisStepperTypes _iters; //Each axis iterator reports the next time it needs to be stepped. _iters is for linear movement
        HomeStepperTypes _homeIters; //Axis iterators used when homing
        EventClockT::duration _baseTime; //The time at which the current path segment began (this will be a fraction of a second before the time which the first step in this path is scheduled for)
        float _duration; //the estimated duration of the current piece, not taking into account acceleration
        MotionType _motionType; //which type of segment is being planned
    public:
        MotionPlanner() : 
            _accel(), 
            _destMechanicalPos(), 
            _iters(), _homeIters(), 
            _baseTime(), 
            _duration(NAN),
            //_maxVel(0), 
            _motionType(MotionNone) {}
        bool readyForNextMove() const {
            //returns true if a call to moveTo() or homeEndstops() wouldn't hang, false if it would hang (or cause other problems)
            //Note: for now, there isn't actually buffering.
            return _motionType == MotionNone;
        }
    private:
        Event _nextStep(drv::AxisStepper &s, bool isHoming) {
            LOGV("MotionPlanner::nextStep() is: %i at %g of %g\n", s.index(), s.time, _duration);
            if (s.time > _duration || s.time <= 0 || std::isnan(s.time)) { //if the next time the given axis wants to step is invalid or past the movement length, then end the motion
                //Note: don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
                //Note: This conditional causes the MotionPlanner to always undershoot the desired position, when it may be desireable to overshoot some of them - see https://github.com/Wallacoloo/printipi/issues/15
                if (isHoming) { 
                    //if homing, then we now know the axis mechanical positions; fetch them
                    _destMechanicalPos = CoordMapT::getHomePosition(_destMechanicalPos);
                }
                //log debug info:
                float x, y, z, e;
                std::tie(x, y, z, e) = CoordMapT::xyzeFromMechanical(_destMechanicalPos);
                LOGD("MotionPlanner::moveTo Got (x,y,z,e) %f, %f, %f, %f\n", x, y, z, e);
                LOGD("MotionPlanner _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
                _motionType = MotionNone; //motion is over.
                return Event();
            }
            float transformedTime = _accel.transform(s.time); //transform the step time according to acceleration profile
            LOGV("Step transformed time: %f\n", transformedTime);
            Event e = s.getEvent(transformedTime);
            e.offset(_baseTime); //AxisSteppers report times relative to the start of motion; transform to absolute.
            _destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction); //update the mechanical position tracked in software
            if (isHoming) {
                s.nextStep(_homeIters); //advance the respective AxisStepper to its next step.
            } else {
                s.nextStep(_iters);
            }
            return e;
        }
        //black magic to get nextStep to work when either AxisStepperTypes or HomeStepperTypes have length 0:
        //If they are length zero, then _nextStep* just returns an empty event and a compilation error is avoided.
        //Otherwise, the templated function is called, and _nextStep is run as usual:
        template <bool T> Event _nextStepHoming(std::integral_constant<bool, T> ) {
            return _nextStep(drv::AxisStepper::getNextTime(_homeIters), true);
        }
        Event _nextStepHoming(std::false_type ) {
            return Event();
        }
        template <bool T> Event _nextStepMoving(std::integral_constant<bool, T> ) {
            return _nextStep(drv::AxisStepper::getNextTime(_iters), false);
        }
        Event _nextStepMoving(std::false_type ) {
            return Event();
        }
    public:
        bool isHoming() const {
            return _motionType == MotionHome;
        }
        Event nextStep() {
            //called by State to query the next step in the current path segment
            if (_motionType == MotionNone) {
                return Event(); //no next step; return a null Event
            }
            if ((isHoming() && std::tuple_size<HomeStepperTypes>::value == 0) || (!isHoming() && std::tuple_size<AxisStepperTypes>::value == 0)) {
                return Event(); //sanity checks. Should get optimized away on most machines.
            }
            if (isHoming()) {
                return _nextStepHoming(std::integral_constant<bool, std::tuple_size<HomeStepperTypes>::value != 0>());
            } else {
                return _nextStepMoving(std::integral_constant<bool, std::tuple_size<AxisStepperTypes>::value != 0>());
            }
        }
        void moveTo(EventClockT::time_point baseTime, float x, float y, float z, float e, float maxVelXyz, float minVelE, float maxVelE) {
            //called by State to queue a movement from the current destination to a new one, with the desired motion beginning at baseTime
            //Note: it is illegal to call this if readyForNextMove() != true
            if (std::tuple_size<AxisStepperTypes>::value == 0) {
                return; //Sanity check. Algorithms only work for machines with atleast 1 axis.
            }
            this->_baseTime = baseTime.time_since_epoch();
            float curX, curY, curZ, curE;
            std::tie(curX, curY, curZ, curE) = CoordMapT::xyzeFromMechanical(_destMechanicalPos);
            std::tie(x, y, z) = CoordMapT::applyLeveling(std::make_tuple(x, y, z)); //get the REAL destination.
            std::tie(x, y, z, e) = CoordMapT::bound(std::make_tuple(x, y, z, e)); //Fix impossible coordinates
            
            //Calculate velocities in x, y, z, e directions, and the duration of the linear movement:
            float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
            float dist = sqrt(distSq);
            float minDuration = dist/maxVelXyz; //duration, should there be no acceleration
            float velE = (e-curE)/minDuration;
            //float newVelE = this->driver.clampExtrusionRate(velE);
            float newVelE = std::max(minVelE, std::min(maxVelE, velE));
            if (velE != newVelE) { //in the case that newXYZ = currentXYZ, but extrusion is different, regulate that.
                velE = newVelE;
                minDuration = (e-curE)/newVelE; //L/(L/t) = t
                maxVelXyz = dist/minDuration;
            }
            
            float vx = (x-curX)/minDuration;
            float vy = (y-curY)/minDuration;
            float vz = (z-curZ)/minDuration;
            LOGD("MotionPlanner::moveTo (%f, %f, %f, %f) -> (%f, %f, %f, %f)\n", curX, curY, curZ, curE, x, y, z, e);
            LOGD("MotionPlanner::moveTo _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
            //LOGD("MotionPlanner::moveTo V:%f, vx:%f, vy:%f, vz:%f, ve:%f dur:%f\n", maxVelXyz, vx, vy, vz, velE, minDuration);
            drv::AxisStepper::initAxisSteppers(_iters, _destMechanicalPos, vx, vy, vz, velE);
            this->_duration = minDuration;
            this->_motionType = MotionLinear;
            this->_accel.begin(minDuration, maxVelXyz);
        }

        void homeEndstops(EventClockT::time_point baseTime, float maxVelXyz) {
            //Called by State to begin a motion that homes to the endstops (and stays there)
            //Note: it is illegal to call this if readyForNextMove() != true
            if (std::tuple_size<HomeStepperTypes>::value == 0) {
                return; //Sanity check. Algorithms only work for machines with atleast 1 axis.
            }
            drv::AxisStepper::initAxisHomeSteppers(_homeIters, maxVelXyz);
            this->_baseTime = baseTime.time_since_epoch();
            this->_duration = NAN;
            this->_motionType = MotionHome;
            this->_accel.begin(NAN, maxVelXyz);
        }
};


#endif
