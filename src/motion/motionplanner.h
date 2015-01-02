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
 * Interface must have 2 public typedefs: CoordMapT and AccelerationProfileT. These are often provided by the machine driver.
 */
#ifndef MOTION_MOTIONPLANNER_H
#define MOTION_MOTIONPLANNER_H

#include <array>
#include <cassert>
#include <stdexcept> //for runtime_error
#include <utility> //for std::declval
#include "accelerationprofile.h"
#include "axisstepper.h"
#include "common/vector3.h"
#include "common/vector4.h"
#include "common/logging.h"

namespace motion {

//There are 3 distinct types of motion that can occur at any given time:
enum MotionType {
    MotionNone,
    MotionLinear,
    MotionArc,
};

//bitwise-or'd flags providing more context for movements
enum MotionFlags {
    MOTIONFLAGS_DEFAULT=0,
    NO_LEVELING=1,
    NO_BOUNDING=2,
    USE_ENDSTOPS=4,
};
//bitwise OR operator for MotionFlags, to avoid NO_LEVELING | NO_BOUNDING resulting in an integer type instead of a MotionFlags type.
inline MotionFlags operator|(MotionFlags a, MotionFlags b) {
    return static_cast<MotionFlags>(static_cast<int>(a) | static_cast<int>(b));
}

template<typename ArrayT> struct array_size;
template<typename ElementT, std::size_t Size> struct array_size<std::array<ElementT, Size> > {
    static const std::size_t size = Size;
};

template <typename AxisStepperTypes, std::size_t IdxPlusOne> class MaxOutputEventSequenceSize {
    typedef decltype(std::get<IdxPlusOne-1>(std::declval<AxisStepperTypes>())) ThisAxisStepper;
    typedef decltype(std::declval<ThisAxisStepper>().getStepOutputEventSequence(EventClockT::time_point())) OutputEventArrayType; //std::array<OutputEvent, N>
    static constexpr std::size_t mySize() {
        return array_size<OutputEventArrayType>::size;
    }
    static constexpr std::size_t prevMaxSize() {
        return MaxOutputEventSequenceSize<AxisStepperTypes, IdxPlusOne-1>::maxSize();
    }
    public:
        static constexpr std::size_t maxSize() {
            return prevMaxSize() > mySize() ? prevMaxSize() : mySize();
            //return std::max(prevMaxSize(), mySize()); //there is no constexpr std::max in c++11 (only c++14)
        }
};

template <typename AxisStepperTypes> struct MaxOutputEventSequenceSize<AxisStepperTypes, 0> {
    static constexpr std::size_t maxSize() {
        return 0;
    }
};

template <typename Interface> class MotionPlanner {
    private:
        struct UpdateOutputEvents {
            template <std::size_t MyIdx, typename T> void operator()(std::integral_constant<std::size_t, MyIdx> myIdx, T &stepper, 
              MotionPlanner<Interface> *_this, EventClockT::time_point baseTime) {
                (void)myIdx; //unused
                auto sequence = stepper.getStepOutputEventSequence(baseTime);
                std::copy(sequence.begin(), sequence.end(), _this->outputEventBuffer.begin());
                _this->curOutputEvent = _this->outputEventBuffer.begin();
                _this->endOutputEvent = _this->outputEventBuffer.begin() + sequence.size();
            }
        };
        typedef typename Interface::CoordMapT CoordMapT;
        typedef typename Interface::AccelerationProfileT AccelerationProfileT;
        typedef decltype(std::declval<CoordMapT>().getAxisSteppers()) AxisStepperTypes;
        typedef decltype(std::declval<CoordMapT>().getArcSteppers())  ArcStepperTypes;
        typedef std::array<OutputEvent, MaxOutputEventSequenceSize<AxisStepperTypes, std::tuple_size<AxisStepperTypes>::value>::maxSize()> OutputEventBufferT;

        //Interface _interface;
        //object that maps from (x, y, z) to mechanical coords (eg A, B, C for a kossel)
        CoordMapT _coordMapper;
        //transforms the constant-velocity motion stream into one that considers acceleration
        AccelerationProfileT _accel;
        //the mechanical position of the last step that was scheduled
        std::array<int, CoordMapT::numAxis()> _destMechanicalPos;
        //Each axis iterator reports the next time it needs to be stepped. _iters is for linear movement
        AxisStepperTypes _iters; 
        //Axis iterators used when moving in an arc
        ArcStepperTypes _arcIters;
        //The time at which the current path segment began (this will be a fraction of a second before the time which the first step in this path is scheduled for)
        EventClockT::time_point _baseTime;
        //the estimated duration of the current piece, not taking into account acceleration
        float _duration; 
        //which type of segment is being planned
        MotionType _motionType; 
        //whether or not to check endstops before each step (typically only useful in homing/autocalibration)
        bool _useEndstops;
        
        //hold the maximum-sized OutputEvent sequence from any AxisStepper.
        OutputEventBufferT outputEventBuffer;
        //iterators used to allow requesting idividual sequential OutputEvents
        typename OutputEventBufferT::iterator curOutputEvent;
        typename OutputEventBufferT::iterator endOutputEvent;
    public:
        MotionPlanner(const Interface &interface) : 
            //_interface(interface),
            _coordMapper(interface.getCoordMap()),
            _accel(interface.getAccelerationProfile()), 
            _destMechanicalPos(), 
            _iters(_coordMapper.getAxisSteppers()), _arcIters(_coordMapper.getArcSteppers()),
            _baseTime(), 
            _duration(NAN),
            _motionType(MotionNone),
            _useEndstops(false),
            outputEventBuffer(),
            curOutputEvent(outputEventBuffer.begin()),
            endOutputEvent(outputEventBuffer.begin()) {}
        const CoordMapT& coordMap() const {
            return _coordMapper;
        }
        CoordMapT& coordMap() {
            return _coordMapper;
        }
        //readForNextMove returns true if a call to moveTo() or homeEndstops() wouldn't hang, false if it would hang (or cause other problems)
        bool readyForNextMove() const {
            return _motionType == MotionNone;
        }
        bool doHomeBeforeFirstMovement() const {
            return _coordMapper.doHomeBeforeFirstMovement();
        }
        //return the actual cartesian position of the effector/hotend
        //
        //Note that if there is a move in progress, this position is based off the last step event 
        // that's been retrieved from <nextOutputEvent>
        Vector4f actualCartesianPosition() const {
            return _coordMapper.xyzeFromMechanical(_destMechanicalPos);
        }
        const std::array<int, CoordMapT::numAxis()> & axisPositions() const {
            return _destMechanicalPos;
        }
        void resetAxisPositions(const std::array<int, CoordMapT::numAxis()> &pos) {
            _destMechanicalPos = pos;
        }
    private:
        template <typename StepperTypes> void _nextStep(StepperTypes &steppers, AxisStepper &s) {
            LOGV("MotionPlanner::nextStep() is: %i at %g of %g\n", s.index(), s.time, _duration);
            if (s.time > _duration || s.time <= 0 || std::isnan(s.time)) { //if the next time the given axis wants to step is invalid or past the movement length, then end the motion
                //Note: don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
                //Note: This conditional causes the MotionPlanner to always undershoot the desired position, when it may be desireable to overshoot some of them - see https://github.com/Wallacoloo/printipi/issues/15
                //log debug info:
                Vector4f pos = _coordMapper.xyzeFromMechanical(_destMechanicalPos);
                LOGD("MotionPlanner::moveTo Got: %s\n", pos.str().c_str());
                LOGD("MotionPlanner _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
                _motionType = MotionNone; //motion is over.
                return;
            }
            float transformedTime = _accel.transform(s.time); //transform the step time according to acceleration profile
            EventClockT::duration transformedChronoTime = std::chrono::duration_cast<EventClockT::duration>(std::chrono::duration<float>(transformedTime));
            LOGV("Step transformed time: %f\n", transformedTime);
            //update outputEventBuffer member variable:
            tupleCallOnIndex(steppers, UpdateOutputEvents(), s.index(), this, _baseTime + transformedChronoTime);            
            //Event e = s.getEvent(transformedTime);
            //e.offset(_baseTime); //AxisSteppers report times relative to the start of motion; transform to absolute.
            _destMechanicalPos[s.index()] += stepDirToSigned<int>(s.direction); //update the mechanical position tracked in software
            //advance the respective AxisStepper to its next step:
            s.nextStep(steppers, _useEndstops);
            LOGV("MotionPlanner::nextStep() generated %zu OutputEvents\n", (endOutputEvent-curOutputEvent));
        }
        //black magic to get nextStep to work when either AxisStepperTypes or HomeStepperTypes have length 0:
        //If they are length zero, then _nextStep* just returns an empty event and a compilation error is avoided.
        //Otherwise, the templated function is called, and _nextStep is run as usual:
        template <bool T> void _nextStepLinear(std::integral_constant<bool, T> ) {
            _nextStep(_iters, AxisStepper::getNextTime(_iters));
        }
        void _nextStepLinear(std::false_type ) {
        }

        template <bool T> void _nextStepArc(std::integral_constant<bool, T> ) {
            _nextStep(_arcIters, AxisStepper::getNextTime(_arcIters));
        }
        void _nextStepArc(std::false_type ) {
        }
    public:
        OutputEvent peekNextEvent() {
            //called by State to query the next step in the current path segment, but NOT advance the iterator.
            //if (curOutputEvent != endOutputEvent) {
            if (_motionType != MotionNone) {
                return *curOutputEvent;
            } else {
                return OutputEvent();
            }
        }
        void consumeNextEvent() {
            //called by State to advance the event iterator (will usually be called directly after peekNextEvent, but also called at the start of a move internally)
            //Only advance the iterator if we aren't at the end of it (the only way we could be at the end of it is if the array contains 0 elements)
            if (curOutputEvent != endOutputEvent) {
                ++curOutputEvent;
            }
            while (curOutputEvent == endOutputEvent) {
                //we're at the end of the single step buffer. Refill it (_nextStep* will also reset the curOutputEvent index)
                //This must be a WHILE loop, because it's possible that the next step will have 0 output events (Although why, I can't imagine)
                switch (_motionType) {
                    case MotionLinear:
                        _nextStepLinear(std::integral_constant<bool, std::tuple_size<AxisStepperTypes>::value != 0>());
                        break;
                    case MotionArc:
                        LOGV("MotionPlanner::nextStep() _motionType is MotionArc\n");
                        _nextStepArc(std::integral_constant<bool, std::tuple_size<ArcStepperTypes>::value != 0>());
                        break;
                    case MotionNone:
                        return;
                    default: //impossible enum type
                        assert(false && "Impossible enum type");
                        return;
                }
            }
        }
        void moveTo(EventClockT::time_point baseTime, const Vector4f &dest_, float maxVelXyz, float minVelE, float maxVelE, MotionFlags flags=MOTIONFLAGS_DEFAULT) {
            //called by State to queue a movement from the current destination to a new one at (x, y, z, e), with the desired motion beginning at baseTime
            //Note: it is illegal to call this if readyForNextMove() != true
            if (std::tuple_size<AxisStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes
            }
            this->_baseTime = baseTime;
            this->_useEndstops = flags & USE_ENDSTOPS;
            Vector4f cur = _coordMapper.xyzeFromMechanical(_destMechanicalPos);
            Vector4f dest = dest_;
            if (! (flags & NO_LEVELING)) {
                //get the REAL destination, after leveling is applied
                dest = Vector4f(_coordMapper.applyLeveling(dest_.xyz()), dest_.e());
            }
            if (! (flags & NO_BOUNDING)) {
                //fix impossible coordinates
                dest = _coordMapper.bound(dest);
            }
            
            //Calculate velocities in x, y, z, e directions, and the duration of the linear movement:
            float dist = cur.xyz().distance(dest.xyz());
            float minDuration = dist/maxVelXyz; //duration, should there be no acceleration
            float velE = (dest.e() - cur.e())/minDuration;
            float newVelE = std::max(minVelE, std::min(maxVelE, velE));

            //in the case that newXYZ = currentXYZ, but extrusion is different, regulate that.
            if (velE != newVelE) { 
                velE = newVelE;
                minDuration = (dest.e()-cur.e())/newVelE;
                maxVelXyz = dist/minDuration;
            }
            
            Vector3f vel = (dest.xyz()-cur.xyz())/minDuration;
            LOGD("MotionPlanner::moveTo %s -> %s\n", cur.str().c_str(), dest.str().c_str());
            LOGD("MotionPlanner::moveTo _destMechanicalPos: (%i, %i, %i, %i)\n", _destMechanicalPos[0], _destMechanicalPos[1], _destMechanicalPos[2], _destMechanicalPos[3]);
            AxisStepper::initAxisSteppers(_iters, _useEndstops, _coordMapper, _destMechanicalPos, Vector4f(vel, velE));
            this->_duration = minDuration;
            this->_motionType = MotionLinear;
            this->_accel.begin(minDuration, maxVelXyz);
            //prepare the move buffer so that peekNextEvent() is valid
            consumeNextEvent();
        }

        void arcTo(EventClockT::time_point baseTime, const Vector4f &dest_, const Vector3f &center_, float maxVelXyz, float minVelE, float maxVelE, bool isCW, MotionFlags flags=MOTIONFLAGS_DEFAULT) {
            //called by State to queue a movement from the current destination to a new one at (x, y, z, e), with the desired motion beginning at baseTime
            //Note: it is illegal to call this if readyForNextMove() != true
            if (std::tuple_size<ArcStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes
            }
            this->_baseTime = baseTime;
            this->_useEndstops = flags & USE_ENDSTOPS;
            Vector4f cur = _coordMapper.xyzeFromMechanical(_destMechanicalPos);
            Vector4f dest = dest_;
            if (! (flags & NO_LEVELING)) {
                //get the REAL destination, after leveling is applied
                dest = Vector4f(_coordMapper.applyLeveling(dest_.xyz()), dest_.e());
            }
            if (! (flags & NO_BOUNDING)) {
                //fix impossible coordinates
                dest = _coordMapper.bound(dest);
            }
            
            //The 3 points, (centerX_, centerY_, centerZ_), (curX, curY, curZ) and (x, y, z) form a plane where the arc will reside.
            //get the REAL (leveled) center
            Vector3f center = (flags & NO_LEVELING) ? center_ : _coordMapper.applyLeveling(center_);
            Vector3f a = cur.xyz()-center;
            Vector3f b = dest.xyz()-center;

            //Need to adjust the center point such that it is equidistant from a and b.
            //Note: the set of points equidistant from a and b is described by the normal vector, n = (b-a)
            //  and a point on the plane, the midpoint between a and b: m = 1/2*(a+b)
            //Then we choose as our new center, the point on the plane that is closest to the old center, c.
            //Note that this is done just by moving c along n until it is on the plane:
            //    c
            //     \   |
            //      \  | n (normal vector)
            //       \ |
            //        \|
            //----*----m------------ (equidistant plane)
            //    ^- closest point on the plane to c.
            // the closest point on the plane is c +Z proj(mc->n) 
            //  (that is, the projection of the line from c to the midpoint of a and b, onto n; the component of mc parallel to n)
            //Note: proj(c->n) = (c . n / |n|^2)n
            Vector3f n = b - a;
            Vector3f mp = (b+center + a+center)*0.5;
            Vector3f projcmpn = (mp-center).proj(n);
            center += projcmpn;
            
            //recalculate our a and b vectors, relative to this new center-point:
            a = cur.xyz() - center; //relative *current* coordinates
            b = dest.xyz() - center; //relative *desired* coordinates
            //now solve for the arcLength and arcAngle
            //To get the arclength, we take the arcangle * arcRad.
            //a . b = |a| |b| cos(theta), so we can find the arcangle with arccos(a . b / |a| / |b|).
            //Note: |a| == |b| == arcRad, as these points are defined to be equi-distant from the center-point.

            float arcRad = a.mag();
            float arcAngle = acos(a.dot(b)/a.magSq()); //a . b = (r)*(r)*cos(theta)
            float arcLength = arcAngle*arcRad; //s = r*theta
            
            //Now that we have the arcLength, we can determine the optimal velocity:
            float minDuration = arcLength/maxVelXyz; //duration, should there be no acceleration
            float velE = (dest.e()-cur.e())/minDuration;
            float newVelE = std::max(minVelE, std::min(maxVelE, velE));
            if (velE != newVelE) { //limit cartesian velocity if it forces extrusion velocity to exceed a doable value
                velE = newVelE;
                minDuration = (dest.e()-cur.e())/newVelE;
                maxVelXyz = arcLength/minDuration;
            }
            float arcVel = maxVelXyz / arcRad;
                        
            //Want two perpindicular vectors such that <x, y, z> = P(t) = <xc, yc, zc> + r*cos(m*t)*u + r*sin(m*t)*v
            //Thus, u is the unit vector parallel to <x0, y0, z0> - <xc, yc, zc>
            Vector3f u = a.norm();
            
            // Now to solve for v, which must be perpindicular to u:
            // |
            // |   / b
            // |  /
            // | /
            // |/    u
            //  ------->
            // To create a unit vector v that is perpidicular to u
            // v is proportional to b - proj(b->u)
            // Therefore: v = (b - b.proj(b->u)).norm() 
            Vector3f v = (b-b.proj(u)).norm();

            //Given <x, y, z> = u*cos(t) + v*sin(t)
            //  if we are CCW, then u x v should be out of the page (+z)
            //  and if we are CW, then u x v should be into the page (-z)
            //If u x v isn't of the desired sign, then we can just invert v (but keep u the same so as not to change P(t=0))
            float uCrossV_z = u.cross(v).z();
            if ((isCW && uCrossV_z > 0) || (!isCW && uCrossV_z < 0)) { //fix direction:
                v = -v;
            }
            
            /*LOGD("MotionPlanner arc center (%f,%f,%f) current (%f,%f,%f) desired (%f,%f,%f) u (%f,%f,%f) v (%f,%f,%f) rad %f vel %f velE %f dur %f\n", 
                  center.x(), center.y(), center.z(), curX, curY, curZ, x, y, z, 
                  u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), arcRad, arcVel, velE, minDuration);
            LOGD("MotionPlanner arc orig center (%f,%f,%f), proj (%f,%f,%f) n(%f,%f,%f), mp(%f,%f,%f)\n", 
                centerX_, centerY_, centerZ_, projcmpn.x(), projcmpn.y(), projcmpn.z(),
                n.x(), n.y(), n.z(), mp.x(), mp.y(), mp.z());*/
            AxisStepper::initAxisArcSteppers(_arcIters, _useEndstops, _coordMapper, _destMechanicalPos, center, u, v, arcRad, arcVel, velE);
            /*if (std::tuple_size<ArcStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes. Place this as far along as possible so one can test most algorithms on the Example machine.
            }*/
            this->_duration = minDuration;
            this->_motionType = MotionArc;
            this->_accel.begin(minDuration, maxVelXyz);
            //prepare the move buffer so that peekNextEvent() is valid
            consumeNextEvent();
        }
};

}

#endif
