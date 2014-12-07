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
#include <cassert>
#include <stdexcept> //for runtime_error
#include "accelerationprofile.h"
#include "drivers/axisstepper.h"
#include "event.h"
#include "common/vector3.h"

//There are 3 distinct types of motion that can occur at any given time:
enum MotionType {
    MotionNone,
    MotionLinear,
    MotionArc,
    MotionHome
};

template <typename Interface> class MotionPlanner {
    private:
        typedef typename Interface::CoordMapT CoordMapT;
        typedef typename Interface::AxisStepperTypes AxisStepperTypes;
        typedef typename Interface::AccelerationProfileT AccelerationProfileT;
        typedef typename Interface::AxisHomeStepperTypes HomeStepperTypes;
        typedef typename Interface::AxisArcStepperTypes ArcStepperTypes;
        Interface _interface;
        CoordMapT _coordMapper; //object that maps from (x, y, z) to mechanical coords (eg A, B, C for a kossel)
        AccelerationProfileT _accel; //transforms the constant-velocity motion stream into one that considers acceleration
        std::array<int, CoordMapT::numAxis()> _destMechanicalPos; //the mechanical position of the last step that was scheduled
        AxisStepperTypes _iters; //Each axis iterator reports the next time it needs to be stepped. _iters is for linear movement
        HomeStepperTypes _homeIters; //Axis iterators used when homing
        ArcStepperTypes _arcIters;
        EventClockT::duration _baseTime; //The time at which the current path segment began (this will be a fraction of a second before the time which the first step in this path is scheduled for)
        float _duration; //the estimated duration of the current piece, not taking into account acceleration
        MotionType _motionType; //which type of segment is being planned
    public:
        MotionPlanner(const Interface &interface) : 
            _interface(interface),
            _coordMapper(interface.getCoordMap()),
            _accel(interface.getAccelerationProfile()), 
            _destMechanicalPos(), 
            _iters(interface.getAxisSteppers()), _homeIters(interface.getHomeSteppers()), _arcIters(interface.getArcSteppers()),
            _baseTime(), 
            _duration(NAN),
            _motionType(MotionNone) {}
        bool readyForNextMove() const {
            //returns true if a call to moveTo() or homeEndstops() wouldn't hang, false if it would hang (or cause other problems)
            //Note: for now, there isn't actually buffering.
            return _motionType == MotionNone;
        }
    private:
        Event _nextStep(drv::AxisStepper &s) {
            LOGV("MotionPlanner::nextStep() is: %i at %g of %g\n", s.index(), s.time, _duration);
            if (s.time > _duration || s.time <= 0 || std::isnan(s.time)) { //if the next time the given axis wants to step is invalid or past the movement length, then end the motion
                //Note: don't combine s.time <= 0 || isnan(s.time) to !(s.time > 0) because that might be broken during optimizations.
                //Note: This conditional causes the MotionPlanner to always undershoot the desired position, when it may be desireable to overshoot some of them - see https://github.com/Wallacoloo/printipi/issues/15
                if (isHoming()) { 
                    //if homing, then we now know the axis mechanical positions; fetch them
                    _destMechanicalPos = _coordMapper.getHomePosition(_destMechanicalPos);
                }
                //log debug info:
                float x, y, z, e;
                std::tie(x, y, z, e) = _coordMapper.xyzeFromMechanical(_destMechanicalPos);
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
            //advance the respective AxisStepper to its next step:
            switch (_motionType) {
                case MotionHome:
                    s.nextStep(_homeIters);
                    break;
                case MotionLinear:
                    s.nextStep(_iters);
                    break;
                case MotionArc:
                    s.nextStep(_arcIters);
                    break;
                case MotionNone:
                default:
                    assert(false); //it is illegal to call nextStep() when a move is not in progress.
            }
            return e;
        }
        //black magic to get nextStep to work when either AxisStepperTypes or HomeStepperTypes have length 0:
        //If they are length zero, then _nextStep* just returns an empty event and a compilation error is avoided.
        //Otherwise, the templated function is called, and _nextStep is run as usual:
        template <bool T> Event _nextStepHome(std::integral_constant<bool, T> ) {
            return _nextStep(drv::AxisStepper::getNextTime(_homeIters));
        }
        Event _nextStepHome(std::false_type ) {
            return Event();
        }
        template <bool T> Event _nextStepLinear(std::integral_constant<bool, T> ) {
            return _nextStep(drv::AxisStepper::getNextTime(_iters));
        }
        Event _nextStepLinear(std::false_type ) {
            return Event();
        }
        template <bool T> Event _nextStepArc(std::integral_constant<bool, T> ) {
            return _nextStep(drv::AxisStepper::getNextTime(_arcIters));
        }
        Event _nextStepArc(std::false_type ){
            return Event();
        }
    public:
        bool isHoming() const {
            return _motionType == MotionHome;
        }
        Event nextStep() {
            //called by State to query the next step in the current path segment
            switch (_motionType) {
                case MotionHome:
                    return _nextStepHome(std::integral_constant<bool, std::tuple_size<HomeStepperTypes>::value != 0>());
                case MotionLinear:
                    return _nextStepLinear(std::integral_constant<bool, std::tuple_size<AxisStepperTypes>::value != 0>());
                case MotionArc:
                    LOGV("MotionPlanner::nextStep() _motionType is MotionArc\n");
                    return _nextStepArc(std::integral_constant<bool, std::tuple_size<ArcStepperTypes>::value != 0>());
                case MotionNone:
                    return Event();
                default:
                    assert(false);
                    return Event();
            }
        }
        void moveTo(EventClockT::time_point baseTime, float x, float y, float z, float e, float maxVelXyz, float minVelE, float maxVelE) {
            //called by State to queue a movement from the current destination to a new one at (x, y, z, e), with the desired motion beginning at baseTime
            //Note: it is illegal to call this if readyForNextMove() != true
            if (std::tuple_size<AxisStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes
            }
            this->_baseTime = baseTime.time_since_epoch();
            float curX, curY, curZ, curE;
            std::tie(curX, curY, curZ, curE) = _coordMapper.xyzeFromMechanical(_destMechanicalPos);
            std::tie(x, y, z) = _coordMapper.applyLeveling(std::make_tuple(x, y, z)); //get the REAL destination.
            std::tie(x, y, z, e) = _coordMapper.bound(std::make_tuple(x, y, z, e)); //Fix impossible coordinates
            
            //Calculate velocities in x, y, z, e directions, and the duration of the linear movement:
            float distSq = (x-curX)*(x-curX) + (y-curY)*(y-curY) + (z-curZ)*(z-curZ);
            float dist = sqrt(distSq);
            float minDuration = dist/maxVelXyz; //duration, should there be no acceleration
            float velE = (e-curE)/minDuration;
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
            drv::AxisStepper::initAxisSteppers(_iters, _coordMapper, _destMechanicalPos, vx, vy, vz, velE);
            this->_duration = minDuration;
            this->_motionType = MotionLinear;
            this->_accel.begin(minDuration, maxVelXyz);
        }

        void homeEndstops(EventClockT::time_point baseTime, float maxVelXyz) {
            //Called by State to begin a motion that homes to the endstops (and stays there)
            //Note: it is illegal to call this if readyForNextMove() != true
            if (std::tuple_size<HomeStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes
            }
            drv::AxisStepper::initAxisHomeSteppers(_homeIters, _coordMapper, maxVelXyz);
            this->_baseTime = baseTime.time_since_epoch();
            this->_duration = NAN;
            this->_motionType = MotionHome;
            this->_accel.begin(NAN, maxVelXyz);
        }
        void arcTo(EventClockT::time_point baseTime, float x, float y, float z, float e, float centerX_, float centerY_, float centerZ_, float maxVelXyz, float minVelE, float maxVelE, bool isCW) {
            //called by State to queue a movement from the current destination to a new one at (x, y, z, e), with the desired motion beginning at baseTime
            //Note: it is illegal to call this if readyForNextMove() != true
            /*if (std::tuple_size<ArcStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes
            }*/
            this->_baseTime = baseTime.time_since_epoch();
            float curX, curY, curZ, curE;
            std::tie(curX, curY, curZ, curE) = _coordMapper.xyzeFromMechanical(_destMechanicalPos);
            std::tie(x, y, z) = _coordMapper.applyLeveling(std::make_tuple(x, y, z)); //get the REAL destination.
            std::tie(x, y, z, e) = _coordMapper.bound(std::make_tuple(x, y, z, e)); //Fix impossible coordinates
            
            //The 3 points, (centerX, centerY, centerZ), (curX, curY, curZ) and (x, y, z) form a plane where the arc will reside.
            //To get the arclength, we take the arcangle * arcRad.
            //a . b = |a| |b| cos(theta), so we can find the arcangle with arccos(a . b / |a| / |b|).
            //Note: |a| == |b| == arcRad, as these points are defined to be equi-distant from the center-point.
            //TODO: in actuality, the mechanical limits will be such that arcs WILL propagate error unless we adjust the center to make (xCur, yCur, zCur) at the same radius as (x, y, z)
            /*float aX = curX-centerX; //relative *current* coordinates
            float aY = curY-centerY;
            float aZ = curZ-centerZ;
            float bX = x-centerX; //relative *desired* coordinates
            float bY = y-centerY;
            float bZ = z-centerZ;*/
            Vector3f center(centerX_, centerY_, centerZ_);
            Vector3f a(curX-centerX_, curY-centerY_, curZ-centerZ_);
            Vector3f b(x-centerX_, y-centerY_, z-centerZ_);

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
            // the closest point on the plane is c + proj(cm->n) 
            //  (that is, the projection of the line from c to the midpoint of a and b, onto n; the component of cm parallel to n)
            //Note: proj(c->n) = (c . n / |n|^2)n
            /*float nX = bX-aX;
            float nY = bY-aY;
            float nZ = bZ-aZ;*/
            Vector3f n = b - a;
            Vector3f mp = (b+a)*0.5;
            //float magNSq = nX*nX + nY*nY + nZ*nZ;
            float magNSq = n.magSq();
            //float projcmp_nX = (mp.x()-centerX)*n.x() / magNSq * n.x();
            //float projcmp_nY = (mp.y()-centerY)*n.y() / magNSq * n.y();
            //float projcmp_nZ = (mp.z()-centerZ)*n.z() / magNSq * n.z();
            //centerX += projcmp_nX;
            //centerY += projcmp_nY;
            //centerZ += projcmp_nZ;
            Vector3f projcmpn = (mp-center).proj(n);
            center += projcmpn;
            
            //recalculate our a and b vectors, relative to this new center-point:
            //a = Vector3f(curX-centerX, curY-centerY, curZ-centerZ); //relative *current* coordinates
            //b = Vector3f(x-centerX, y-centerY, z-centerZ); //relative *desired* coordinates
            a = Vector3f(curX, curY, curZ) - center; //relative *current* coordinates
            b = Vector3f(x, y, z) - center; //relative *desired* coordinates
            //now solve for the arcLength and arcAngle
            float aDotB = a.dot(b);
            float arcRadSq = a.magSq();
            float arcRad = sqrt(arcRadSq);
            float arcAngle = acos(aDotB/arcRadSq);
            float arcLength = arcAngle*arcRad;
            
            //Now that we have the arcLength, we can determine the optimal velocity:
            float minDuration = arcLength/maxVelXyz; //duration, should there be no acceleration
            float velE = (e-curE)/minDuration;
            float newVelE = std::max(minVelE, std::min(maxVelE, velE));
            if (velE != newVelE) { //limit cartesian velocity if it forces extrusion velocity to exceed a doable value
                velE = newVelE;
                minDuration = (e-curE)/newVelE;
                maxVelXyz = arcLength/minDuration;
            }
            float arcVel = maxVelXyz / arcRad;
                        
            //Want two perpindicular vectors such that <x, y, z> = P(t) = <xc, yc, zc> + r*cos(m*t)*u + r*sin(m*t)*v
            //Thus, u is the normal vector of <x0, y0, z0> - <xc, yc, zc>
            float magA = a.mag();
            Vector3f u = a/magA;
            
            //Now to get v; v is some linear combination of u + <x1-xc, y1-yc, z1-zc> that is perpindicular to u and has length 1.
            //So, v = au + b<x1-xc, y1-yc, z1-zc>
            //and u.v = 0 = a*1 + b<x1-xc, y1-yc, z1-zc> . u
            //a = -b<x1-xc, y1-yc, z1-zc> . u
            //let b=1 and solve for a. This gives us our direction, but non-normalized:
            //a = -[(x1-xc)*ux + (y1-yc)*uy + (z1-zc)*uz]
            //Then normalize.
            //TODO: Another way to put this may be that v is <x1-xc, y1-yc, z1-zc> minus the projection of <x1-xc, y1-yc, z1-zc> onto u, normalized.
            
            //float a = -b.dot(u); //-(bX*u.x() + bY*u.y() + bZ*u.z());
            //Vector3f v = a*u - b;
            Vector3f v = u*-b.dot(u) - b;
            float magV = v.mag();
            v = v/magV;

            //Given <x, y, z> = u*cos(t) + v*sin(t)
            //  if we are CCW, then u x v should be out of the page (+z)
            //  and if we are CW, then u x v should be into the page (-z)
            //If u x v isn't of the desired sign, then we can just invert v (but keep u the same!)
            //float uCrossV_z = ux*vy - uy*vx; //z component of u x v
            float uCrossV_z = u.cross(v).z();
            if ((isCW && uCrossV_z > 0) || (!isCW && uCrossV_z < 0)) { //fix direction:
                v = v*-1.f;
            }
            
            //LOGD("MotionPlanner arc center (%f,%f,%f) current (%f,%f,%f) desired (%f,%f,%f) phase (%f,%f,%f) rad %f vel %f velE %f dur %f\n", centerX, centerY, centerZ, curX, curY, curZ, x, y, z, xAng, yAng, zAng, arcRad, arcVel, velE, minDuration);
            LOGD("MotionPlanner arc center (%f,%f,%f) current (%f,%f,%f) desired (%f,%f,%f) u (%f,%f,%f) v (%f,%f,%f) rad %f vel %f velE %f dur %f\n", 
                  center.x(), center.y(), center.z(), curX, curY, curZ, x, y, z, 
                  u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), arcRad, arcVel, velE, minDuration);
            LOGD("MotionPlanner arc orig center (%f,%f,%f), proj (%f,%f,%f) n(%f,%f,%f), mp(%f,%f,%f)\n", 
                centerX_, centerY_, centerZ_, projcmpn.x(), projcmpn.y(), projcmpn.z(),
                n.x(), n.y(), n.z(), mp.x(), mp.y(), mp.z());
            drv::AxisStepper::initAxisArcSteppers(_arcIters, _coordMapper, _destMechanicalPos, center.x(), center.y(), center.z(), u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), arcRad, arcVel, velE);
            if (std::tuple_size<ArcStepperTypes>::value == 0) {
                return; //Prevents hanging on machines with 0 axes. Place this as far along as possible so one can test most algorithms on the Example machine.
            }
            this->_duration = minDuration;
            this->_motionType = MotionArc;
            this->_accel.begin(minDuration, maxVelXyz);
        }
};


#endif
