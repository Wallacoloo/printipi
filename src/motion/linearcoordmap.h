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
 
 
#ifndef MOTION_LINEARCOORDMAP_H
#define MOTION_LINEARCOORDMAP_H

#include <array>
#include <utility> //for std::move
#include <tuple>

#include "coordmap.h"
#include "common/matrix.h"
#include "linearstepper.h"
#include "motion/motionplanner.h" //for motion::USE_ENDSTOPS
#include "iodrivers/endstop.h"

namespace motion {

/* 
 * LinearCoordMap implements the CoordMap interface for Cartesian robots.
 * This is the CoordMap that your machine should use if it is a Cartesian robot.
 */
template <typename Stepper1, typename Stepper2, typename Stepper3, typename Stepper4, typename BedLevelT=Matrix3x3> class LinearCoordMap : public CoordMap {
    typedef std::tuple<Stepper1, Stepper2, Stepper3, Stepper4> StepperDriverTypes;
    typedef std::tuple<LinearStepper<Stepper1>, 
                       LinearStepper<Stepper2>, 
                       LinearStepper<Stepper3>, 
                       LinearStepper<Stepper4> > _AxisStepperTypes;

    float _STEPS_MM_X, _MM_STEPS_X;
    float _STEPS_MM_Y, _MM_STEPS_Y;
    float _STEPS_MM_Z, _MM_STEPS_Z;
    float _STEPS_MM_E, _MM_STEPS_E;
    float homeVelocity;
    BedLevelT bedLevel;
    std::array<iodrv::Endstop, 4> endstops; //x, y, z, e (null)
    StepperDriverTypes stepperDrivers;
    public:
        inline float STEPS_MM(std::size_t axisIdx) const { 
            return axisIdx == CARTESIAN_AXIS_X ? _STEPS_MM_X
              :   (axisIdx == CARTESIAN_AXIS_Y ? _STEPS_MM_Y
              :   (axisIdx == CARTESIAN_AXIS_Z ? _STEPS_MM_Z : _STEPS_MM_E));
        }
        inline float MM_STEPS(std::size_t axisIdx) const { 
            return axisIdx == CARTESIAN_AXIS_X ? _MM_STEPS_X
              :   (axisIdx == CARTESIAN_AXIS_Y ? _MM_STEPS_Y
              :   (axisIdx == CARTESIAN_AXIS_Z ? _MM_STEPS_Z : _MM_STEPS_E));
        }
        inline LinearCoordMap(float STEPS_MM_X, float STEPS_MM_Y, float STEPS_MM_Z, float STEPS_MM_E, float homeVelocity, 
          Stepper1 &&stepper1, Stepper2 &&stepper2, Stepper3 &&stepper3, Stepper4 &&stepper4,
          iodrv::Endstop &&endstopX, iodrv::Endstop &&endstopY, iodrv::Endstop &&endstopZ,
          const BedLevelT& t)
         : _STEPS_MM_X(STEPS_MM_X), _MM_STEPS_X(1. / STEPS_MM_X),
           _STEPS_MM_Y(STEPS_MM_Y), _MM_STEPS_Y(1. / STEPS_MM_Y),
           _STEPS_MM_Z(STEPS_MM_Z), _MM_STEPS_Z(1. / STEPS_MM_Z),
           _STEPS_MM_E(STEPS_MM_E), _MM_STEPS_E(1. / STEPS_MM_E),
           homeVelocity(homeVelocity),
           bedLevel(t),
           endstops({{std::move(endstopX), std::move(endstopY), std::move(endstopZ), std::move(iodrv::Endstop())}}),
           stepperDrivers(std::move(stepper1), std::move(stepper2), std::move(stepper3), std::move(stepper4)) {}
        inline std::tuple<Stepper1&, Stepper2&, Stepper3&, Stepper4&, 
          iodrv::Endstop&, iodrv::Endstop&, iodrv::Endstop&> getDependentIoDrivers() {
            return std::tie(
                std::get<0>(stepperDrivers), 
                std::get<1>(stepperDrivers), 
                std::get<2>(stepperDrivers), 
                std::get<3>(stepperDrivers),
                endstops[0],
                endstops[1],
                endstops[2]);
        }
        inline _AxisStepperTypes getAxisSteppers() const {
            return std::make_tuple(
                LinearStepper<Stepper1>(0, CARTESIAN_AXIS_X, *this, std::get<0>(stepperDrivers), &endstops[0]), 
                LinearStepper<Stepper2>(1, CARTESIAN_AXIS_Y, *this, std::get<1>(stepperDrivers), &endstops[1]), 
                LinearStepper<Stepper3>(2, CARTESIAN_AXIS_Z, *this, std::get<2>(stepperDrivers), &endstops[2]), 
                LinearStepper<Stepper4>(3, CARTESIAN_AXIS_E, *this, std::get<3>(stepperDrivers), &endstops[3])
            );
        }

        inline static constexpr std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        inline int getAxisPosition(const std::array<int, 4> &cur, std::size_t axis) const {
            return cur[axis];
        }
        template <typename Interface> void executeHomeRoutine(Interface &interface) {
            //must disable buffering so that endstops can be reliably checked
            //interface.setUnbufferedMove(true);
            Vector4f curPos = interface.actualCartesianPosition();
            //try to move each axis towards the endstops
            Vector4f destPos = curPos + Vector4f(-1000, -1000, -1000, 0);
            interface.moveTo(destPos, homeVelocity, motion::USE_ENDSTOPS | motion::NO_LEVELING | motion::NO_BOUNDING);
            //reset the indexed axis positions:
            interface.resetAxisPositions(getHomePosition(interface.axisPositions()));
            //interface.setUnbufferedMove(false);
        }
        inline std::array<int, 4> getHomePosition(const std::array<int, 4> &cur) const {
            return std::array<int, 4>({{0, 0, 0, cur[3]}});
        }
        inline Vector3f applyLeveling(const Vector3f &xyz) const {
            return bedLevel.transform(xyz);
        }
        inline Vector4f bound(const Vector4f &xyze) const {
            return xyze; //no bounding.
        }
        inline Vector4f xyzeFromMechanical(const std::array<int, 4> &mech) const {
            return Vector4f(mech[CARTESIAN_AXIS_X]*_MM_STEPS_X, 
                                   mech[CARTESIAN_AXIS_Y]*_MM_STEPS_Y, 
                                   mech[CARTESIAN_AXIS_Z]*_MM_STEPS_Z, 
                                   mech[CARTESIAN_AXIS_E]*_MM_STEPS_E);
        }

};

}

#endif
