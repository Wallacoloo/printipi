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

#ifndef MOTION_COORDMAP_H
#define MOTION_COORDMAP_H

#include <tuple>
#include <array>

#include "common/vector3.h"
#include "common/vector4.h"
#include "common/logging.h"

namespace iodrv { 
    //forward declare for class in "endstop.h"
    class Endstop; 
}

namespace motion {

/* 
 * CoordMaps are used to translate cartesian coordinates to and from the machine's coordinate system
 * This allows for a bot to internally use a non-cartesian coordinate system - very useful for delta bots.
 *
 * Note: CoordMap is an interface, and not an implementation.
 * An implementation is needed for each coordinate style - Cartesian, deltabot, etc.
 * These implementations must provide the functions outlined further down in the header.
 *
 * Note: the term "axis coordinates" refers to an individual motor's microstep index (no wraparound).
 *   whereas "cartesian coordinates" refers to the print head's location in [x, y, z[, e]] space in units of mm.
 *   For example, if your Z motor has 1000 steps/mm and is 200 mm tall, then an axis coordinate of 0 translates to a cartesian Z coordinate of 0mm,
 *     and an axis coordinate of 200000 translates to a cartesian Z coordinate of 200mm.
 */
class CoordMap {
    public:
        //If the coordmap internally contains IoDrivers (e.g. stepper drivers or servos to lower the probe), then it should return a tuple of REFERENCES to them.
        inline std::tuple<> getDependentIoDrivers() {
            return std::tuple<>();
        }
        inline std::tuple<> getAxisSteppers() const {
            //return a tuple of uninitialized AxisSteppers for moving in a straight line. See axisstepper.h for the definition of AxisStepper.
            return std::tuple<>();
        }
        //return a reference to the Endstop associated with a given axis (eg X-endstop, Y-endstop, etc)
        //Depending on which AxisSteppers your CoordMap uses, you may not need to implement this function.
        inline const iodrv::Endstop& getEndstop(std::size_t axis) const {
            (void)axis; //unused
            return *(iodrv::Endstop*)(nullptr);
        }
        //return the number of axis (physical motors) that we have.
        inline static constexpr std::size_t numAxis() {
            return 0;
        }
        //Home the machine by sending commands to @interface
        template <typename Interface> void executeHomeRoutine(Interface &interface) {
            interface.resetAxisPositions(getHomePosition(interface.axisPositions()));
        }
        //given the current tracked motor coordinates, and knowing that we are at home position,
        //@return the true motor coordinates.
        inline std::array<int, 0> getHomePosition(const std::array<int, 0> &cur) const {
            (void)cur;
            return std::array<int, 0>();
        }
        //apply some leveling transformation to the [x,y,z] cartesian coordinate to compensate for an unlevel bed.
        //Note: this is only applied to the endpoints of a line, so a non-planar bed cannot properly be leveled.
        inline Vector3f applyLeveling(const Vector3f &xyz) const {
            return xyz;
        }
        //ensure that the desired coordinate is accessible. (i.e. motors won't ram the endstops, etc).
        inline Vector4f bound(const Vector4f &xyze) const {
            return xyze;
        }
        //given axis coordinates &mech, calculate the cartesian [x,y,z,e] coordinates that the printhead is at.
        inline Vector4f xyzeFromMechanical(const std::array<int, 0> &mech) const {
            (void)mech; //unused in this stub
            LOGW_ONCE("xyzeFromMechanical should be implemented in CoordMap implementations\n");
            return Vector4f(0, 0, 0, 0);
        }
        //if we get a G1 before the first G28, then we *probably* want to home first,
        //    but feel free to override this in other implementations.
        inline bool doHomeBeforeFirstMovement() const {
            return true;
        }
};

}


#endif
