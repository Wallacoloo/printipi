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
 * This is a minimal Machine that will respond to gcode commands,
 *   but won't actually do any movements or control hotends, fans, etc.
 */

#ifndef DRIVERS_MACHINES_EXAMPLE_H
#define DRIVERS_MACHINES_EXAMPLE_H

#include <tuple>
#include "motion/constantacceleration.h"
#include "motion/linearcoordmap.h"
#include "machines/machine.h"
#include "common/matrix.h"
#include "drivers/endstop.h"

namespace machines {
namespace generic {

using namespace drv; //for all the drivers
using namespace motion; //for Acceleration & such

class example : public Machine {
    public:
        inline ConstantAcceleration getAccelerationProfile() const {
            return ConstantAcceleration(500);
        }
        inline LinearCoordMap<> getCoordMap() const {
            return LinearCoordMap<>(1.0, 1.0, 1.0, 1.0, //steps/mm in X, Y, Z, E axis
                Endstop(), Endstop(), Endstop(), //null endstops
                Matrix3x3( //bed level matrix
            1, 0, 0,
            0, 1, 0,
            0, 0, 1));
        }
        inline std::tuple<> getIoDrivers() const {
            return std::tuple<>(); //no I/O capability
        }
};

}
}

#endif
