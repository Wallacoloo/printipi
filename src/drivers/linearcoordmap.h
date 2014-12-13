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
 * Printipi/drivers/linearcoordmap.h
 *
 * LinearCoordMap implements the CoordMap interface for Cartesian robots.
 * This is the CoordMap that your driver (src/drivers/driver.h implementation) should use if it is a Cartesian robot.
 */
 
 
#ifndef DRIVERS_LINEARCOORDMAP_H
#define DRIVERS_LINEARCOORDMAP_H

#include "coordmap.h"
#include "common/matrix.h"

namespace drv {

enum CartesianAxis {
    CARTESIAN_AXIS_X=0,
    CARTESIAN_AXIS_Y=1,
    CARTESIAN_AXIS_Z=2,
    CARTESIAN_AXIS_E=3
};

template <typename BedLevelT=Matrix3x3> class LinearCoordMap : public CoordMap {
    float _STEPS_MM_X, _MM_STEPS_X;
    float _STEPS_MM_Y, _MM_STEPS_Y;
    float _STEPS_MM_Z, _MM_STEPS_Z;
    float _STEPS_MM_E, _MM_STEPS_E;
    BedLevelT bedLevel;
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
        inline LinearCoordMap(float STEPS_MM_X, float STEPS_MM_Y, float STEPS_MM_Z, float STEPS_MM_E, const BedLevelT& t)
         : _STEPS_MM_X(STEPS_MM_X), _MM_STEPS_X(1. / STEPS_MM_X),
           _STEPS_MM_Y(STEPS_MM_Y), _MM_STEPS_Y(1. / STEPS_MM_Y),
           _STEPS_MM_Z(STEPS_MM_Z), _MM_STEPS_Z(1. / STEPS_MM_Z),
           _STEPS_MM_E(STEPS_MM_E), _MM_STEPS_E(1. / STEPS_MM_E),
           bedLevel(t) {}
        inline static constexpr std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        inline std::array<int, 4> getHomePosition(const std::array<int, 4> &cur) const {
            return std::array<int, 4>({0, 0, 0, cur[3]});
        }
        inline std::tuple<float, float, float> applyLeveling(const std::tuple<float, float, float> &xyz) const {
            return bedLevel.transform(xyz);
        }
        inline std::tuple<float, float, float, float> bound(const std::tuple<float, float, float, float> &xyze) const {
            return xyze; //no bounding.
        }
        inline std::tuple<float, float, float, float> xyzeFromMechanical(const std::array<int, 4> &mech) const {
            return std::make_tuple(mech[CARTESIAN_AXIS_X], mech[CARTESIAN_AXIS_Y], mech[CARTESIAN_AXIS_Z], mech[CARTESIAN_AXIS_E]);
        }

};

}

#endif
