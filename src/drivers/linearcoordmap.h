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

template <typename Transform=matr::Identity3Static> class LinearCoordMap : public CoordMap {
    static constexpr std::size_t xIdx = 0;
    static constexpr std::size_t yIdx = 1;
    static constexpr std::size_t zIdx = 2;
    static constexpr std::size_t eIdx = 3;
    //Transform transform;
    public:
        static constexpr std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        static constexpr std::array<int, 4> getHomePosition(const std::array<int, 4> &cur) {
            return std::array<int, 4>({0, 0, 0, cur[3]});
        }
        static std::tuple<float, float, float> applyLeveling(const std::tuple<float, float, float> &xyz) {
            return Transform::transform(xyz);
        }
        static std::tuple<float, float, float, float> bound(const std::tuple<float, float, float, float> &xyze) {
            return xyze; //no bounding.
        }
        static std::tuple<float, float, float, float> xyzeFromMechanical(const std::array<int, 4> &mech) {
            return std::make_tuple(mech[xIdx], mech[yIdx], mech[zIdx], mech[eIdx]);
        }

};

}

#endif
