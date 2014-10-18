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
 * Printipi/drivers/coordmap.h
 *
 * CoordMaps are used to translate cartesian coordinates to and from the machine's coordinate system
 * This allows for a bot to internally use a non-cartesian coordinate system - very useful for delta bots.
 *
 * Note: CoordMap is an interface, and not an implementation.
 * An implementation is needed for each coordinate style - Cartesian, deltabot, etc.
 * These implementations must provide the functions outlined further down in the header.
 */

#ifndef DRIVERS_COORDMAP_H
#define DRIVERS_COORDMAP_H

//#include <tuple> //needed for children

namespace drv {

class CoordMap {
    public:
        //static std::tuple<float, float, float, float> xyzeFromMechanical(const std::array<int, N>&);
        //constexpr static std::size_t numAxis();
        //return the home position, in cartesian coordinates:
        //static constexpr std::array<int, 4> getHomePosition(const std::array<int, 4> &cur)
};

}


#endif
