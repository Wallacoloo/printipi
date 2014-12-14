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
 * Printipi/motion/lineardeltacoordmap.h
 * 
 * LinearDeltaCoordMap implements the CoordMap interface for (rail-based) Delta-style robots like the Kossel
 * This class allows for translating mechanical positions to the cartesian x, y, z, e system.
 * It assumes there are 3 legs arranged in a circle (120 degrees between each adjacent pair)
 *   and these legs have carriages a distance d from their base.
 * The leg at (x=0, y=+) is axis A,
 * The leg at (x>0, y<0) is axis B,
 * The leg at (x<0, y<0) is axis C
 * Additionally, the carriages host a free-spinning joint with an arm of length L linked to an end effector,
 * and the carriages are r units apart.
 *
 * R1000 is 'r' (in mm) multiplied by 1000,
 * L1000 is 'L' (in mm) multiplied by 1000
 *
 * The math is described more in /code/proof-of-concept/coordmath.py and coord-math.nb (note: file has been deleted; must view an archived version of printipi on Github to view this documentation)
 */

#ifndef MOTION_LINEARDELTACOORDMAP_H
#define MOTION_LINEARDELTACOORDMAP_H

#include "coordmap.h"
#include "common/logging.h"
#include "common/matrix.h"
#include "lineardeltastepper.h"
#include "iodrivers/endstop.h"
#include <array>
#include <tuple>
#include <utility> //for std::move

namespace motion {

template <typename BedLevelT=Matrix3x3> class LinearDeltaCoordMap : public CoordMap {
    static constexpr float MIN_Z() { return -2; } //useful to be able to go a little under z=0 when tuning.
    float _r, _L, _h, _buildrad;
    float _STEPS_MM, _MM_STEPS;
    float _STEPS_MM_EXT, _MM_STEPS_EXT;
    BedLevelT bedLevel;

    std::array<drv::Endstop, 4> endstops; //A, B, C and E (E is null)

    typedef std::tuple<LinearDeltaStepper<DELTA_AXIS_A>, 
                       LinearDeltaStepper<DELTA_AXIS_B>, 
                       LinearDeltaStepper<DELTA_AXIS_C>, 
                       LinearStepper<CARTESIAN_AXIS_E> > _AxisStepperTypes;
    typedef AxisStepper::GetHomeStepperTypes<_AxisStepperTypes>::HomeStepperTypes _HomeStepperTypes;
    typedef AxisStepper::GetArcStepperTypes<_AxisStepperTypes>::ArcStepperTypes _ArcStepperTypes;
    private:
        inline float STEPS_MM() const { return _STEPS_MM; }
        inline float MM_STEPS() const { return _MM_STEPS; }
        inline float STEPS_MM_EXT() const { return _STEPS_MM_EXT; }
        inline float MM_STEPS_EXT() const { return _MM_STEPS_EXT; }
    public:
        inline float r() const { return _r; }
        inline float L() const { return _L; }
        inline float h() const { return _h; }
        inline float buildrad() const { return _buildrad; }
        inline float STEPS_MM(std::size_t axisIdx) const { return axisIdx == DELTA_AXIS_E ? STEPS_MM_EXT() : STEPS_MM(); }
        inline float MM_STEPS(std::size_t axisIdx) const { return axisIdx == DELTA_AXIS_E ? MM_STEPS_EXT() : MM_STEPS(); }
        inline LinearDeltaCoordMap(float r, float L, float h, float buildrad, float STEPS_MM, float STEPS_MM_EXT, 
            drv::Endstop &&endstopA, drv::Endstop &&endstopB, drv::Endstop &&endstopC, const BedLevelT &t)
         : _r(r), _L(L), _h(h), _buildrad(buildrad),
           _STEPS_MM(STEPS_MM), _MM_STEPS(1. / STEPS_MM),
           _STEPS_MM_EXT(STEPS_MM_EXT), _MM_STEPS_EXT(1./ STEPS_MM_EXT),
           bedLevel(t),
           endstops({{std::move(endstopA), std::move(endstopB), std::move(endstopC), std::move(drv::Endstop())}}) {}
        inline _AxisStepperTypes getAxisSteppers() const {
            return _AxisStepperTypes();
        }
        inline _HomeStepperTypes getHomeSteppers() const {
            return _HomeStepperTypes();
        }
        inline _ArcStepperTypes getArcSteppers() const {
            return _ArcStepperTypes();
        }

        inline static constexpr std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        inline int getAxisPosition(const std::array<int, 4> &cur, std::size_t axis) const {
            return cur[axis];
        }
        inline const drv::Endstop& getEndstop(std::size_t axis) const {
            return endstops[axis];
        }
        inline std::array<int, 4> getHomePosition(const std::array<int, 4> &cur) const {
            return std::array<int, 4>({{(int)(h()*STEPS_MM()), (int)(h()*STEPS_MM()), (int)(h()*STEPS_MM()), cur[3]}});
        }
        inline std::tuple<float, float, float> applyLeveling(const std::tuple<float, float, float> &xyz) const {
            return bedLevel.transform(xyz);
        }
        inline std::tuple<float, float, float, float> bound(const std::tuple<float, float, float, float> &xyze) const {
            //bound z:
            float z = std::max(MIN_Z(), std::min((float)((h()+sqrt(L()*L()-r()*r()))*STEPS_MM()), std::get<2>(xyze)));
            float x = std::get<0>(xyze);
            float y = std::get<1>(xyze);
            if (x*x + y*y > buildrad()*buildrad()) { //bring x, y onto the platform.
                float ratio = std::sqrt(buildrad()*buildrad() / (x*x + y*y));
                x *= ratio;
                y *= ratio;
            }
            //TODO: force x & y to be on the platform.
            return std::make_tuple(x, y, z, std::get<3>(xyze));
        }
        inline std::tuple<float, float, float, float> xyzeFromMechanical(const std::array<int, 4> &mech) const {
            float e = mech[DELTA_AXIS_E]*MM_STEPS_EXT();
            float x, y, z;
            float A = mech[DELTA_AXIS_A]*MM_STEPS(); //convert mechanical positions (steps) to MM.
            float B = mech[DELTA_AXIS_B]*MM_STEPS();
            float C = mech[DELTA_AXIS_C]*MM_STEPS();
            if (A == B && B == C) { //prevent a division-by-zero.
                LOGV("LinearDeltaCoordMap::A==B==C\n");
                x = 0;
                y = 0;
                z = A-sqrt(L()*L()-r()*r());
                //LOGV("LinearDeltaCoordMap::z=%f (%f)\n", z, A-sqrt(L*L-r*r));
            } else if (B == C) { //prevent a division-by-zero.
                LOGV("LinearDeltaCoordMap::A!=B==C\n");
                /*ydiv = (2.*(4*A*A - 8*A*B + 4*B*B + 9*r*r))
                ya = 2*(A-B)*(A-B)*r
                yb = 4*Sqrt((A - B)*(A - B)*(-(A - B)*(A - B)*(A - B)*(A - B) + 4*(A - B)*(A - B)*L*L + 3*(-2*(A - B)*(A - B) + 3*L*L)*r*r - 9*r*r*r*r))
                com1 = abs(yb/((A-B)*ydiv))
                com2 = ya/ydiv
                z = 0.5*(A+B - 3*r*(com2/(A-B) + com1))
                y = com2 + (A-B)*com1*/
                auto ydiv = 2*(4*A*A - 8*A*B + 4*B*B + 9*r()*r());
                auto ya = 2*(A-B)*(A-B)*r();
                auto yb = 4*sqrt((A - B)*(A - B)*(-(A - B)*(A - B)*(A - B)*(A - B) + 4*(A - B)*(A - B)*L()*L() + 3*(-2*(A - B)*(A - B) + 3*L()*L())*r()*r() - 9*r()*r()*r()*r()));
                auto com1 = fabs(yb/((A-B)*ydiv));
                auto com2 = ya/ydiv;
                z = 0.5*(A+B - 3*r()*(com2/(A-B) + com1));
                y = com2 + (A-B)*com1;
                x = 0;
            } else {
                LOGV("LinearDeltaCoordMap::B!=C\n");
                auto za = (B - C)*r()*(2*A*A*A - A*A*(B + C) - A*(B*B + C*C - 3*r()*r()) + (B + C)*(2*B*B - 3*B*C + 2*C*C + 3*r()*r()));
                auto zb = sqrt(3)*sqrt(-((B - C)*(B - C)*r()*r()*((A - B)*(A - B)*(A - C)*(A - C)*(B - C)*(B - C) + 3*(A*A + B*B - B*C + C*C - A*(B + C))*(A*A + B*B - B*C + C*C - A*(B + C) - 4*L()*L())*r()*r() + 9*(2*(A*A + B*B - B*C + C*C - A*(B + C)) - 3*L()*L())*r()*r()*r()*r() + 27*r()*r()*r()*r()*r()*r())));
                auto zdiv = (B - C)*r()*(4*(A*A + B*B - B*C + C*C - A*(B + C)) + 9*r()*r());

                //will use smaller of z.
                //if sign(zb) == sign(zdiv), this should be z2, else z1.
                //therefore z = za/zdiv - abs(zb/zdiv)
                z = za/zdiv - fabs(zb/zdiv);
                //Solving for x, y in terms of z gives 
                x = ((B - C)*(B + C - 2*z))/(2*sqrt(3)*r());
                y = -((-2*A*A + B*B + C*C + 4*A*z - 2*B*z - 2*C*z)/(6*r()));
            }
            return std::make_tuple(x, y, z, e);
        }

};

}

#endif
