/* This class allows for translating mechanical positions to the cartesian x, y, z, e system.
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
 * The math is described more in /code/proof-of-concept/coordmath.py and coord-math.nb
 */

#ifndef DRIVERS_LINEARDELTACOORDMAP_H
#define DRIVERS_LINEARDELTACOORDMAP_H

#include "coordmap.h"
#include "logging.h"

namespace drv {

template <std::size_t AIdx, std::size_t BIdx, std::size_t CIdx, std::size_t EIdx, unsigned R1000, unsigned L1000, unsigned H1000, unsigned STEPS_M> class LinearDeltaCoordMap : public CoordMap {
	static constexpr float r = R1000 / 1000.;
	static constexpr float L = L1000 / 1000.;
	static constexpr float h = H1000 / 1000.;
	static constexpr float STEPS_MM = STEPS_M / 1000.;
	static constexpr float MM_STEPS = 1. / STEPS_MM;
	public:
		template <std::size_t size> static void getHomePosition(std::array<int, size> &mech) {
			//z = A*MM_STEPS-sqrt(L*L-r*r);
			//A=(z+sqrt(L*L-r*r))*STEPS_MM
			mech[EIdx] = 0;
			mech[AIdx] = mech[BIdx] = mech[CIdx] = (h+sqrt(L*L-r*r))*STEPS_MM;
		}
		template <std::size_t size> static void xyzeFromMechanical(const std::array<int, size> &mech, float &x, float &y, float &z, float &e) {
			e = mech[EIdx];
			float A = mech[AIdx]*MM_STEPS; //convert mechanical positions (steps) to MM.
			float B = mech[BIdx]*MM_STEPS;
			float C = mech[CIdx]*MM_STEPS;
			if (A == B && B == C) { //prevent a division-by-zero.
				LOGV("LinearDeltaCoordMap::A==B==C\n");
				x = 0;
				y = 0;
				z = A-sqrt(L*L-r*r);
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
				auto ydiv = 2*(4*A*A - 8*A*B + 4*B*B + 9*r*r);
				auto ya = 2*(A-B)*(A-B)*r;
				auto yb = 4*sqrt((A - B)*(A - B)*(-(A - B)*(A - B)*(A - B)*(A - B) + 4*(A - B)*(A - B)*L*L + 3*(-2*(A - B)*(A - B) + 3*L*L)*r*r - 9*r*r*r*r));
				auto com1 = fabs(yb/((A-B)*ydiv));
				auto com2 = ya/ydiv;
				z = 0.5*(A+B - 3*r*(com2/(A-B) + com1));
				y = com2 + (A-B)*com1;
				x = 0;
			} else {
				LOGV("LinearDeltaCoordMap::B!=C\n");
				auto za = (B - C)*r*(2*A*A*A - A*A*(B + C) - A*(B*B + C*C - 3*r*r) + (B + C)*(2*B*B - 3*B*C + 2*C*C + 3*r*r));
				auto zb = sqrt(3)*sqrt(-((B - C)*(B - C)*r*r*((A - B)*(A - B)*(A - C)*(A - C)*(B - C)*(B - C) + 3*(A*A + B*B - B*C + C*C - A*(B + C))*(A*A + B*B - B*C + C*C - A*(B + C) - 4*L*L)*r*r + 9*(2*(A*A + B*B - B*C + C*C - A*(B + C)) - 3*L*L)*r*r*r*r + 27*r*r*r*r*r*r)));
				auto zdiv = (B - C)*r*(4*(A*A + B*B - B*C + C*C - A*(B + C)) + 9*r*r);

				//will use smaller of z.
				//if sign(zb) == sign(zdiv), this should be z2, else z1.
				//therefore z = za/zdiv - abs(zb/zdiv)
				z = za/zdiv - fabs(zb/zdiv);
				//Solving for x, y in terms of z gives 
				x = ((B - C)*(B + C - 2*z))/(2*sqrt(3)*r);
				y = -((-2*A*A + B*B + C*C + 4*A*z - 2*B*z - 2*C*z)/(6*r));
			}
		}

};

}

#endif
