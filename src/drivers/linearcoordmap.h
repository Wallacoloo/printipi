#ifndef DRIVERS_LINEARCOORDMAP_H
#define DRIVERS_LINEARCOORDMAP_H

/* 
 * Printipi/drivers/linearcoordmap.h
 * (c) 2014 Colin Wallace
 *
 * LinearCoordMap implements the CoordMap interface for Cartesian robots.
 * This is the CoordMap that your driver (src/drivers/driver.h implementation) should use if it is a Cartesian robot.
 */

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
