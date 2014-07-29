#ifndef DRIVERS_LINEARCOORDMAP_H
#define DRIVERS_LINEARCOORDMAP_H

#include "coordmap.h"

namespace drv {

template <std::size_t xIdx=0, std::size_t yIdx=1, std::size_t zIdx=2, std::size_t eIdx=3> class LinearCoordMap : public CoordMap {
	public:
		static constexpr std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
		static void xyzeFromMechanical(const std::array<int, 4> &mech) {
			return std::make_tuple(mech[xIdx], mech[yIdx], mech[zIdx], mech[eIdx]);
		}
		static constexpr std::array<int, 4> getHomePosition() {
			return std::array<int, 4>({0, 0, 0, 0});
		}

};

}

#endif
