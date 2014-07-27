#ifndef DRIVERS_LINEARCOORDMAP_H
#define DRIVERS_LINEARCOORDMAP_H

#include "coordmap.h"

namespace drv {

template <std::size_t xIdx, std::size_t yIdx, std::size_t zIdx, std::size_t eIdx> class LinearCoordMap : public CoordMap {
	public:
		template <std::size_t size> static void xyzeFromMechanical(const std::array<int, size> &mech) {
			return std::make_tuple(mech[xIdx], mech[yIdx], mech[zIdx], mech[eIdx]);
		}

};

}

#endif
