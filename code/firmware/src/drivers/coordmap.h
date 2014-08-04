#ifndef DRIVERS_COORDMAP_H
#define DRIVERS_COORDMAP_H

/* 
 * Printipi/drivers/coordmap.h
 * (c) 2014 Colin Wallace
 *
 * CoordMaps are used to translate cartesian coordinates to and from the machine's coordinate system
 * This allows for a bot to internally use a non-cartesian coordinate system - very useful for delta bots.
 *
 * Note: CoordMap is an interface, and not an implementation.
 * An implementation is needed for each coordinate style - Cartesian, deltabot, etc.
 * These implementations must provide the functions outlined further down in the header.
 */

#include <tuple> //needed for children

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
