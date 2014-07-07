#ifndef LINEARSTEPPER_H
#define LINEARSTEPPER_H

#include "axisstepper.h"
#include "typesettings.h"
#include "logging.h"
#include <tuple>

namespace drv {

template <int STEPS_PER_METER, CoordAxis CoordType> class LinearStepper : public AxisStepper {
	private:
		float timePerStep;
	public:
		static constexpr float STEPS_MM() { return STEPS_PER_METER/1000.0; }
		static constexpr float GET_COORD(float x, float y, float z, float e) {
			return CoordType==COORD_X ? x : \
				  (CoordType==COORD_Y ? y : \
				  (CoordType==COORD_Z ? z : 
				  (CoordType==COORD_E ? e : 0) ) );
		}
		LinearStepper() {}
		template <std::size_t sz> LinearStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
			: AxisStepper(idx, curPos, vx, vy, vz, ve),
			timePerStep(abs( GET_COORD(vx, vy, vz, ve) / STEPS_MM() )) {
				this->direction = stepDirFromSign( GET_COORD(vx, vy, vz, ve) / STEPS_MM() );
			}
		void _nextStep() {
			this->time += timePerStep;
			LOG("LinearStepper::_nextStep() %i, %f\n", CoordType, timePerStep);
		}
};

/*template <int STEPS_PER_METER, AxisIdType AXIS_ID> class LinearStepper : public AxisStepper {
	private:
		float timePerStep;
	public:
		static constexpr float STEPS_MM() { return STEPS_PER_METER/1000.0; }
		LinearStepper() {}
		template <std::size_t sz> LinearStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
			: AxisStepper(idx, curPos, vx, vy, vz, ve),
			timePerStep(abs( std::get<AXIS_ID>(std::make_tuple(vx, vy, vz, ve)) / STEPS_MM() )) {
				this->direction = stepDirFromSign( std::get<AXIS_ID>(std::make_tuple(vx, vy, vz, ve)) / STEPS_MM() );
			}
		void _nextStep() {
			this->time += timePerStep;
		}
};*/

}

#endif
