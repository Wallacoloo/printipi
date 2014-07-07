#ifndef EXTRUDERSTEPPER_H
#define EXTRUDERSTEPPER_H

#include "axisstepper.h"

namespace drv {

template <int STEPS_PER_METER> class ExtruderStepper : public AxisStepper {
	private:
		float timePerStep;
	public:
		static const float STEPS_MM = STEPS_PER_METER/1000.0;
		ExtruderStepper() {}
		template <std::size_t sz> ExtruderStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
			: AxisStepper(idx, curPos, vx, vy, vz, ve),
			timePerStep(abs(ve/STEPS_MM)) {
				this->direction = stepDirFromSign(ve/STEPS_MM);
			}
		void _nextStep() {
			this->time += timePerStep;
		}
};

}

#endif
