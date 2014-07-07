#ifndef DRIVERS_AXISSTEPPER_H
#define DRIVERS_AXISSTEPPER_H

#include "../event.h"
#include <tuple>

namespace drv {

class AxisStepper {
	public:
		float time;
		StepDirection direction;
		template <typename TupleT> static AxisStepper& getNextTime(TupleT &axes);
		
};

//Helper classes for AxisStepper::getNextTime method
//C++ doesn't support partial template function specialization, so we need to use templated classes instead.
//Below function(s) select the AxisStepper with the smallest time attribute from a tuple of such AxisSteppers
template <typename TupleT, int idx> class _AxisStepper__getNextTime {
	public:
		AxisStepper& operator()(TupleT &axes) {
			AxisStepper &m1 = _AxisStepper__getNextTime<TupleT, idx-1>()(axes);
			AxisStepper &m2 = std::get<idx>(axes);
			return (m1.time < m2.time) ? m1 : m2;
		}
};

template <typename TupleT> class _AxisStepper__getNextTime<TupleT, 0> {
	public:
		AxisStepper& operator()(TupleT &axes) {
			return std::get<0>(axes);
			
		}
};

template <typename TupleT> AxisStepper& AxisStepper::getNextTime(TupleT &axes) {
	return _AxisStepper__getNextTime<TupleT, std::tuple_size<TupleT>::value-1>()(axes);
}


}

#endif
