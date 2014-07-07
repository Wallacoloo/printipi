#ifndef DRIVERS_AXISSTEPPER_H
#define DRIVERS_AXISSTEPPER_H

#include "../event.h"
#include <tuple>
#include <array>

namespace drv {

class AxisStepper {
	public:
		float time; //time of next step
		StepDirection direction; //direction of next step
		int index; //ID of stepper.
		AxisStepper() {}
		template <std::size_t sz> AxisStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
			: index(idx) {}
		template <typename TupleT> static AxisStepper& getNextTime(TupleT &axes);
		template <typename TupleT> static void initAxisSteppers(TupleT &steppers, const std::array<int, std::tuple_size<TupleT>::value>& curPos, float vx, float vy, float vz, float ve);
		Event getEvent() const; //NOT TO BE OVERRIDEN
		
};

//Helper classes for AxisStepper::getNextTime method
//C++ doesn't support partial template function specialization, so we need to use templated classes instead.
//Below function(s) select the AxisStepper with the smallest time attribute from a tuple of such AxisSteppers
template <typename TupleT, int idx> struct _AxisStepper__getNextTime {
	AxisStepper& operator()(TupleT &axes) {
		AxisStepper &m1 = _AxisStepper__getNextTime<TupleT, idx-1>()(axes);
		AxisStepper &m2 = std::get<idx>(axes);
		return (m1.time < m2.time) ? m1 : m2;
	}
};

template <typename TupleT> struct _AxisStepper__getNextTime<TupleT, 0> {
	AxisStepper& operator()(TupleT &axes) {
		return std::get<0>(axes);
		
	}
};

template <typename TupleT> AxisStepper& AxisStepper::getNextTime(TupleT &axes) {
	return _AxisStepper__getNextTime<TupleT, std::tuple_size<TupleT>::value-1>()(axes);
}

//Helper classes for AxisStepper::initAxisSteppers

template <typename TupleT, int idx> struct _AxisStepper__initAxisSteppers {
	void operator()(TupleT &steppers, const std::array<int, std::tuple_size<TupleT>::value>& curPos, float vx, float vy, float vz, float ve) {
		_AxisStepper__initAxisSteppers<TupleT, idx-1>()(steppers, curPos, vx, vy, vz, ve); //initialize all previous values.
		std::get<idx>(steppers) = typename std::tuple_element<idx, TupleT>::type(idx, curPos, vx, vy, vz, ve);
	}
};

template <typename TupleT> struct _AxisStepper__initAxisSteppers<TupleT, 0> {
	void operator()(TupleT &steppers, const std::array<int, std::tuple_size<TupleT>::value>& curPos, float vx, float vy, float vz, float ve) {
		std::get<0>(steppers) = typename std::tuple_element<0, TupleT>::type(0, curPos, vx, vy, vz, ve);
	}
};

template <typename TupleT> void AxisStepper::initAxisSteppers(TupleT &steppers, const std::array<int, std::tuple_size<TupleT>::value>& curPos, float vx, float vy, float vz, float ve) {
	_AxisStepper__initAxisSteppers<TupleT, std::tuple_size<TupleT>::value-1>()(steppers, curPos, vx, vy, vz, ve);
}


}

#endif
