#ifndef DRIVERS_IODRIVER_H
#define DRIVERS_IODRIVER_H

#include "typesettings.h"

namespace drv {

class IODriver {
	public:
		//for a (stepper) motor, advance +/- 1 step:
		inline void stepForward() {} //OVERRIDE THIS
		inline void stepBackward() {} //OVERRIDE THIS
		/*deactivate: called at program exit.
		safely deactivate any IOs, including motors, heaters, etc.*/
		inline void deactivate() {} //OVERRIDE THIS
		//selectAndStep...: used internally
		template <typename TupleT> static void selectAndStepForward(TupleT &drivers, AxisIdType axis);
		template <typename TupleT> static void selectAndStepBackward(TupleT &drivers, AxisIdType axis);
};

//IODriver::selectAndStepForward helper functions:

template <typename TupleT, std::size_t myIdx> struct IODriver__stepForward {
	void operator()(TupleT &drivers, AxisIdType desiredIdx) {
		IODriver__stepForward<TupleT, myIdx-1>()(drivers, desiredIdx);
		if (myIdx == desiredIdx) {
			std::get<myIdx>(drivers).stepForward();
		}
	}
};

template <typename TupleT> struct IODriver__stepForward<TupleT, 0> {
	void operator()(TupleT &drivers, AxisIdType desiredIdx) {
		if (0 == desiredIdx) {
			std::get<0>(drivers).stepForward();
		}
	}
};


template <typename TupleT> void IODriver::selectAndStepForward(TupleT &drivers, AxisIdType axis) {
	IODriver__stepForward<TupleT, std::tuple_size<TupleT>::value-1>()(drivers, axis);
}

//IODriver::selectAndStepBackward helper functions:

template <typename TupleT, std::size_t myIdx> struct IODriver__stepBackward {
	void operator()(TupleT &drivers, AxisIdType desiredIdx) {
		IODriver__stepBackward<TupleT, myIdx-1>()(drivers, desiredIdx);
		if (myIdx == desiredIdx) {
			std::get<myIdx>(drivers).stepBackward();
		}
	}
};

template <typename TupleT> struct IODriver__stepBackward<TupleT, 0> {
	void operator()(TupleT &drivers, AxisIdType desiredIdx) {
		if (0 == desiredIdx) {
			std::get<0>(drivers).stepBackward();
		}
	}
};


template <typename TupleT> void IODriver::selectAndStepBackward(TupleT &drivers, AxisIdType axis) {
	IODriver__stepBackward<TupleT, std::tuple_size<TupleT>::value-1>()(drivers, axis);
}

}

#endif
