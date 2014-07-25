#ifndef DRIVERS_IODRIVER_H
#define DRIVERS_IODRIVER_H

#include "typesettings.h"
#include "scheduler.h"

namespace drv {

/*template<typename T> struct IODriverInfo {
    template<typename U, size_t (U::*)() const> struct SFINAE {};
    template<typename U> static char Test(SFINAE<U, &U::deactivate>*);
    template<typename U> static int Test(...);
    static constexpr bool HasDeactivateMethod = sizeof(Test<T>(0)) == sizeof(char);
};*/

class IODriver {
	public:
		template <typename ThisT> IODriver(const ThisT * /* _this */) {
			//TODO: only register an exit handler if ThisT::deactivate is non-empty
			//can do this with C++ "SFINAE"
			//if (IODriverInfo<ThisT>::HasDeactivateMethod) {
				//Scheduler::registerExitHandler((void(*)())&ThisT::deactivate);
			//}
			Scheduler::registerExitHandler((void(*)())&ThisT::deactivate, SCHED_IO_EXIT_LEVEL);
		}
		//for a (stepper) motor, advance +/- 1 step:
		inline void stepForward() {} //OVERRIDE THIS
		inline void stepBackward() {} //OVERRIDE THIS
		/*deactivate: called at program exit.
		safely deactivate any IOs, including motors, heaters, etc.*/
		inline void deactivate() {} //OVERRIDE THIS
		/* called when the scheduler has extra time,
		Can be used to check the status of inputs, etc */
		inline void onIdleCpu() {} //OVERRIDE THIS
		//selectAndStep...: used internally
		template <typename TupleT> static void selectAndStepForward(TupleT &drivers, AxisIdType axis);
		template <typename TupleT> static void selectAndStepBackward(TupleT &drivers, AxisIdType axis);
		template <typename TupleT> static void callIdleCpuHandlers(TupleT &drivers);
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

//IODriver::callIdleCpuHandlers helper functions:

template <typename TupleT, std::size_t myIdx> struct IODriver__onIdleCpu {
	void operator()(TupleT &drivers) {
		IODriver__onIdleCpu<TupleT, myIdx-1>()(drivers);
		std::get<myIdx>(drivers).onIdleCpu();
	}
};

template <typename TupleT> struct IODriver__onIdleCpu<TupleT, 0> {
	void operator()(TupleT &drivers) {
		std::get<0>(drivers).onIdleCpu();
	}
};

template <typename TupleT> void IODriver::callIdleCpuHandlers(TupleT &drivers) {
	IODriver__onIdleCpu<TupleT, std::tuple_size<TupleT>::value-1>()(drivers);
}

}

#endif
