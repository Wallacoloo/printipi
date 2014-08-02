#ifndef DRIVERS_IODRIVER_H
#define DRIVERS_IODRIVER_H

/* 
 * Printipi/drivers/iodriver.h
 * (c) 2014 Colin Wallace
 *
 * IODrivers control the electrical interface to each component.
 * One IODriver is needed for each stepper motor, fan, hotend, etc.
 * Note that the stepForward and stepBackward methods may have different meanings for non-stepper motors.
 *   for fans or hotends, this would mean turn on, or turn off
 *
 * Note: IODriver is an interface, and not an implementation.
 * An implementation is needed for each electrical component - the fan, hotend, and 1 for each stepper motor, etc.
 * These implementations must provide the functions outlined further down in the header.
 */

#include "common/typesettings.h"
#include "common/tupleutil.h"
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
			SchedulerBase::registerExitHandler((void(*)())&ThisT::deactivate, SCHED_IO_EXIT_LEVEL);
		}
		//for a (stepper) motor, advance +/- 1 step:
		inline void stepForward() {} //OVERRIDE THIS
		inline void stepBackward() {} //OVERRIDE THIS
		/*deactivate: called at program exit.
		safely deactivate any IOs, including motors, heaters, etc.*/
		inline void deactivate() {} //OVERRIDE THIS
		/* called by M17; Enable/power all stepper motors */
		inline void lockAxis() {} //OVERRIDE THIS (stepper motor drivers only)
		/* called by M18; Disable all stepper motors. Intention is to let them move 'freely', eg, for manual adjustment or to disable idle noise. */
		inline void unlockAxis() {} //OVERRIDE THIS (stepper motor drivers only)
		inline bool isFan() const { return false; } //OVERRIDE THIS (fans only: return true)
		inline float fanPwmPeriod() const { return 0.1; }
		/* called when the scheduler has extra time,
		Can be used to check the status of inputs, etc.
		Return true if object needs to continue to be serviced, false otherwise. */
		template <typename Sched> inline bool onIdleCpu(Sched & /*sched*/) { return false; } //OVERRIDE THIS
		//selectAndStep...: used internally
		template <typename TupleT> static void selectAndStepForward(TupleT &drivers, AxisIdType axis);
		template <typename TupleT> static void selectAndStepBackward(TupleT &drivers, AxisIdType axis);
		template <typename TupleT, typename ...Args > static bool callIdleCpuHandlers(TupleT &drivers, Args... args);
		template <typename TupleT> static void lockAllAxis(TupleT &drivers);
		template <typename TupleT> static void unlockAllAxis(TupleT &drivers);
};

//IODriver::selectAndStepForward helper functions:
struct IODriver__stepForward {
	template <typename T> operator()(std::size_t index, T &driver, AxisIdType desiredIndex) {
		if (index == desiredIndex) {
			driver.stepForward();
		}
	}
};
template <typename TupleT> void IODriver::selectAndStepForward(TupleT &drivers, AxisIdType axis) {
	callOnAll(drivers, IODriver__stepForward(), axis);
}


//IODriver::selectAndStepBackward helper functions:
struct IODriver__stepBackward {
	template <typename T> operator()(std::size_t index, T &driver, AxisIdType desiredIndex) {
		if (index == desiredIndex) {
			driver.stepBackward();
		}
	}
};
template <typename TupleT> void IODriver::selectAndStepBackward(TupleT &drivers, AxisIdType axis) {
	callOnAll(drivers, IODriver__stepBackward(), axis);
}

//IODriver::callIdleCpuHandlers helper functions:

template <typename TupleT, std::size_t myIdx, typename ...Args> struct IODriver__onIdleCpu {
	bool operator()(TupleT &drivers, Args... args) {
		bool prev = IODriver__onIdleCpu<TupleT, myIdx-1, Args...>()(drivers, args...);
		bool cur = std::get<myIdx>(drivers).onIdleCpu(args...); //EXPLICITLY CALCULATE THIS SEPARATELY TO PREVENT SHORT-CIRCUIT OPERATIONS
		return prev || cur; //return true if ANY objects need future servicing.
	}
};

template <typename TupleT, typename ...Args> struct IODriver__onIdleCpu<TupleT, 0, Args...> {
	bool operator()(TupleT &drivers, Args... args) {
		return std::get<0>(drivers).onIdleCpu(args...);
	}
};

template <typename TupleT, typename ...Args> bool IODriver::callIdleCpuHandlers(TupleT &drivers, Args... args) {
	return IODriver__onIdleCpu<TupleT, std::tuple_size<TupleT>::value-1, Args...>()(drivers, args...);
}

//IODriver::lockAllAxis helper functions:
struct IODriver__lockAllAxis {
	template <typename T> operator()(std::size_t index, T &driver) {
		driver.lockAxis();
	}
};
template <typename TupleT> void IODriver::lockAllAxis(TupleT &drivers) {
	callOnAll(drivers, IODriver__lockAllAxis());
}

//IODriver::unlockAllAxis helper functions:
struct IODriver__unlockAllAxis {
	template <typename T> operator()(std::size_t index, T &driver) {
		driver.unlockAxis();
	}
};
template <typename TupleT> void IODriver::unlockAllAxis(TupleT &drivers) {
	callOnAll(drivers, IODriver__unlockAllAxis());
}

}

#endif
