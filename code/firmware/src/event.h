#ifndef EVENT_H
#define EVENT_H

//There will be these events:
//  step forward (motor 0, 1, 2, 3)
//  step backward (motor 0, 1, 2, 3)
//Optionally:
//  read nozzle/bed temperature.
//    This is a potentially long process (time cap discharge).
//    Still want to have scheduler manage it, but shouldn't have it be in the event queue.
//  wait for temperature to stabilize.
//    This could be integrated into the event processing.
//      No - there are some movements which don't require bed to be heated (eg probing).
//    Could be done unbuffered?
//      yes - M116 is unbuffered.

#include <time.h> //for timespec
#include "typesettings.h" //for AxisIdType

/*enum EventType {
	Evt_StepForward,
	Evt_StepBackward
};*/

class Event {
	struct timespec _time;
	AxisIdType _stepperNum;
	bool _isForward;
	public:
		AxisIdType stepperId() const;
		StepDirection direction() const;
		const struct timespec& time() const;
		Event() {}
		Event(const timespec &t, AxisIdType stepperNum, StepDirection dir);
		static Event StepperEvent(float relTime, AxisIdType stepperNum, StepDirection dir);
		
};

#endif
