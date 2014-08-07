#ifndef EVENT_H
#define EVENT_H

/* 
 * Printipi/event.h
 * (c) 2014 Colin Wallace
 *
 * An Event encapsulates information about something that needs to happen at a specific time.
 *   Eg, "advance stepper #2 at time T", or "set hotend #1 high at time T"
 * Events are queued in the Scheduler and then handled by the State at their appropriate time.
 */

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

//#include <time.h> //for timespec
#include "common/typesettings.h" //for AxisIdType

/*enum EventType {
	Evt_StepForward,
	Evt_StepBackward
};*/

class Event {
	EventClockT::time_point _time;
	AxisIdType _stepperNum;
	bool _isForward;
	public:
		static const AxisIdType NULL_STEPPER_ID = 255;
		AxisIdType stepperId() const;
		StepDirection direction() const;
		EventClockT::time_point time() const;
		//bool isTime() const;
		bool isNull() const;
		Event() : _time(), _stepperNum(NULL_STEPPER_ID) {}
		Event(EventClockT::time_point t, AxisIdType stepperNum, StepDirection dir);
		static Event StepperEvent(float relTime, AxisIdType stepperNum, StepDirection dir);
		
		void offset(const EventClockT::duration &offset);
		void offsetNano(unsigned nsec); //must be less than 1 second.
		bool operator<(const Event &other) const;
		bool operator>(const Event &other) const;
		
};

#endif
