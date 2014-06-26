#ifndef GPARSE_EVENT_H
#define GPARSE_EVENT_H

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

namespace gparse {

/*enum EventType {
	Evt_StepForward,
	Evt_StepBackward
};*/

enum StepDirection {
	StepForward,
	StepBackward
};

class Event {
	struct timespec _time;
	char _stepperNum;
	bool _isForward;
	public:
		int stepperNumber() const;
		StepDirection direction() const;
		const struct timespec& time() const;
		
};


}
#endif
