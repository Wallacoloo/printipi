#ifndef MOTION_ACCELERATIONPROFILE_H
#define MOTION_ACCELERATIONPROFILE_H

/* 
 * Printipi/motion/accelerationprofile.h
 * (c) 2014 Colin Wallace
 *
 * An AccelerationProfile takes event times and transforms them into a refined time based upon the acceleration mode.
 * As an example, a movement at 30 mm/sec with duration=0.6 sec might have Events with times like this:
 * 0.1, 0.2, 0.3, 0.4, 0.5, 0.6
 * and transform them to something like:
 * 0.2, 0.35, 0.5, 0.6, 0.75, 0.9
 * Currently, the initial and final velocity should both be 0 mm/sec.
 * Note that the events are already encoded at a *constant* velocity of Vmax (mm/sec) when they are passed through the AccelerationProfile. The AccelerationProfile should re-encode them so that the accelerate up to Vmax and then back to 0, and the velocity NEVER EXCEEDS Vmax.
 *
 * Note: AccelerationProfile is an interface and all derivatives must implement the methods outlined in the AccelerationProfile class. NoAcceleration can be considered a default implementation of this interface.
 */


struct AccelerationProfile {
	inline void begin(float /*moveDuration*/, float /*Vmax*/) {} //Optional, but almost surely needed.
	//float transform(float inp, float moveDuration, float Vmax);
};

struct NoAcceleration : public AccelerationProfile {
	float transform(float inp) { return inp; }
};


#endif
