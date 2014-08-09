#ifndef MOTION_ACCELERATIONPROFILE_H
#define MOTION_ACCELERATIONPROFILE_H


struct AccelerationProfile {
	void begin(float /*moveDuration*/, float /*Vmax*/) {}
	//float transform(float inp, float moveDuration, float Vmax);
};

struct NoAcceleration : public AccelerationProfile {
	float transform(float inp) { return inp; }
};


#endif
