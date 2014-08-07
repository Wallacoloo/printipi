#ifndef MOTION_ACCELERATIONPROFILE_H
#define MOTION_ACCELERATIONPROFILE_H


struct AccelerationProfile {
	//float transform(float inp, float moveDuration, float Vmax);
};

class NoAcceleration : public AccelerationProfile {
	public:
		float transform(float inp, float /*moveDuration*/, float /*Vmax*/) { return inp; }
};


#endif
