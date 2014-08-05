#ifndef ACCELERATIONPROFILE_H
#define ACCELERATIONPROFILE_H


struct AccelerationProfile {
	//float transform(float inp);
};

class NoAcceleration : public AccelerationProfile {
	public:
		float transform(float inp) { return inp; }
};


#endif
