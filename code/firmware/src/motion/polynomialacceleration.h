#ifndef MOTION_POLYNOMIAL_ACCELERATION_H
#define MOTION_POLYNOMIAL_ACCELERATION_H

/* Canceled.
  Implementation is non-trivial; requires solving the roots of a polynomial of degree N+1.
  Therefore, only constant-acceleration is trivial.
*/

#include "accelerationprofile.h"

template <int Coeffs...> class PolynomialAcceleration : public AccelerationProfile {
	public:
		float transform(float inp) {
			return inp;
		}
};

#endif
