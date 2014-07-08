#ifndef GMATH_H
#define GMATH_H

#include <limits>

namespace gmath {

const float MM_PER_IN = 25.4; //It appears that by definition, there are EXACTLY 25.4 mm per inch.
const float NANOSECOND = 1e-9;
const float EPSILON = std::numeric_limits<float>::epsilon(); //represents off-by-one LSB for float value=1.0

template <typename T1, typename T2> constexpr bool ltepsilon(T1 a, T2 b, float epsilon=EPSILON) {
	//return a < b + epsilon;
	return a - b < epsilon;
}

template <typename T1, typename T2> constexpr bool eqepsilon(T1 a, T2 b, float epsilon=EPSILON) {
	//return a < b + epsilon;
	return std::fabs(a - b) <= epsilon;
}

template <typename T1> T1 makeZeroIfClose(T1 a, float epsilon=NANOSECOND) {
	if (eqepsilon(a, 0, epsilon)) {
		return 0;
	} else {
		return a;
	}
}


}

#endif
