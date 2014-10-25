#ifndef COMMON_MATHUTIL_H
#define COMMON_MATHUTIL_H

/* 
 * Printipi/common/mathutil.h
 *
 * This file provides some useful conversion (mm to inches, celcius to kelvin, etc), mathematical constants, and basic mathematical functions for convenience.
 */


namespace mathutil {

const float MM_PER_IN = 25.4; //It appears that by definition, there are EXACTLY 25.4 mm per inch.

template <typename T> constexpr T CtoK(T C) {
    return C + 273.15;
}
template <typename T> constexpr T KtoC(T K) {
    return K - 273.15;
}


}

#endif
