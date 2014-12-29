#ifndef COMMON_MATHUTIL_H
#define COMMON_MATHUTIL_H

/* 
 * Printipi/common/mathutil.h
 *
 * This file provides some useful conversion (mm to inches, celcius to kelvin, etc), mathematical constants, and basic mathematical functions for convenience.
 */


namespace mathutil {

//By definition, there are EXACTLY 25.4 mm per inch.
const float MM_PER_IN = 25.4;
//It is impossible for any temperature to be < this value, in Celcius
const float ABSOLUTE_ZERO_C = -273.15;

//Convert from Celcius temperature to Kelvin
template <typename T> constexpr T CtoK(T C) {
    return C + 273.15;
}

//Convert from Kelvin temperature to Celcius
template <typename T> constexpr T KtoC(T K) {
    return K - 273.15;
}


}

#endif
