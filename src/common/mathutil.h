/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef COMMON_MATHUTIL_H
#define COMMON_MATHUTIL_H

/* 
 * This  provides some useful conversion (mm to inches, celcius to kelvin, etc), mathematical constants, and basic mathematical functions for convenience.
 */
namespace mathutil {

//By definition, there are EXACTLY 25.4 mm per inch.
const float MM_PER_IN = 25.4;
//It is impossible for any temperature to be < this value, in Celcius
const float ABSOLUTE_ZERO_CELCIUS = -273.15;

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
