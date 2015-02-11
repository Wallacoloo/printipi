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

#ifndef COMMON_MATRIX_H
#define COMMON_MATRIX_H

#include "common/vector3.h"


/*
 * Matrices are useful in applying linear transformations. 
 * Notably, they can be used to adjust coordinates to a different coordinate-space in order to account for an unlevel bed.
 */
class Matrix3x3 {
    //r0, r1, r2 each represent one row of the matrix.
    Vector3f r0, r1, r2;
    public:
        Matrix3x3() : r0(), r1(), r2() {}
        Matrix3x3(float a00, float a01, float a02,
                float a10, float a11, float a12,
                float a20, float a21, float a22) :
            r0(a00, a01, a02),
            r1(a10, a11, a12),
            r2(a20, a21, a22) {}
        template <typename VecT> VecT transform(const VecT &xyz) const {
            return VecT(r0.dot(xyz), r1.dot(xyz), r2.dot(xyz));
        }

        static inline Matrix3x3 identity() {
            return Matrix3x3(1, 0, 0,
                             0, 1, 0,
                             0, 0, 1);
        }

        static inline Matrix3x3 rotationAboutPositiveZ(float angleRad) {
            float cosa = cos(angleRad);
            float sina = sin(angleRad);
            return Matrix3x3(cosa, -sina, 0,
                             sina,  cosa, 0,
                             0,        0, 1);
        }

};

#endif
