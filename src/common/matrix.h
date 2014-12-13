#ifndef COMMON_MATRIX_H
#define COMMON_MATRIX_H

/* 
 * Printipi/common/matrix.h
 * (c) 2014 Colin Wallace
 *
 * This file exposes classes and templates used to construct 3x3 matrices.
 * Matrices are useful in applying linear transformations. Notably, they can be used to adjust coordinates to a different coordinate-space in order to account for an unlevel bed.
 */


/*
 * In general, it is difficult to apply static operations to matrixes (add, mul, etc at compile-time).
 * There is this library for templated floats: http://www.edwardrosten.com/code/fp_template.html
 *   But it claims a mere 8 FLOPS during compilation (opteron 2.6)
 * So it's probably best to do everything at runtime. A smart compiler may still be able to optimize it.
 * Plus, with auto bed-leveling, that's all done at runtime anyway.
 */

#include <tuple>

class Matrix3x3 {
    float a00, a01, a02;
    float a10, a11, a12;
    float a20, a21, a22;
    public:
        Matrix3x3() : a00(0), a01(0), a02(0), a10(0), a11(0), a12(0), a20(0), a21(0), a22(0) {}
        Matrix3x3(float a00, float a01, float a02,
                float a10, float a11, float a12,
                float a20, float a21, float a22) :
            a00(a00), a01(a01), a02(a02),
            a10(a10), a11(a11), a12(a12),
            a20(a20), a21(a21), a22(a22) {}
        inline std::tuple<float, float, float> transform(const std::tuple<float, float, float> &xyz) const {
            return std::tuple<float, float, float>(
                  a00*std::get<0>(xyz) + a01*std::get<1>(xyz) + a02*std::get<2>(xyz),
                  a10*std::get<0>(xyz) + a11*std::get<1>(xyz) + a12*std::get<2>(xyz),
                  a20*std::get<0>(xyz) + a21*std::get<1>(xyz) + a22*std::get<2>(xyz));
        }

};

#endif
