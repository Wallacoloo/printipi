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

namespace matr {


template <int A00_, int A01_, int A02_,
          int A10_, int A11_, int A12_,
          int A20_, int A21_, int A22_, int Denom=1> class Matrix3Static {
    static constexpr float A00() { return float(A00_) / Denom; }
    static constexpr float A01() { return float(A01_) / Denom; }
    static constexpr float A02() { return float(A02_) / Denom; }
    static constexpr float A10() { return float(A10_) / Denom; }
    static constexpr float A11() { return float(A11_) / Denom; }
    static constexpr float A12() { return float(A12_) / Denom; }
    static constexpr float A20() { return float(A20_) / Denom; }
    static constexpr float A21() { return float(A21_) / Denom; }
    static constexpr float A22() { return float(A22_) / Denom; }
    public:
        static constexpr std::tuple<float, float, float> transform(const std::tuple<float, float, float> &xyz) {
              return std::tuple<float, float, float>(
                  A00()*std::get<0>(xyz) + A01()*std::get<1>(xyz) + A02()*std::get<2>(xyz),
                  A10()*std::get<0>(xyz) + A11()*std::get<1>(xyz) + A12()*std::get<2>(xyz),
                  A20()*std::get<0>(xyz) + A21()*std::get<1>(xyz) + A22()*std::get<2>(xyz));
        }
};

typedef Matrix3Static<1, 0, 0,   0, 1, 0,   0, 0, 1> Identity3Static;

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
        std::tuple<float, float, float> transform(const std::tuple<float, float, float> &xyz) const {
            return std::tuple<float, float, float>(
                  a00*std::get<0>(xyz) + a01*std::get<1>(xyz) + a02*std::get<2>(xyz),
                  a10*std::get<0>(xyz) + a11*std::get<1>(xyz) + a12*std::get<2>(xyz),
                  a20*std::get<0>(xyz) + a21*std::get<1>(xyz) + a22*std::get<2>(xyz));
        }

};

}
#endif
