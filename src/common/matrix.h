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
			//float x, y, z;
			//std::tie(x, y, z) = xyz;
			/*return std::tuple<float, float, float>(A00()*x + A01()*y + A02()*z,
			                          A10()*x + A11()*y + A12()*z,
			                          A20()*x + A21()*y + A22()*z);*/
              return std::tuple<float, float, float>(
                  A00()*std::get<0>(xyz) + A01()*std::get<1>(xyz) + A02()*std::get<2>(xyz),
			      A10()*std::get<0>(xyz) + A11()*std::get<1>(xyz) + A12()*std::get<2>(xyz),
			      A20()*std::get<0>(xyz) + A21()*std::get<1>(xyz) + A22()*std::get<2>(xyz));
		}
};

typedef Matrix3Static<1, 0, 0,   0, 1, 0,   0, 0, 1> Identity3Static;


/*struct Matrix3 {
	std::array<std::array<float, 3>, 3> coeffs; //index as [row][col]
	
};*/

}
#endif
