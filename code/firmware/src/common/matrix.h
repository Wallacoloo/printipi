#ifndef COMMON_MATRIX_H
#define COMMON_MATRIX_H

#include <tuple>

namespace matr {


template <int A00_1000, int A01_1000, int A02_1000,
          int A10_1000, int A11_1000, int A12_1000,
          int A20_1000, int A21_1000, int A22_1000> class Matrix3 {
    static constexpr float A00() { return A00_1000 / 1000.; }
    static constexpr float A01() { return A01_1000 / 1000.; }
    static constexpr float A02() { return A02_1000 / 1000.; }
    static constexpr float A10() { return A10_1000 / 1000.; }
    static constexpr float A11() { return A11_1000 / 1000.; }
    static constexpr float A12() { return A12_1000 / 1000.; }
    static constexpr float A20() { return A20_1000 / 1000.; }
    static constexpr float A21() { return A21_1000 / 1000.; }
    static constexpr float A22() { return A22_1000 / 1000.; }
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

typedef Matrix3<1000, 0, 0,   0, 1000, 0,   0, 0, 1000> Identity3;


}
#endif
