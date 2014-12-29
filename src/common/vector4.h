#ifndef COMMON_VECTOR4_H
#define COMMON_VECTOR4_H

#include "vector3.h"

//4-Vector composed of an (x, y, z) point in cartesian space plus an e (Extruded-length) component
template <typename F> class Vector4 {
	//cartesian components
	Vector3<F> _xyz;
	//extruder location
	F _e;
	public:
		//initialize to all 0's
		Vector4() : _xyz(), _e(0) {}
		//initialize from components
		Vector4(F x, F y, F z, F e) : _xyz(x, y, z), _e(e) {}
		//initialize from a cartesian (x, y, z) point plus an extruder coordinate
		//
		//allow for initialization from a different precision (eg a Vector3<double>)
		template <typename T2> Vector4(const Vector3<T2> &xyz, T2 e) : _xyz(xyz.x(), xyz.y(), xyz.z()), _e(e) {}
		//initialize from another Vector4, possibly of a different precision
		template <typename T2> Vector4(const Vector4<T2> &v) : _xyz(v.x(), v.y(), v.z()), _e(v.e()) {}

		//cast to a tuple of <x, y, z, e>
		std::tuple<F, F, F, F> tuple() const {
			return std::make_tuple(x(), y(), z(), e());
		}
		//cast to a tuple of <x, y, z, e>
		operator std::tuple<F, F, F, F>() const {
			return tuple();
		}

		//string representation: "Vector4f(x, y, z, e)"
		std::string str() const {
			return "Vector4(" + std::to_string(x()) + ", " + std::to_string(y()) + ", " + std::to_string(z()) + ", " + std::to_string(e()) + ")";
		}
		//string representation: "Vector4f(x, y, z, e)"
		operator std::string() const {
			return str();
		}

		//return the x, y, z components as a <Vector3>
		const Vector3<F>& xyz() const {
			return _xyz;
		}
		//return the e (extruder) component
		const F e() const {
			return _e;
		}
		//return the x component
		const F x() const {
			return _xyz.x();
		}
		//return the y component
		const F y() const {
			return _xyz.y();
		}
		//return the z component
		const F z() const {
			return _xyz.z();
		}

		//unary negation operator (x = -y)
		Vector4<F> operator-() const {
			return *this * -1;
		}

		//operators:
		Vector4<F> operator+(const Vector4<F> &v) const {
			return Vector4<F>(x() + v.x(), y() + v.y(), z() + v.z(), e() + v.e());
		}
		Vector4<F>& operator+=(const Vector4<F> &v) {
			return *this = (*this + v);
		}

		Vector4<F> operator-(const Vector4<F> &v) const {
			return *this + (-v);
		}
		Vector4<F>& operator-=(const Vector4<F> &v) {
			return *this = (*this - v);
		}

		Vector4<F> operator*(F s) const {
			return Vector4<F>(s*x(), s*y(), s*z(), s*e());
		}
		Vector4<F>& operator*=(F s) {
			return *this = (*this * s);
		}

		Vector4<F> operator/(F s) const {
			return *this * ((F)1. / s);
		}
		Vector4<F>& operator/=(F s) {
			return *this = (*this / s);
		}
		
};

//4-component (x, y, z) vector using (32-bit) floats
typedef Vector4<float> Vector4f;
//4-component (x, y, z) vector using higher precision (64-bit) doubles
typedef Vector4<double> Vector4d;

#endif