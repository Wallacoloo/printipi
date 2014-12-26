#ifndef COMMON_VECTOR4_H
#define COMMON_VECTOR4_H

#include "vector3.h"

template <typename T> class Vector4 {
	//cartesian (x, y, z) point
	Vector3<T> _xyz;
	//extruder location
	T _e;
	public:
		//initialize to all 0's
		Vector4() : _xyz(), _e(0) {}
		//initialize from components
		Vector4(T x, T y, T z, T e) : _xyz(x, y, z), _e(e) {}
		//initialize from a cartesian (x, y, z) point plus an extruder coordinate
		//allow for initialization from a different precision (eg a Vector3<double>)
		template <typename T2> Vector4(const Vector3<T2> &xyz, T2 e) : _xyz(xyz.x(), xyz.y(), xyz.z()), _e(e) {}
		//initialize from another Vector4, possibly of a different precision
		template <typename T2> Vector4(const Vector4<T2> &v) : _xyz(v.x(), v.y(), v.z()), _e(v.e()) {}
		//return the x, y, z components as a <Vector3>
		const Vector3<T>& xyz() const {
			return _xyz;
		}
		//return the e (extruder) component
		const T e() const {
			return _xyz.e();
		}
		//return the x component
		const T x() const {
			return _xyz.x();
		}
		//return the y component
		const T y() const {
			return _xyz.y();
		}
		//return the z component
		const T z() const {
			return _xyz.z();
		}
		
};

#endif