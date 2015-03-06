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

#ifndef COMMON_VECTOR_H
#define COMMON_VECTOR_H

#include <cmath>
#include <string>
#include <tuple>
#include <array>

//mathematical vector utility
template <typename F> class Vector3 {
	F _x, _y, _z;
	public:
		//default initialize: all components are zeroed
		Vector3() : _x(0), _y(0), _z(0) {}
		//initialize from components
		Vector3(F x, F y, F z) : _x(x), _y(y), _z(z) {}
		//initialize from another Vector3, possibly of a different precision
		template <typename T2> Vector3(const Vector3<T2> &v)
		  : _x(v.x()), _y(v.y()), _z(v.z()) {}

		//cast to a tuple of <x, y, z>
		std::tuple<F, F, F> tuple() const {
			return std::make_tuple(x(), y(), z());
		}
		//cast to a tuple of <x, y, z>
		operator std::tuple<F, F, F>() const {
			return tuple();
		}
		//cast to a std::array of <x, y, z>
		std::array<F, 3> array() const {
			return std::array<F, 3>({{x(), y(), z()}});
		}
		//cast to a std::array of <x, y, z>
		operator std::array<F, 3>() const {
			return array();
		}
		

		//string representation: "Vector3(x, y, z)"
		std::string str() const {
			return "Vector3(" + std::to_string(x()) + ", " + std::to_string(y()) + ", " + std::to_string(z()) + ")";
		}
		//string representation: "Vector3(x, y, z)"
		operator std::string() const {
			return str();
		}
		//@return x component
		F x() const { return _x; }
		//@return y component
		F y() const { return _y; }
		//@return z component
		F z() const { return _z; }
		//@return the square of the magnitude (length) of the vector.
		//equivalent to this->mag() * this->mag(), but less verbose and explicitly avoids the sqrt operation.
		F magSq() const { 
			return this->dot(*this); 
		}
		//@return magnitude (length) of the vector
		F mag() const { 
			return sqrt(magSq()); 
		}

		//psuedo-modifiers

		//@return the equivalent vector, but with @_x=@x
		Vector3<F> withX(F x) {
			return Vector3<F>(x, y(), z());
		}
		//@return the equivalent vector, but with @_y=@y
		Vector3<F> withY(F y) {
			return Vector3<F>(x(), y, z());
		}
		//@return the equivalent vector, but with @_y=@y
		Vector3<F> withZ(F z) {
			return Vector3<F>(x(), y(), z);
		}

		//unary negation operator (x = -y)
		Vector3<F> operator-() const {
			return *this * -1;
		}
		//Normalize the vector
		//@return a vector of magnitude 1, but with the same direction as `this'
		Vector3<F> norm() const {
			return *this / mag();
		}

		//operators:

		Vector3<F> operator+(const Vector3<F> &v) const {
			return Vector3<F>(x() + v.x(), y() + v.y(), z() + v.z());
		}
		Vector3<F>& operator+=(const Vector3<F> &v) {
			return *this = (*this + v);
		}

		Vector3<F> operator-(const Vector3<F> &v) const {
			return *this + (-v);
		}
		Vector3<F>& operator-=(const Vector3<F> &v) {
			return *this = (*this - v);
		}

		Vector3<F> operator*(F s) const {
			return Vector3<F>(s*x(), s*y(), s*z());
		}
		Vector3<F>& operator*=(F s) {
			return *this = (*this * s);
		}

		Vector3<F> operator/(F s) const {
			return *this * ((F)1. / s);
		}
		Vector3<F>& operator/=(F s) {
			return *this = (*this / s);
		}

		//The vector dot product: this . v
		F dot(const Vector3<F> &v) const {
			return x() * v.x() + y() * v.y() + z() * v.z();
		}
		F dot(F ox, F oy, F oz) const {
			return dot(Vector3<F>(ox, oy, oz));
		}
		//The vector cross product: this x v
		//|   i   j   k   |
		//|   ux  uy  uz  |
		//|   vx  vy  vz  |
		// = <uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx>
		Vector3<F> cross(const Vector3<F> &v) const {
			return Vector3<F>(y()*v.z() - z()*v.y(), z()*v.x() - x()*v.z(), x()*v.y() - y()*v.x());
		}
		Vector3<F> cross(F ox, F oy, F oz) const {
			return cross(Vector3<F>(ox, oy, oz));
		}
		//The scalar projection of `this' onto v.
		//That is, the scalar component of `this' in the direction of v.
		F scalarProj(const Vector3<F> &v) const {
			return this->dot(v) / v.mag();
		}
		//projection of `this' onto v.
		//That is, the vector componenent of `this' in the direction of v.
		//This is equivalent to (this->scalarProj(v)) * v.norm(),
		//  but we explicitly avoid the sqrt operations
		Vector3<F> proj(const Vector3<F> &v) const {
			return v * (this->dot(v) / v.magSq());
		}
		//@return the distance from @this to @v
		F distance(const Vector3<F> &v) const {
			return (*this - v).mag();
		}
		//@return the distance from @this to the point indicated by (@x, @y, @z)
		F distance(F x, F y, F z) const {
			return distance(Vector3<F>(x, y, z));
		}
};

//3-component (x, y, z) vector using (32-bit) floats
typedef Vector3<float> Vector3f;
//3-component (x, y, z) vector using higher precision (64-bit) doubles
typedef Vector3<double> Vector3d;

#endif