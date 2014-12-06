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

#include <cmath>

template <typename F> class Vector3 {
	//mathematical vector utility
	F _x, _y, _z;
	public:
		Vector3() : _x(0), _y(0), _z(0) {}
		Vector3(F x, F y, F z) : _x(x), _y(y), _z(z) {}
		template <typename Other> Vector3(const Other &v)
		  : _x(v.x()), _y(v.y()), _z(v.z()) {}

		//accessors:
		F x() const { return _x; }
		F y() const { return _y; }
		F z() const { return _z; }
		F magSq() const { 
			//the square of the magnitude (length) of the vector.
			//equivalent to this->mag() * this->mag(), but less verbose and explicitly avoids the sqrt operation.
			return this->dot(*this); 
		}
		F mag() const { 
			//magnitude (length) of the vector
			return sqrt(magSq()); 
		}

		//unary operators
		Vector3<F> operator-() const {
			return *this * -1;
		}
		Vector3<F> norm() const {
			//Normalize the vector; return a vector of magnitude 1, but with the same direction as `this'
			return *this / mag();
		}

		//operators:
		Vector3<F> operator+(const Vector3<F> &v) const {
			return Vector3<F>(x() + v.x(), y() + v.y(), z() + v.z());
		}
		Vector3<F> operator-(const Vector3<F> &v) const {
			return Vector3<F>(x() - v.x(), y() - v.y(), z() - v.z());
		}
		Vector3<F> operator*(F s) const {
			return Vector3<F>(s*x(), s*y(), s*z());
		}
		Vector3<F> operator/(F s) const {
			return *this * ((F)1. / s);
		}

		F dot(const Vector3<F> &v) const {
			//The vector dot product: this . v
			return x() * v.x() + y() * v.y() + z() * v.z();
		}
		Vector3<F> cross(const Vector3<F> &v) const {
			//The vector cross product: this x v
			//|   i   j   k   |
			//|   ux  uy  uz  |
			//|   vx  vy  vz  |
			// = <uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx>
			return Vector3<F>(y()*v.z() - z()*v.y(), z()*v.x() - x()*v.z(), x()*v.y() - y()*v.x());
		}
		F scalarProj(const Vector3<F> &v) const {
			//the scalar projection of `this' onto v.
			//That is, the scalar component of `this' in the direction of v.
			return this->dot(v) / v.mag();
		}
		Vector3<F> proj(const Vector3<F> &v) const {
			//projection of `this' onto v.
			//That is, the vector componenent of `this' in the direction of v.
			//This is equivalent to (this->scalarProj(v)) * v.norm(),
			//  but we explicitly avoid the sqrt operations
			return v * (this->dot(v) / v.magSq());
		}
};

typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

#endif