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

#ifndef COMMON_OPTIONALARG_H
#define COMMON_OPTIONALARG_H

//allow for a function default value that is not a constant.
//Example:
//void f(OptionalArg<float> velocity=OptionalArg<float>::NotPresent) {
//  doSomething(velocity.get(this->currentVelocity()));
//}
//
template <typename T> class OptionalArg {
	bool wasInit;
	T value;
	public:
		static const OptionalArg<T> NotPresent;
		OptionalArg() : wasInit(false) {}
		OptionalArg(const T &value) : wasInit(true), value(value) {}
		//get the OptionalArg's value. If the argument wasn't passed, then return @fallback
		T get(const T &fallback) const {
			return wasInit ? value : fallback;
		}
};

template <typename T> const OptionalArg<T> OptionalArg<T>::NotPresent = OptionalArg<T>();

#endif