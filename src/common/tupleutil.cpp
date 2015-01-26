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

#include "tupleutil.h"
#include "catch.hpp"

struct AddIndex {
	//to each element in the tuple, add index*@scale to it (and overwrite it)
	template <typename T> void operator()(std::size_t index, T& t, int scale) {
		t += scale*index;
	}
};

struct ReturnAddIndex {
	template <typename T> float operator()(std::size_t index, T& t, int scale) {
		return t + scale*index;
	}
};

struct TestEquality {
	template <typename T> bool operator()(std::size_t index, T& t, int test) {
		(void)index; //unused
		return t == test;
	}
};

TEST_CASE("Tupleutil operations are correct", "[tupleutil]") {
	std::tuple<int, float, int, double> t(1, 2.5, 3, 4.25);
	SECTION("Test tupleutil::callOnAll") {
		callOnAll(t, AddIndex(), 3);
		REQUIRE(std::get<0>(t) == Approx(1   +3*0));
		REQUIRE(std::get<1>(t) == Approx(2.5 +3*1));
		REQUIRE(std::get<2>(t) == Approx(3   +3*2));
		REQUIRE(std::get<3>(t) == Approx(4.25+3*3));
	}
	SECTION("Test tupleutil::tupleCallOnIndex") {
		tupleCallOnIndex(t, AddIndex(), 2, 3);
		REQUIRE(std::get<0>(t) == Approx(1       ));
		REQUIRE(std::get<1>(t) == Approx(2.5     ));
		REQUIRE(std::get<2>(t) == Approx(3   +3*2));
		REQUIRE(std::get<3>(t) == Approx(4.25    ));
	}
}
