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

#include "vector3.h"
#include "catch.hpp"

template <typename V> void vectorTestSuite() {
	auto REQUIRE_VECS_EQ = [](V a, V b) {
		REQUIRE(a.x() == Approx(b.x()));
		REQUIRE(a.y() == Approx(b.y()));
		REQUIRE(a.z() == Approx(b.z()));
	};
	SECTION("Vector magSq is correct") {
		REQUIRE(V(2, 3, 6).magSq() == Approx(49));
	}
	SECTION("Vector mag is correct") {
		REQUIRE(V(2, 3, 6).mag() == Approx(7));
	}
    SECTION("Vector negation is correct") {
        REQUIRE_VECS_EQ(-V(2.0, -3.0, 1.5), V(-2.0, 3.0, -1.5));
    }
    SECTION("Vector normalization is correct") {
    	REQUIRE_VECS_EQ(V(2.0, 3.0, 6.0).norm(), V(2.0/7.0, 3.0/7.0, 6.0/7.0));
    }
    SECTION("Vector scaling is correct") {
        REQUIRE_VECS_EQ(V(2.0, -3.0, 1.5)*4, V(8.0, -12.0, 6.0));
    }
    SECTION("Vector scaling (by division) is correct") {
    	REQUIRE_VECS_EQ(V(2.0, -3.0, 4.0)/4.0, V(0.5, -0.75, 1.0));
    }
    SECTION("Vector addition is correct") {
        REQUIRE_VECS_EQ(V(2.0, -3.0, 1.5)+V(-1.5, 1.5, 0.5), V(0.5, -1.5, 2.0));
    }
    SECTION("Vector subtraction is correct") {
        REQUIRE_VECS_EQ(V(2.0, -3.0, 1.5)-V(-1.5, 1.5, 0.5), V(3.5, -4.5, 1.0));
    }
    SECTION("Vector dot product is correct") {
    	REQUIRE(V(5, 3, 2).dot(V(-2, 3, 4)) == Approx(7));
    }
    SECTION("Vector cross product is correct") {
    	REQUIRE_VECS_EQ(V(1, 2, 3).cross(V(2, 3, 4)), V(-1, 2, -1));
    }
    SECTION("Vector scalar projection is correct") {
    	REQUIRE(V(3, 4, 5).scalarProj(V(2, 0, 0)) == Approx(3));
    	REQUIRE(V(3, 4, 5).scalarProj(V(0, 2, 0)) == Approx(4));
    	REQUIRE(V(3, 4, 5).scalarProj(V(2, 6, 9)) == Approx(75.0/11.0));
    }
    SECTION("Vector project is correct") {
    	REQUIRE_VECS_EQ(V(3, 1, 4).proj(V(6, 0, 8)), V(3, 0, 4));
    }
    SECTION("Vector distance is correct") {
    	REQUIRE(V(2, 1, 3).distance(V(6, 2, -5)) == Approx(9.0));
    }
}

TEST_CASE("Vector3f math operations are accurate", "[vector3]") {
	vectorTestSuite<Vector3f>();
}
TEST_CASE("Vector3d math operations are accurate", "[vector3]") {
	vectorTestSuite<Vector3d>();
}