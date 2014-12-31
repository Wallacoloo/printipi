#ifndef COMMON_MATRIX_H
#define COMMON_MATRIX_H

#include "common/vector3.h"


/*
 * Matrices are useful in applying linear transformations. 
 * Notably, they can be used to adjust coordinates to a different coordinate-space in order to account for an unlevel bed.
 */
class Matrix3x3 {
    //r0, r1, r2 each represent one row of the matrix.
    Vector3f r0, r1, r2;
    public:
        Matrix3x3() : r0(), r1(), r2() {}
        Matrix3x3(float a00, float a01, float a02,
                float a10, float a11, float a12,
                float a20, float a21, float a22) :
            r0(a00, a01, a02),
            r1(a10, a11, a12),
            r2(a20, a21, a22) {}
        template <typename VecT> VecT transform(const VecT &xyz) const {
            return VecT(r0.dot(xyz), r1.dot(xyz), r2.dot(xyz));
    }

};

#endif
