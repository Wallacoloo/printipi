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

#ifndef COMMON_TUPLEUTIL_H
#define COMMON_TUPLEUTIL_H

#include <tuple>
#include <cassert>
#include <utility> //for std::forward
//for std::integral_constant
#include <type_traits>


/* 
 * This namespace provides utilities for manipulating tuples.
 * Namely, it provides a way to apply a polymorphic (templated) function to each item in a tuple (callOnAll)
 * Or call a function with the nth element of a tuple, where n is not a compile-time constant. (tupleCallOnIndex)
 */
namespace tupleutil {
namespace {
    //place helper functions in an unnamed namespace to limit visibility and hint the documentation generator

    //callOnAll helper functions:
    template <typename TupleT, std::size_t IdxPlusOne, typename Func, typename ...Args> struct __callOnAll {
        void operator()(TupleT &t, Func &f, Args... args) {
            __callOnAll<TupleT, IdxPlusOne-1, Func, Args...>()(t, f, args...); //call on all previous indices
            f(std::integral_constant<std::size_t, IdxPlusOne-1>(), std::get<IdxPlusOne-1>(t), args...); //call on our index.
        }
    };

    //handle callOnAll base recursion case.
    template <typename TupleT, typename Func, typename ...Args> struct __callOnAll<TupleT, 0, Func, Args...> {
        void operator()(TupleT &, Func &, Args... ) {}
    };

    //callOnIndex helper functions:
    template <typename TupleT, std::size_t MyIdxPlusOne, typename Func, typename ...Args> struct __callOnIndex {
        auto operator()(TupleT &t, Func &f, std::size_t desiredIdx, Args... args)
           -> decltype(f(std::integral_constant<std::size_t, MyIdxPlusOne-1>(), std::get<MyIdxPlusOne-1>(t), args...)) {
            return desiredIdx < MyIdxPlusOne-1 ? __callOnIndex<TupleT, MyIdxPlusOne-1, Func, Args...>()(t, f, desiredIdx, args...)
                                               : f(std::integral_constant<std::size_t, MyIdxPlusOne-1>(), std::get<MyIdxPlusOne-1>(t), args...);
        }
    };
    //callOnIndex recursion base case:
    template <typename TupleT, typename Func, typename ...Args> struct __callOnIndex<TupleT, 1, Func, Args...> {
        auto operator()(TupleT &t, Func &f, std::size_t desiredIdx, Args... args) -> decltype(f(std::integral_constant<std::size_t, 0>(), std::get<0>(t), args...)) {
            (void)desiredIdx; //unused
            return f(std::integral_constant<std::size_t, 0>(), std::get<0>(t), args...);
        }
    };
    //special callOnIndex case for TupleT::size == 0 (auto return type doesn't work, so we use void)
    template <typename Func, typename ...Args> struct __callOnIndex<std::tuple<>, 0, Func, Args...> {
        void operator()(std::tuple<>&, Func &, std::size_t , Args...) {
        }
    };
}


//Apply @f(item, @args...) for each item in the tuple @t
template <typename TupleT, typename Func, typename ...Args> void callOnAll(TupleT &t, Func f, Args... args) {
    __callOnAll<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, args...);
}
//Apply @f(item, @args...) for each item in the tuple @t
//This second version allows to pass a function object by pointer, so that it can perhaps be modified. TODO: Maybe just use an auto reference (Func &&f)?
template <typename TupleT, typename Func, typename ...Args> void callOnAll(TupleT &t, Func *f, Args... args) {
    __callOnAll<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, *f, args...);
}
//Return @f(@t[@idx], args...)
//Note: if @idx > the size of the tuple, behavior is undefined.
//  Most likely, that would result in applying @f to the last item in the tuple (but no guarantee)
template <typename TupleT, typename Func, typename ...Args> auto tupleCallOnIndex(TupleT &t, Func f, std::size_t idx, Args... args)
   -> decltype(__callOnIndex<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, idx, args...)) {
    return __callOnIndex<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, idx, args...);
}

}

using namespace tupleutil;
#endif
