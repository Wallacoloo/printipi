#ifndef COMMON_TUPLEUTIL_H
#define COMMON_TUPLEUTIL_H

/* 
 * Printipi/common/tupleutil.h
 * (c) 2014 Colin Wallace
 *
 * This file provides utilities for manipulating tuples.
 * Namely, it provides a way to apply a polymorphic (templated) function to each item in a tuple.
 */

#include <tuple>
#include <cassert>
#include <utility> //for std::forward
//for std::integral_constant
#include <type_traits>

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


    //tupleReduce helper functions:
    template <typename TupleT, std::size_t IdxPlusOne, typename Func, typename Reduce, typename ReducedDefault, typename ...Args> struct __callOnAllReduce {
        auto operator()(TupleT &t, Func &f, Reduce &r, ReducedDefault d, Args... args) -> decltype(d) {
            auto prev = __callOnAllReduce<TupleT, IdxPlusOne-1, Func, Reduce, ReducedDefault, Args...>()(t, f, r, d, args...); //result of all previous items;
            auto cur = f(IdxPlusOne-1, std::get<IdxPlusOne-1>(t), args...); //call on this index.
            return r(prev, cur);
        }
    };

    //handle tupleReduce base recursion case
    template <typename TupleT, typename Func, typename Reduce, typename ReducedDefault, typename ...Args> struct __callOnAllReduce<TupleT, 0, Func, Reduce, ReducedDefault, Args...> {
        auto operator()(TupleT &, Func &, Reduce &, ReducedDefault d, Args... ) -> decltype(d) {
            return d;
        }
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
//Apply @r( @r( d, @f(@t[0], @args...) ), @f(@t[1], @args...) ), ...
//This function is also sometime known as "fold".
//If, for example, the reducing function @r sums its two arguments, then the result will be the sum of ALL items in the tuple.
template <typename TupleT, typename Func, typename Reduce, typename ReducedDefault, typename ...Args> auto tupleReduce(TupleT &t, Func f, Reduce r, ReducedDefault d, Args... args) -> decltype(d) {
    return __callOnAllReduce<TupleT, std::tuple_size<TupleT>::value, Func, Reduce, ReducedDefault, Args...>()(t, f, r, d, args...);
}
//Return @f(@t[0], args...) || @f(@t[1], args...) || @f(@t[2], args...) || ...
template <typename TupleT, typename Func, typename ...Args> bool tupleReduceLogicalOr(TupleT &t, Func f, Args... args) {
    //default value must be false, otherwise the only value ever returned would be <True>
    return tupleReduce(t, f, [](bool a, bool b) { return a||b; }, false, args...);
}
//Return @f(@t[@idx], args...)
//Note: if @idx > the size of the tuple, behavior is undefined.
//  Most likely, that would result in applying @f to the last item in the tuple (but no guarantee)
template <typename TupleT, typename Func, typename ...Args> auto tupleCallOnIndex(TupleT &t, Func f, std::size_t idx, Args... args) -> decltype(__callOnIndex<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, idx, args...)) {
    return __callOnIndex<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, idx, args...);
}
}

using namespace tupleutil;
#endif
