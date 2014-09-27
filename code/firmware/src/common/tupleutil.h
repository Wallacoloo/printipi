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

//callOnAll helper functions:

template <typename TupleT, std::size_t IdxPlusOne, typename Func, typename ...Args> struct __callOnAll {
    void operator()(TupleT &t, Func &f, Args... args) {
        __callOnAll<TupleT, IdxPlusOne-1, Func, Args...>()(t, f, args...); //call on all previous indices
        f(IdxPlusOne-1, std::get<IdxPlusOne-1>(t), args...); //call on our index.
    }
};

template <typename TupleT, typename Func, typename ...Args> struct __callOnAll<TupleT, 0, Func, Args...> {
    //handle the base recursion case.
    void operator()(TupleT &, Func &, Args... ) {}
};

template <typename TupleT, typename Func, typename ...Args> void callOnAll(TupleT &t, Func f, Args... args) {
    __callOnAll<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, args...);
};
//This second version allows to pass a function object by pointer, so that it can perhaps be modified.
template <typename TupleT, typename Func, typename ...Args> void callOnAll(TupleT &t, Func *f, Args... args) {
    __callOnAll<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, *f, args...);
};

//tupleReduce helper functions:

template <typename TupleT, std::size_t IdxPlusOne, typename Func, typename Reduce, typename ReducedDefault, typename ...Args> struct __callOnAllReduce {
    auto operator()(TupleT &t, Func &f, Reduce &r, ReducedDefault &d, Args... args) -> decltype(d()) {
        auto prev = __callOnAllReduce<TupleT, IdxPlusOne-1, Func, Reduce, ReducedDefault, Args...>()(t, f, r, d, args...); //result of all previous items;
        auto cur = f(IdxPlusOne-1, std::get<IdxPlusOne-1>(t), args...); //call on this index.
        return r(prev, cur);
    }
};

template <typename TupleT, typename Func, typename Reduce, typename ReducedDefault, typename ...Args> struct __callOnAllReduce<TupleT, 0, Func, Reduce, ReducedDefault, Args...> {
    //handle the base recursion case
    auto operator()(TupleT &, Func &, Reduce &, ReducedDefault &d, Args... ) -> decltype(d()) {
        return d();
    }
};

template <typename TupleT, typename Func, typename Reduce, typename ReducedDefault, typename ...Args> auto tupleReduce(TupleT &t, Func f, Reduce r, ReducedDefault d, Args... args) -> decltype(d()) {
    return __callOnAllReduce<TupleT, std::tuple_size<TupleT>::value, Func, Reduce, ReducedDefault, Args...>()(t, f, r, d, args...);
}


template <typename TupleT, typename Func, typename ...Args> bool tupleReduceLogicalOr(TupleT &t, Func f, Args... args) {
    //default value must be false, otherwise the only value ever returned would be <True>
    return tupleReduce(t, f, [](bool a, bool b) { return a||b; }, false, args...);
}

//callOnIndex helper functions:

template <typename TupleT, std::size_t MyIdxPlusOne, typename Func, typename ...Args> struct __callOnIndex {
    auto operator()(TupleT &t, Func &f, std::size_t desiredIdx, Args... args) -> decltype(f(std::get<MyIdxPlusOne-1>(t), args...)) {
        return desiredIdx < MyIdxPlusOne-1 ? __callOnIndex<TupleT, MyIdxPlusOne-1, Func, Args...>()(t, f, desiredIdx, args...) : f(std::get<MyIdxPlusOne-1>(t), args...);
    }
};
//recursion base case:
template <typename TupleT, typename Func, typename ...Args> struct __callOnIndex<TupleT, 1, Func, Args...> {
    auto operator()(TupleT &t, Func &f, std::size_t /*desiredIdx*/, Args... args) -> decltype(f(std::get<0>(t), args...)) {
        return f(std::get<0>(t), args...);
    }
};
//special case for TupleT::size == 0 (auto return type doesn't work, so we use void)
template <typename Func, typename ...Args> struct __callOnIndex<std::tuple<>, 0, Func, Args...> {
    void operator()(std::tuple<>&, Func &, std::size_t , Args...) {
    }
};

template <typename TupleT, typename Func, typename ...Args> auto tupleCallOnIndex(TupleT &t, Func f, std::size_t idx, Args... args) -> decltype(__callOnIndex<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, idx, args...)) {
    return __callOnIndex<TupleT, std::tuple_size<TupleT>::value, Func, Args...>()(t, f, idx, args...);
}

#endif
