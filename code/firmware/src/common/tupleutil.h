#ifndef COMMON_TUPLEUTIL_H
#define COMMON_TUPLEUTIL_H

/* 
 * Printipi//mathutil.h
 * (c) 2014 Colin Wallace
 *
 * This file provides utilities for manipulating tuples.
 * Namely, it provides a way to apply a polymorphic (templated) function to each item in a tuple.
 */

#include <tuple>

template <typename TupleT, std::size_t Idx, typename Func, typename ...Args> struct __callOnAll {
	void operator()(TupleT &t, Func f, Args... args) {
		__callOnAll<TupleT, Idx-1, Func, Args...>()(t, f, args...); //call on all previous indices
		f(Idx, std::get<Idx>(t), args...); //call on our index.
	}
};

template <typename TupleT, typename Func, typename ...Args> struct __callOnAll<TupleT, 0, Func, Args...> {
	//handle the base recursion case.
	void operator()(TupleT &t, Func f, Args... args) {
		f(0, std::get<0>(t), args...);
	}
};

template <typename TupleT, typename Func, typename ...Args> void callOnAll(TupleT &t, Func f, Args... args) {
	__callOnAll<TupleT, std::tuple_size<TupleT>::value-1, Func, Args...>()(t, f, args...);
};

#endif
