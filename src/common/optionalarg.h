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