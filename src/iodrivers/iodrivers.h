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
 

#ifndef DRIVERS_IODRIVERS_H
#define DRIVERS_IODRIVERS_H

#include <tuple>

namespace iodrv {

/*
 * Container type for IODrivers.
 * Provides several conveniences, like iterators and filtering
 *   while still maintaining type-information for each item
 */ 
template <typename TupleT> class IODrivers {
	TupleT drivers;
	public:
		//@ioDrivers std::tuple of IODrivers to construct from (will be moved).
		//  Eg IODrivers(std::make_tuple(Fan(), A4988(), Endstop()))
		IODrivers(TupleT &&ioDrivers) : drivers(std::move(ioDrivers)) {}
		//@return a reference to the underlying tuple of IODrivers
		TupleT& tuple() {
			return drivers;
		}
		class iteratorbase;
	private:
		//Filter function that always returns true
		struct NoPredicate {
			bool operator()(const iteratorbase &self) {
				(void)self; //unused
				return true;
			}
		};
		//tupleutil::callOnIndex and callOnAll pass both the index and the tuple item as an argument to the function
		//On the other hand, IODrivers::filter only passes the tuple item.
		//Use this class to wrap a function object that expects only the tuple item and make it also work with (index, item)
		template <typename T> struct IndexOptional : public T {
			using T::operator();
			template <std::size_t Index, typename ...Args> auto operator()(std::integral_constant<std::size_t, Index> index, Args&& ...args)
			 -> decltype(std::declval<T>()(std::forward<Args>(args)...)) const {
				(void)index; //unused
				return (*this)(std::forward<Args>(args)...);
			}
		};
		//(Unwrapped) Polymorphic adaptors for IODriver member functions...
		//These allow e.g. _GenericLockAxis()(myIoDriver) to call either A4988::lockAxis(), Drv8825::lockAxis(), etc, without a virtual function call.
		//This is done simply by templating the argument type.
		//Note: these functions are wrapped further down to accept either (driver, args...) or (index, driver, args...) as function arguments.
		//  Avoid using the types prefixed with _ unless necessary.
        struct _GenericLockAxis {
            template <typename T> void operator()(T &driver) const {
                driver.lockAxis();
            }
        };
        struct _GenericUnlockAxis {
            template <typename T> void operator()(T &driver) const {
                driver.unlockAxis();
            }
        };
        struct _GenericIsFan {
            template <typename T> bool operator()(T &driver) const {
                return driver.isFan();
            }
        };
        struct _GenericIsHotend {
            template <typename T> bool operator()(T &driver) const {
                return driver.isHotend();
            }
        };
        struct _GenericIsHeatedBed {
            template <typename T> bool operator()(T &driver) const {
                return driver.isHeatedBed();
            }
        };
        struct _GenericIsServo {
            template <typename T> bool operator()(T &driver) const {
                return driver.isServo();
            }
        };
        struct _GenericIsEndstop {
            template <typename T> bool operator()(T &driver) const {
                return driver.isEndstop();
            }
        };
        struct _GenericIsEndstopTriggered {
            template <typename T> bool operator()(T &driver) const {
                return driver.isEndstopTriggered();
            }
        };
        struct _GenericSetFanDutyCycle {
            template <typename T> void operator()(T &driver, float duty) const {
                driver.setFanDutyCycle(duty);
            }
        };
        struct _GenericSetTargetTemperature {
            template <typename T> void operator()(T &driver, CelciusType temp) const {
                driver.setTargetTemperature(temp);
            }
        };
        struct _GenericGetTargetTemperature {
            template <typename T> CelciusType operator()(T &driver) const {
                return driver.getTargetTemperature();
            }
        };
        struct _GenericGetMeasuredTemperature {
            template <typename T> CelciusType operator()(T &driver) const {
                return driver.getMeasuredTemperature();
            }
        };
        struct _GenericSetServoAngleDegrees {
            template <typename T> void operator()(T &driver, float angle) const {
                driver.setServoAngleDegrees(angle);
            }
        };
        struct _GenericPeekNextEvent {
            template <typename T> OutputEvent operator()(T &driver) const {
                return driver.peekNextEvent();
            }
        };
        struct _GenericConsumeNextEvent {
            template <typename T> void operator()(T &driver) const {
                return driver.consumeNextEvent();
            }
        };
        struct _GenericOnIdleCpu {
            template <typename T> bool operator()(T &driver, OnIdleCpuIntervalT interval) const {
                return driver.onIdleCpu(interval);
            }
        };
    public:
    	//Wrap the generic functions to either accept (driver, args...) or (index, driver, args...) as function arguments
    	//The index argument is provided by tupleutil::callOn*, but other places (e.g. predicates) expect no index argument.
        typedef IndexOptional<_GenericLockAxis>               GenericLockAxis;
		typedef IndexOptional<_GenericUnlockAxis>             GenericUnlockAxis;
		typedef IndexOptional<_GenericIsFan>                  GenericIsFan;
		typedef IndexOptional<_GenericIsHotend>               GenericIsHotend;
		typedef IndexOptional<_GenericIsHeatedBed>            GenericIsHeatedBed;
		typedef IndexOptional<_GenericIsServo>                GenericIsServo;
		typedef IndexOptional<_GenericIsEndstop>              GenericIsEndstop;
		typedef IndexOptional<_GenericIsEndstopTriggered>     GenericIsEndstopTriggered;
		typedef IndexOptional<_GenericSetFanDutyCycle>        GenericSetFanDutyCycle;
		typedef IndexOptional<_GenericSetTargetTemperature>   GenericSetTargetTemperature;
		typedef IndexOptional<_GenericGetTargetTemperature>   GenericGetTargetTemperature;
		typedef IndexOptional<_GenericGetMeasuredTemperature> GenericGetMeasuredTemperature;
		typedef IndexOptional<_GenericSetServoAngleDegrees>   GenericSetServoAngleDegrees;
		typedef IndexOptional<_GenericPeekNextEvent>          GenericPeekNextEvent;
		typedef IndexOptional<_GenericConsumeNextEvent>       GenericConsumeNextEvent;
        typedef IndexOptional<_GenericOnIdleCpu>              GenericOnIdleCpu;

        //Base type for iterators. 
        //Implements all the generic functions to apply to the IODriver currently pointed to
        //  as well as comparison operators.
        //But it does not implement operator++;
        //  this is expected to be implemented by the derived types which may do filtering.
		class iteratorbase {
            TupleT &tuple;
        	protected:
            	std::size_t idx;
            	bool isAtEnd() const {
            		return idx == std::tuple_size<TupleT>::value;
            	}
            public:
                iteratorbase(TupleT &tuple, std::size_t idx=0) : tuple(tuple), idx(idx) {
                }
                iteratorbase& operator*() {
                    return *this;
                }
                friend bool operator==(const iteratorbase &a, const iteratorbase &b) {
                    return a.idx == b.idx;
                }
                friend bool operator!=(const iteratorbase &a, const iteratorbase &b) {
                    return !(a == b);
                }
                void lockAxis() const {
                    return tupleCallOnIndex(tuple, GenericLockAxis(), idx);
                }
                void unlockAxis() const {
                    return tupleCallOnIndex(tuple, GenericUnlockAxis(), idx);
                }
                bool isFan() const {
                    return tupleCallOnIndex(tuple, GenericIsFan(), idx);
                }
                bool isHotend() const {
                    return tupleCallOnIndex(tuple, GenericIsHotend(), idx);
                }
                bool isHeatedBed() const {
                    return tupleCallOnIndex(tuple, GenericIsHeatedBed(), idx);
                }
                bool isServo() const {
                    return tupleCallOnIndex(tuple, GenericIsServo(), idx);
                }
                bool isEndstop() const {
                    return tupleCallOnIndex(tuple, GenericIsEndstop(), idx);
                }
                bool isEndstopTriggered() const {
                    return tupleCallOnIndex(tuple, GenericIsEndstopTriggered(), idx);
                }
                void setFanDutyCycle(float duty) {
                    tupleCallOnIndex(tuple, GenericSetFanDutyCycle(), idx, duty);
                }
                void setTargetTemperature(CelciusType temp) {
                    tupleCallOnIndex(tuple, GenericSetTargetTemperature(), idx, temp);
                }
                CelciusType getTargetTemperature() const {
                    return tupleCallOnIndex(tuple, GenericGetTargetTemperature(), idx);
                }
                CelciusType getMeasuredTemperature() const {
                    return tupleCallOnIndex(tuple, GenericGetMeasuredTemperature(), idx);
                }
                void setServoAngleDegrees(float angle) {
                    tupleCallOnIndex(tuple, GenericSetServoAngleDegrees(), idx, angle);
                }
                OutputEvent peekNextEvent() const {
                    return tupleCallOnIndex(tuple, GenericPeekNextEvent(), idx);
                }
                void consumeNextEvent() {
                    tupleCallOnIndex(tuple, GenericConsumeNextEvent(), idx);
                }
                bool onIdleCpu(OnIdleCpuIntervalT interval) {
                    return tupleCallOnIndex(tuple, GenericOnIdleCpu(), idx, interval);
                }
        };
    public:
    	//iterator class that also supports a filter predicate.
    	//@Predicate function that should return false for any item that is not part of the desired set.
    	//  Note that the Predicate function cannot easily store state info, as it may be instantiated for each item.
        template <typename Predicate=NoPredicate> class iterator : public iteratorbase {
        	public:
        		iterator(TupleT &drivers, std::size_t idx=0, bool filterFirst=true)
        		 : iteratorbase(drivers, idx) {
        		 	if (filterFirst && !Predicate()(*this)) {
        		 		++(*this);
        		 	}
        		}
        		//Increment to the next item that passes the predicate
        		void operator++() {
        		 	do {
                    	++this->idx;
                    } while (!Predicate()(*this) && !this->isAtEnd());
        		}
        		//Apply the increment operator @add times
        		iterator operator+(std::size_t add) {
        			iterator other = *this;
        			while (add--) {
        				++other;
        			}
        			return other;
        		}

    	};

    	//Allow one to build a filter before iterating.
    	//Also supports indexing and convenience functions that operate on the whole set.
    	template <typename Predicate=NoPredicate> class iterinfo {
    		TupleT &drivers;
	    	public:
	    		iterinfo(TupleT &drivers) : drivers(drivers) {}
	    		iterator<Predicate> begin() {
	    			return iterator<Predicate>(drivers);
	    		}
	    		iterator<Predicate> end() {
	    			return iterator<Predicate>(drivers, std::tuple_size<TupleT>::value, false);
	    		}
	    		iterator<Predicate> operator[](std::size_t idx) {
	    			return begin() + idx;
	    		}
	    		template <typename F, typename ...Args> void apply(F &&f, Args ...args) {
		    		for (auto &d : *this) {
		    			f(d, args...);
		    		}
		    	}
    	};

    	//return an iterable/indexable object containing ALL the iodrivers
    	iterinfo<> all() {
    		return iterinfo<>(drivers);
    	}
    	//begin iterator for the set of all IODrivers
        iterator<> begin() {
            return all().begin();
        }
        //end iterator for the set of all IODrivers
        iterator<> end() {
            return all().end();
        }
        //access an IODriver by index
        //@return an iterator pointing to the IODriver at index @idx
        iterator<> operator[](std::size_t idx) {
        	return all()[idx];
        }
        //obtain the set of all IODrivers that pass the filter Predicate.
        //  The predicate should accept an IODriver::iteratorbase as its argument 
        //    and return true if that IODriver is to be included in the set.
        //  The predicate is passed by value to achieve type deduction, but a new Predicate may be instantiated for each IODriver.
        //@return an iterable <iterinfo> object based on the Predicate
        template <typename Predicate> iterinfo<Predicate> filter(const Predicate &p) {
        	(void)p; //unused
        	return iterinfo<Predicate>(drivers);
    	}
    	//@return an iterable <iterinfo> object that contains only the fan IODrivers
    	iterinfo<GenericIsFan> fans() {
    		return filter(GenericIsFan());
    	}
    	//@return an iterable <iterinfo> object that contains only the hotend IODrivers
    	iterinfo<GenericIsHotend> hotends() {
    		return filter(GenericIsHotend());
    	}
    	//@return an iterable <iterinfo> object that contains only the heated bed IODrivers
    	iterinfo<GenericIsHeatedBed> heatedBeds() {
    		return filter(GenericIsHeatedBed());
    	}
    	//@return an iterable <iterinfo> object that contains only the servo IODrivers
    	iterinfo<GenericIsServo> servos() {
    		return filter(GenericIsServo());
    	}
    	//@return an iterable <iterinfo> object that contains only the endstop IODrivers
    	iterinfo<GenericIsEndstop> endstops() {
    		return filter(GenericIsEndstop());
    	}
    	//apply T::lockAxis on each IODriver in the set
        void lockAllAxes() {
        	all().apply(GenericLockAxis());
		}
		//apply T::unlockAxis on each IODriver in the set
		void unlockAllAxes() {
		    all().apply(GenericUnlockAxis());
		}
		//apply T::setTargetTemperature(temp) on each hotend in the set
		void setHotendTemp(CelciusType temp) {
			hotends().apply(GenericSetTargetTemperature(), temp);
		}
		//apply T::setTargetTemperature(temp) on each heated bed in the set
		void setBedTemp(CelciusType temp) {
			heatedBeds().apply(GenericSetTargetTemperature(), temp);
		}
		void setServoAngleAtServoIndex(int index, float angleDeg) {
			servos()[index].setServoAngleDegrees(angleDeg);
		}
};

}


#endif