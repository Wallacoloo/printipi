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

template <typename TupleT> class IODrivers {
    private:
		TupleT drivers;
	public:
		IODrivers(TupleT &&ioDrivers) : drivers(std::move(ioDrivers)) {}
		TupleT& tuple() {
			return drivers;
		}
		class iteratorbase;
	private:
		struct NoPredicate {
			bool operator()(const iteratorbase &self) {
				(void)self; //unused
				return true;
			}
		};
		template <typename T> struct IndexOptional : public T {
			using T::operator();
			template <std::size_t Index, typename ...Args> auto operator()(std::integral_constant<std::size_t, Index> index, Args&& ...args)
			 -> decltype(std::declval<T>()(std::forward<Args>(args)...)) const {
				(void)index; //unused
				return (*this)(std::forward<Args>(args)...);
			}
		};
        struct _WrapperLockAxis {
            template <typename T> void operator()(T &driver) const {
                driver.lockAxis();
            }
        };
        struct _WrapperUnlockAxis {
            template <typename T> void operator()(T &driver) const {
                driver.unlockAxis();
            }
        };
        struct _WrapperIsFan {
            template <typename T> bool operator()(T &driver) const {
                return driver.isFan();
            }
        };
        struct _WrapperIsHotend {
            template <typename T> bool operator()(T &driver) const {
                return driver.isHotend();
            }
        };
        struct _WrapperIsHeatedBed {
            template <typename T> bool operator()(T &driver) const {
                return driver.isHeatedBed();
            }
        };
        struct _WrapperIsServo {
            template <typename T> bool operator()(T &driver) const {
                return driver.isServo();
            }
        };
        struct _WrapperIsEndstop {
            template <typename T> bool operator()(T &driver) const {
                return driver.isEndstop();
            }
        };
        struct _WrapperIsEndstopTriggered {
            template <typename T> bool operator()(T &driver) const {
                return driver.isEndstopTriggered();
            }
        };
        struct _WrapperSetFanDutyCycle {
            template <typename T> void operator()(T &driver, float duty) const {
                driver.setFanDutyCycle(duty);
            }
        };
        struct _WrapperSetTargetTemperature {
            template <typename T> void operator()(T &driver, CelciusType temp) const {
                driver.setTargetTemperature(temp);
            }
        };
        struct _WrapperGetTargetTemperature {
            template <typename T> CelciusType operator()(T &driver) const {
                return driver.getTargetTemperature();
            }
        };
        struct _WrapperGetMeasuredTemperature {
            template <typename T> CelciusType operator()(T &driver) const {
                return driver.getMeasuredTemperature();
            }
        };
        struct _WrapperSetServoAngleDegrees {
            template <typename T> void operator()(T &driver, float angle) const {
                driver.setServoAngleDegrees(angle);
            }
        };
        struct _WrapperPeekNextEvent {
            template <typename T> OutputEvent operator()(T &driver) const {
                return driver.peekNextEvent();
            }
        };
        struct _WrapperConsumeNextEvent {
            template <typename T> void operator()(T &driver) const {
                return driver.consumeNextEvent();
            }
        };
        struct _WrapperOnIdleCpu {
            template <typename T> bool operator()(T &driver, OnIdleCpuIntervalT interval) const {
                return driver.onIdleCpu(interval);
            }
        };
    public:
        typedef IndexOptional<_WrapperLockAxis> WrapperLockAxis;
		typedef IndexOptional<_WrapperUnlockAxis> WrapperUnlockAxis;
		typedef IndexOptional<_WrapperIsFan> WrapperIsFan;
		typedef IndexOptional<_WrapperIsHotend> WrapperIsHotend;
		typedef IndexOptional<_WrapperIsHeatedBed> WrapperIsHeatedBed;
		typedef IndexOptional<_WrapperIsServo> WrapperIsServo;
		typedef IndexOptional<_WrapperIsEndstop> WrapperIsEndstop;
		typedef IndexOptional<_WrapperIsEndstopTriggered> WrapperIsEndstopTriggered;
		typedef IndexOptional<_WrapperSetFanDutyCycle> WrapperSetFanDutyCycle;
		typedef IndexOptional<_WrapperSetTargetTemperature> WrapperSetTargetTemperature;
		typedef IndexOptional<_WrapperGetTargetTemperature> WrapperGetTargetTemperature;
		typedef IndexOptional<_WrapperGetMeasuredTemperature> WrapperGetMeasuredTemperature;
		typedef IndexOptional<_WrapperSetServoAngleDegrees> WrapperSetServoAngleDegrees;
		typedef IndexOptional<_WrapperPeekNextEvent> WrapperPeekNextEvent;
		typedef IndexOptional<_WrapperConsumeNextEvent> WrapperConsumeNextEvent;
        typedef IndexOptional<_WrapperOnIdleCpu> WrapperOnIdleCpu;

		class iteratorbase {
            TupleT &tuple;
        	protected:
            	std::size_t idx;
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
                    return tupleCallOnIndex(tuple, WrapperLockAxis(), idx);
                }
                void unlockAxis() const {
                    return tupleCallOnIndex(tuple, WrapperUnlockAxis(), idx);
                }
                bool isFan() const {
                    return tupleCallOnIndex(tuple, WrapperIsFan(), idx);
                }
                bool isHotend() const {
                    return tupleCallOnIndex(tuple, WrapperIsHotend(), idx);
                }
                bool isHeatedBed() const {
                    return tupleCallOnIndex(tuple, WrapperIsHeatedBed(), idx);
                }
                bool isServo() const {
                    return tupleCallOnIndex(tuple, WrapperIsServo(), idx);
                }
                bool isEndstop() const {
                    return tupleCallOnIndex(tuple, WrapperIsEndstop(), idx);
                }
                bool isEndstopTriggered() const {
                    return tupleCallOnIndex(tuple, WrapperIsEndstopTriggered(), idx);
                }
                void setFanDutyCycle(float duty) {
                    tupleCallOnIndex(tuple, WrapperSetFanDutyCycle(), idx, duty);
                }
                void setTargetTemperature(CelciusType temp) {
                    tupleCallOnIndex(tuple, WrapperSetTargetTemperature(), idx, temp);
                }
                CelciusType getTargetTemperature() const {
                    return tupleCallOnIndex(tuple, WrapperGetTargetTemperature(), idx);
                }
                CelciusType getMeasuredTemperature() const {
                    return tupleCallOnIndex(tuple, WrapperGetMeasuredTemperature(), idx);
                }
                void setServoAngleDegrees(float angle) {
                    tupleCallOnIndex(tuple, WrapperSetServoAngleDegrees(), idx, angle);
                }
                OutputEvent peekNextEvent() const {
                    return tupleCallOnIndex(tuple, WrapperPeekNextEvent(), idx);
                }
                void consumeNextEvent() {
                    tupleCallOnIndex(tuple, WrapperConsumeNextEvent(), idx);
                }
                bool onIdleCpu(OnIdleCpuIntervalT interval) {
                    return tupleCallOnIndex(tuple, WrapperOnIdleCpu(), idx, interval);
                }
        };
    public:
        template <typename Predicate=NoPredicate> class iterator : public iteratorbase {
        	public:
        		iterator(TupleT &drivers, std::size_t idx=0, bool filterFirst=true)
        		 : iteratorbase(drivers, idx) {
        		 	while (filterFirst && !Predicate()(*this)) {
        		 		++this->idx;
        		 	}
        		}
        		void operator++() {
        		 	do {
                    	++this->idx;
                    } while (!Predicate()(*this));
        		}
        		iterator operator+(std::size_t add) {
        			iterator other = *this;
        			while (add--) {
        				++other;
        			}
        			return other;
        		}

    	};

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

    	iterinfo<> all() {
    		return iterinfo<>(drivers);
    	}
        iterator<> begin() {
            return all().begin();
        }
        iterator<> end() {
            return all().end();
        }
        iterator<> operator[](std::size_t idx) {
        	return all()[idx];
        }
        //pass the predicate by value to achieve type deduction
        template <typename Predicate> iterinfo<Predicate> filter(const Predicate &p) {
        	(void)p; //unused
        	return iterinfo<Predicate>(drivers);
    	}
    	iterinfo<WrapperIsFan> fans() {
    		return filter(WrapperIsFan());
    	}
    	iterinfo<WrapperIsHotend> hotends() {
    		return filter(WrapperIsHotend());
    	}
    	iterinfo<WrapperIsHeatedBed> heatedBeds() {
    		return filter(WrapperIsHeatedBed());
    	}
    	iterinfo<WrapperIsServo> servos() {
    		return filter(WrapperIsServo());
    	}
    	iterinfo<WrapperIsEndstop> endstops() {
    		return filter(WrapperIsEndstop());
    	}

        void lockAllAxes() {
        	all().apply(WrapperLockAxis());
		}
		void unlockAllAxes() {
		    all().apply(WrapperUnlockAxis());
		}
		void setHotendTemp(CelciusType temp) {
			hotends().apply(WrapperSetTargetTemperature(), temp);
		}
		void setBedTemp(CelciusType temp) {
			heatedBeds().apply(WrapperSetTargetTemperature(), temp);
		}
		void setServoAngleAtServoIndex(int index, float angleDeg) {
			servos()[index].setServoAngleDegrees(angleDeg);
		}
};

}


#endif