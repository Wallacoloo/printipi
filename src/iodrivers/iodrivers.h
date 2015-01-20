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
	TupleT drivers;
	public:
		IODrivers(TupleT &&ioDrivers) : drivers(std::move(ioDrivers)) {}
		TupleT& tuple() {
			return drivers;
		}

		class iterator {
            struct WrapperLockAxis {
                template <typename T> void operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    driver.lockAxis();
                }
            };
            struct WrapperUnlockAxis {
                template <typename T> void operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    driver.unlockAxis();
                }
            };
            struct WrapperIsFan {
                template <typename T> bool operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.isFan();
                }
            };
            struct WrapperIsHotend {
                template <typename T> bool operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.isHotend();
                }
            };
            struct WrapperIsHeatedBed {
                template <typename T> bool operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.isHeatedBed();
                }
            };
            struct WrapperIsServo {
                template <typename T> bool operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.isServo();
                }
            };
            struct WrapperIsEndstop {
                template <typename T> bool operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.isEndstop();
                }
            };
            struct WrapperIsEndstopTriggered {
                template <typename T> bool operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.isEndstopTriggered();
                }
            };
            struct WrapperSetFanDutyCycle {
                template <typename T> void operator()(std::size_t index, T &driver, float duty) {
                    (void)index; //unused;
                    driver.setFanDutyCycle(duty);
                }
            };
            struct WrapperSetTargetTemperature {
                template <typename T> void operator()(std::size_t index, T &driver, CelciusType temp) {
                    (void)index; //unused;
                    driver.setTargetTemperature(temp);
                }
            };
            struct WrapperGetTargetTemperature {
                template <typename T> CelciusType operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.getTargetTemperature();
                }
            };
            struct WrapperGetMeasuredTemperature {
                template <typename T> CelciusType operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.getMeasuredTemperature();
                }
            };
            struct WrapperSetServoAngleDegrees {
                template <typename T> void operator()(std::size_t index, T &driver, float angle) {
                    (void)index; //unused;
                    driver.setServoAngleDegrees(angle);
                }
            };
            struct WrapperPeekNextEvent {
                template <typename T> OutputEvent operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.peekNextEvent();
                }
            };
            struct WrapperConsumeNextEvent {
                template <typename T> void operator()(std::size_t index, T &driver) {
                    (void)index; //unused;
                    return driver.consumeNextEvent();
                }
            };
            struct WrapperOnIdleCpu {
                template <typename T> bool operator()(std::size_t index, T &driver, OnIdleCpuIntervalT interval) {
                    (void)index; //unused;
                    return driver.onIdleCpu(interval);
                }
            };
            TupleT &tuple;
            std::size_t idx;
            public:
                iterator(TupleT &tuple, std::size_t idx=0) : tuple(tuple), idx(idx) {}
                iterator& operator*() {
                    return *this;
                }
                void operator++() {
                    ++idx;
                }
                iterator operator+(std::size_t add) {
                    return iterator(tuple, idx+add);
                }
                friend bool operator==(const iterator &a, const iterator &b) {
                    return a.idx == b.idx;
                }
                friend bool operator!=(const iterator &a, const iterator &b) {
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

        iterator begin() {
            return iterator(drivers, 0);
        }
        iterator end() {
            return iterator(drivers, std::tuple_size<TupleT>::value);
        }

        void lockAllAxes() {
		    for (auto& d : *this) {
		        d.lockAxis();
		    }
		}
		void unlockAllAxes() {
		    for (auto& d : *this) {
		        d.unlockAxis();
		    }
		}
		void setHotendTemp(CelciusType temp) {
		    for (auto& d : *this) {
		        if (d.isHotend()) {
		            d.setTargetTemperature(temp);
		        }
		    }
		}
		void setBedTemp(CelciusType temp) {
		    for (auto& d : *this) {
		        if (d.isHeatedBed()) {
		            d.setTargetTemperature(temp);
		        }
		    }
		}
		void setServoAngleAtServoIndex(int index, float angleDeg) {
		    for (auto& d : *this) {
		        if (d.isServo() && index-- == 0) {
		            d.setServoAngleDegrees(angleDeg);
		        }
		    }
		}
};

}


#endif