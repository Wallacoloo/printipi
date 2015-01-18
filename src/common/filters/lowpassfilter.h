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

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H


/* 
 * Provides a low-pass filter implementation for algorithms that allow a filter parameter, such as in src/iodrivers/tempcontrol.h
 * A low-pass filter allows low-frequency signals to pass while attenuating (decreasing the amplitude of) the higher frequencies.
 * 
 * Note that this implementation currently assumes it is fed 1 sample per second.
 * The implementation used is: http://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
 *  y[n] = y[n-1] + a(x[n] - y[n-1]), where x=input, y=output, n=sample number, a = dt / (RC + dt)
 *  http://www.dsplog.com/2007/12/02/digital-implementation-of-rc-low-pass-filter/
 *  y[n] = y[n-1] + k(x[n-1] - y[n-1]), where k=1/(RC)
 *  Lower k = tighter frequency response.
 *    that is, higher RC = lower frequency cutoff/more aggressive filtering
 *  Will need to convert this to work with seconds, and not sample numbers.
*/
class LowPassFilter {
    float _RC;
    float _last;
    inline float a() { return 1 / (_RC + 1); }
    public:
        inline LowPassFilter(float RC) : _RC(RC), _last(0) {}
        inline float feed(float inp) {
            _last = _last + a()*(inp - _last);
            return _last;
        }
};



#endif
