#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

/* 
 * Printipi/filters/nofilter.h
 * (c) 2014 Colin Wallace
 *
 * Provides a low-pass filter implementation for algorithms that allow a filter parameter, such as in src/drivers/tempcontrol.h
 * A low-pass filter allows low-frequency signals to pass while attenuating (decreasing the amplitude of) the higher frequencies.
 * 
 * Note that this implementation currently assumes it is fed 1 sample per second.
 */


/* http://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
   y[n] = y[n-1] + a(x[n] - y[n-1]), where x=input, y=output, n=sample number, a = dt / (RC + dt)
   http://www.dsplog.com/2007/12/02/digital-implementation-of-rc-low-pass-filter/
   y[n] = y[n-1] + k(x[n-1] - y[n-1]), where k=1/(RC)
   Lower k = tighter frequency response
   Will need to convert this to work with seconds, and not sample numbers.
*/

template <unsigned RAD_SEC_1000> class LowPassFilter {
    static constexpr float RC() { return RAD_SEC_1000/1000.; }
    //for now, just assume that the time between samples is a steady 1 second.
    static constexpr float a() { return 1 / (RC() + 1); }
    float _last;
    public:
        LowPassFilter() : _last(0) {}
        float feed(float inp) {
            _last = _last + a()*(inp - _last);
            return _last;
            //return inp;
        }
};



#endif
