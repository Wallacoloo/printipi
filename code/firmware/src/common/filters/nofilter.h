#ifndef FILTERS_NOFILTER_H
#define FILTERS_NOFILTER_H

/* 
 * Printipi/filters/nofilter.h
 * (c) 2014 Colin Wallace
 *
 * Provides a default filter for algorithms that allow a filter parameter, such as in src/drivers/tempcontrol.h
 * The filter's output is the same as its input.
 */

struct NoFilter {
    inline float feed(float inp) {
        return inp;
    }
};


#endif
