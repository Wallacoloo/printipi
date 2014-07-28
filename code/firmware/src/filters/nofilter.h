#ifndef FILTERS_NOFILTER_H
#define FILTERS_NOFILTER_H

struct NoFilter {
	inline float feed(float inp) {
		return inp;
	}
};


#endif
