#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#ifdef DNO_LOGGING
	#define DO_LOG 0
#else
	#define DO_LOG 1
#endif
#ifdef DNO_LOG_M105
	#define NO_LOG_M105 1
#else
	#define NO_LOG_M105 0
#endif

#if DO_LOG == 1

#define LOGE(format, args...) \
	if (logging::isInfoEnabled()) { \
		fprintf(stderr, format, ## args); \
	}
#define LOGW(format, args...) \
	if (logging::isInfoEnabled()) { \
		printf(format, ## args); \
	}
#define LOG(format, args...) \
	if (logging::isInfoEnabled()) { \
		printf(format, ## args); \
	}
#define LOGD(format, args...) \
	if (logging::isVerboseEnabled()) { \
		printf(format, ## args); \
	}
#define LOGV(format, args...) \
	if (logging::isVerboseEnabled()) { \
		printf(format, ## args); \
	}
	
#else

#define LOGE(format, args...) {}
#define LOGW(format, args...) {}
#define LOG(format, args...) {}
#define LOGD(format, args...) {}
#define LOGV(format, args...) {}

#endif
	
namespace logging {

#if DO_LOG == 1

bool isInfoEnabled();
bool isVerboseEnabled();
void disable();
void enableVerbose();

#else

inline bool isInfoEnabled() {
	return false;
}
inline bool isVerboseEnabled() {
	return false;
}

inline void disable() {}
inline void enableVerbose() {}

#endif

}

#endif
