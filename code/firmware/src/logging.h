#include <stdio.h>
#ifdef NO_LOGGING
	#define DO_LOG false
#else
	#define DO_LOG true
#endif

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
	
namespace logging {

bool isInfoEnabled();
bool isVerboseEnabled();
void disable();
void enableVerbose();

}
