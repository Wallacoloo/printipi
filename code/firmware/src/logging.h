#include <stdio.h>
#ifdef NO_LOGGING
	#define DO_LOG false
#else
	#define DO_LOG true
#endif

#define LOG(format, args...) \
	if (DO_LOG && logging::isInfoEnabled()) { \
		printf(format, ## args); \
	}
#define LOGV(format, args...) \
	if (DO_LOG && logging::isVerboseEnabled()) { \
		printf(format, ## args); \
	}
#define LOGD(format, args...) \
	if (DO_LOG && logging::isVerboseEnabled()) { \
		printf(format, ## args); \
	}
	
namespace logging {

bool isInfoEnabled();
bool isVerboseEnabled();
void disable();
void enableVerbose();

}
