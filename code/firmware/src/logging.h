#include <stdio.h>
#ifdef NO_LOGGING
	#define DO_LOG false
#else
	#define DO_LOG true
#endif

#define LOG(format, args...) \
	if (DO_LOG && logging::isEnabled()) { \
		printf(format, ## args); \
	}
	
namespace logging {

bool isEnabled();
void disable();

}
