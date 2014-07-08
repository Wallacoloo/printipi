#include "logging.h"

namespace logging {

#ifdef DO_LOG

bool _info = true;
bool _verbose = false;

bool isInfoEnabled() {
	return _info;
}
bool isVerboseEnabled() {
	return _verbose;
}

void disable() {
	_info = false;
	_verbose = false;
}

void enableVerbose() {
	_verbose = true;
	LOGV("verbose logging enabled\n");
}

#else

bool isInfoEnabled() {
	return false;
}
bool isVerboseEnabled() {
	return false;
}

void disable() {}
void enableVerbose() {}

#endif

}
