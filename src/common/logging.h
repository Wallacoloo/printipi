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

#ifndef COMMON_LOGGING_H
#define COMMON_LOGGING_H

#include <stdio.h>
#include <inttypes.h> //allow use of PRId64 by other files that make use of logging
#include "compileflags.h"

//64-bit printf specifier
#ifndef PRId64
    #define PRId64 "lld"
#endif

#if DO_LOG
    //NOTE: these logging functions must be implemented as macros, instead of templated functions
    // in order to get format verification at compile time.
    #define _LOG(tag, enableFunc, outputFile, format, args...) \
        if (enableFunc()) { \
            fprintf(outputFile, "[" tag "] " format, ## args); \
        }
#else
    #include <tuple>
    //make a tuple with the arguments and make it as (void) to avoid unused variable warnings.
    #define _LOG(args...) do { (void)std::make_tuple(##args); } while(0);
#endif

#define LOGE(format, args...) _LOG("ERR ",   logging::isInfoEnabled,    stderr, format, ##args)
#define LOGW(format, args...) _LOG("WARN",    logging::isInfoEnabled,    stdout, format, ##args)
#define LOG(format, args...)  _LOG("INFO",    logging::isInfoEnabled,    stdout, format, ##args)
#define LOGD(format, args...) _LOG("DBG ",   logging::isDebugEnabled,   stdout, format, ##args)
#define LOGV(format, args...) _LOG("VERB", logging::isVerboseEnabled, stdout, format, ##args)


#define _UNIQUE_NAME_LINE2( name, line ) name##line
#define _UNIQUE_NAME_LINE( name, line ) _UNIQUE_NAME_LINE2( name, line )
#define _UNIQUE_NAME( name ) _UNIQUE_NAME_LINE( name, __LINE__ )

#define _LOGONCE2(LOG_FUNC, FLAG_NAME, format, args...) \
    do { \
        static int FLAG_NAME=0; \
        if (!FLAG_NAME) { \
            FLAG_NAME=1; \
            LOG_FUNC("[ONCE] " format, ## args); \
        } \
    } while (0);
#define _LOGONCE(LOG_FUNC, format, args...) _LOGONCE2(LOG_FUNC, _UNIQUE_NAME(_LOG_ONCE_FLAG_), format, ## args)
#define LOGE_ONCE(format, args...) _LOGONCE(LOGE, format, ## args)
#define LOGW_ONCE(format, args...) _LOGONCE(LOGW, format, ## args)
#define LOG_ONCE(format, args...)  _LOGONCE(LOG, format, ## args)
#define LOGD_ONCE(format, args...) _LOGONCE(LOGD, format, ## args)
#define LOGV_ONCE(format, args...) _LOGONCE(LOGV, format, ## args)

    
/* 
 * This provides some functions that allow for logging information to stdout.
 * Use LOG for information logging, LOGE to log an error, LOGW for warnings, LOGD for debug logging, and LOGV for verbose debug logging.
 * LOG, LOGE, etc are not a part of the logging namespace because they are implemented as macros
 */

namespace logging {

#if DO_LOG == 1

    extern bool _info;
    extern bool _debug;
    extern bool _verbose;

    inline bool isInfoEnabled() {
        return _info;
    }
    inline bool isDebugEnabled() {
        return _debug;
    }
    inline bool isVerboseEnabled() {
        return _verbose;
    }

    inline void disable() {
        _info = false;
        _debug = false;
        _verbose = false;
    }

    inline void enableDebug(bool en=true) {
        _debug = en;
        LOG("debug logging set to: %i\n", en);
    }
    inline void enableVerbose(bool en=true) {
        _verbose = en;
        LOG("verbose logging set to: %i\n", en);
    }
    inline void enableInfo(bool en=true) {
        _info = en;
        LOG("info logging set to: %i\n", en);
    }

#else

    inline bool isInfoEnabled() {
        return false;
    }
    inline bool isVerboseEnabled() {
        return false;
    }
    inline bool isDebugEnabled() {
        return false;
    }

    inline void disable(bool en=true) {
        (void)en; 
    }
    inline void enableVerbose(bool en=true) {
        (void)en; 
    }
    inline void enableDebug(bool en=true) {
        (void)en; 
    }

#endif

}

#endif
