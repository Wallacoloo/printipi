#ifndef COMMON_TYPESETTINGS_H
#define COMMON_TYPESETTINGS_H

/* 
 * Printipi/common/typesettings.h
 * (c) 2014 Colin Wallace
 *
 * This file contains typedefs and enums for types used throughout the program.
 * It also processes some command-line based build settings (defines).
 *
 * TODO: it may be beneficial to split this into different files, eg typesettings/eventclockt.h, typesettings/sleept.h.
 *   This way circular includes can be avoided inside types contained in typesettings that depend on other type settings.
 */

//#include "typesettings/clocks.h"
//#include "typesettings/schedinterfacehardwarescheduler.h"

#include "typesettings/compileflags.h"

#include "typesettings/primitives.h"
#include "typesettings/enums.h"

#endif
