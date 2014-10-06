#ifndef COMMON_TYPESETTINGS_PRIMITIVES_H
#define COMMON_TYPESETTINGS_PRIMITIVES_H

#include <cstdint> //for uint8_t

typedef uint8_t AxisIdType;
typedef int GpioPinIdType; //Even if a machine only has 64 gpio pins, they may be separated into, say, 2 side-by-side bytes. So use an int by default.
typedef float CelciusType;

#endif
