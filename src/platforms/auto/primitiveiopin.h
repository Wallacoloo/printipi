#ifndef PLATFORMS_AUTO_PRIMITIVEIOPIN_H
#define PLATFORMS_AUTO_PRIMITIVEIOPIN_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_PRIMITIVEIOPIN
    #include PLATFORM_DRIVER_PRIMITIVEIOPIN
    typedef plat::TARGET_PLATFORM_LOWER::PrimitiveIoPin PrimitiveIoPin;
#else
    #include "platforms/generic/primitiveiopin.h"
    typedef plat::generic::PrimitiveIoPin PrimitiveIoPin;
#endif

#endif
