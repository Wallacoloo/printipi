#ifndef DRIVERS_AUTO_PRIMITIVEIOPIN_H
#define DRIVERS_AUTO_PRIMITIVEIOPIN_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_PRIMITIVEIOPIN
    #include PLATFORM_DRIVER_PRIMITIVEIOPIN
    typedef drv::TARGET_PLATFORM_LOWER::PrimitiveIoPin PrimitiveIoPin;
#else
    #include "drivers/generic/primitiveiopin.h"
    typedef drv::generic::PrimitiveIoPin PrimitiveIoPin;
#endif

#endif
