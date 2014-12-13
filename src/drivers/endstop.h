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

/* 
 * Printipi/drivers/axisstepper.h
 *
 * Endstops are queriable switches placed at the axis limits.
 * They typically represent a "known" point to which the device can be homed upon initiailization,
 *   or a point beyond which the device should not be pushed.
 */

#ifndef DRIVERS_ENDSTOP_H
#define DRIVERS_ENDSTOP_H

#include <utility> //for std::move
#include "iodriver.h"
#include "iopin.h"

namespace drv {


class Endstop : public IODriver {
    IoPin pin;
    public:
        Endstop(IoPin &&pin) : IODriver(), pin(std::move(pin)) {
            this->pin.makeDigitalInput();
        }
        bool isTriggered() {
            bool t = pin.digitalRead() == IoHigh;
            LOGV("LeverEndstop is %i\n", t);
            return t;
        }
};

//default Endstop implementation which always acts as if untriggered:
//typedef Endstop<NoPin> EndstopNoExist;
typedef Endstop EndstopNoExist;

}
#endif
