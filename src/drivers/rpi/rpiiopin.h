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
 * Printipi/drivers/rpi/rpiiopin.h
 * 
 * Implementation of src/drivers/iopin.h suitable for the RaspberryPi.
 * Used to pass pin-assignment information to other IODrivers, like telling the fan how to drive its pin, etc.
 */

#ifndef DRIVERS_RPI_RPIIOPIN_H
#define DRIVERS_RPI_RPIIOPIN_H

#include "drivers/iopin.h" //for IoPin
#include "drivers/rpi/mitpi.h"

namespace drv {
namespace rpi {

template <GpioPinIdType PinIdx, IoLevel Default=IoLow, mitpi::GpioPull PullUpDown=mitpi::GPIOPULL_NONE> class RpiIoPin : public IoPin {
    public:
        RpiIoPin() {
            mitpi::init();
            static IoPinOnExit<RpiIoPin<PinIdx, Default, PullUpDown>, Default> _onExit; //register deactivation of IO pin upon exit.
        }
        GpioPinIdType id() const {
            return PinIdx;
        }
        bool areWritesInverted() const {
            return false;
        }
        void makeDigitalOutput(IoLevel lev) {
            //bcm2835_gpio_fsel(PinIdx, BCM2835_GPIO_FSEL_OUTP); //configure this pin as output
            //bcm2835_gpio_set_pud(PinIdx, PullUpDown); //wtf? wrong place for a pull down...
            mitpi::makeOutput(PinIdx);
            digitalWrite(lev);
        }
        void makeDigitalInput() {
            //bcm2835_gpio_fsel(PinIdx, BCM2835_GPIO_FSEL_INPT); //configure this pin as input
            mitpi::makeInput(PinIdx);
            //TODO: add pullup/down support
            mitpi::setPinPull(PinIdx, PullUpDown);
        }
        IoLevel digitalRead() const {
            //return bcm2835_gpio_lev(PinIdx) == HIGH ? IoHigh : IoLow;
            return mitpi::readPinState(PinIdx) ? IoHigh : IoLow;
        }
        void digitalWrite(IoLevel lev) {
            //bcm2835_gpio_write(PinIdx, lev == IoHigh ? HIGH : LOW);
            mitpi::setPinState(PinIdx, lev == IoHigh ? 1 : 0);
        }

};


}
}
#endif
