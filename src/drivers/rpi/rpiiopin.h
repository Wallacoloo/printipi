#ifndef DRIVERS_RPI_RPIIOPIN_H
#define DRIVERS_RPI_RPIIOPIN_H

/* 
 * Printipi/drivers/rpi/rpiiopin.h
 * (c) 2014 Colin Wallace
 * 
 * Implementation of src/drivers/iopin.h suitable for the RaspberryPi.
 * Used to pass pin-assignment information to other IODrivers, like telling the fan how to drive its pin, etc.
 */

#include "drivers/iopin.h" //for IoPin
//#include "drivers/rpi/rpi.h" //for initIO
#include "drivers/rpi/mitpi.h"
//#include "bcm2835.h" //for bcm2835*

namespace drv {
namespace rpi {

//template <GpioPinIdType PinIdx, IoLevel Default=IoLow, bcm2835PUDControl PullUpDown=BCM2835_GPIO_PUD_OFF> class RpiIoPin : public IoPin {
template <GpioPinIdType PinIdx, IoLevel Default=IoLow, mitpi::GpioPull PullUpDown=mitpi::GPIOPULL_NONE> class RpiIoPin : public IoPin {
    //InitRpiType _initRpi;
    public:
        RpiIoPin() {
            //initIO();
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
