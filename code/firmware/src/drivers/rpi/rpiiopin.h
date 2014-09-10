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
#include "drivers/rpi/rpi.h" //for initIO
#include "bcm2835.h" //for bcm2835_*

namespace drv {
namespace rpi {

template <uint8_t PinIdx, IoLevel Default=IoLow, bcm2835PUDControl PullUpDown=BCM2835_GPIO_PUD_OFF> class RpiIoPin : public IoPin {
	//InitRpiType _initRpi;
	public:
		RpiIoPin() {
			initIO();
			static IoPinOnExit<RpiIoPin<PinIdx, Default, PullUpDown>, Default> _onExit; //register deactivation of IO pin upon exit.
		}
		int id() const {
		    return PinIdx;
		}
		void makeDigitalOutput(IoLevel lev) {
			bcm2835_gpio_fsel(PinIdx, BCM2835_GPIO_FSEL_OUTP); //configure this pin as output
			bcm2835_gpio_set_pud(PinIdx, PullUpDown);
			digitalWrite(lev);
		}
		void makeDigitalInput() {
			bcm2835_gpio_fsel(PinIdx, BCM2835_GPIO_FSEL_INPT); //configure this pin as input
		}
		IoLevel digitalRead() const {
			return bcm2835_gpio_lev(PinIdx) == HIGH ? IoHigh : IoLow;
		}
		void digitalWrite(IoLevel lev) {
			bcm2835_gpio_write(PinIdx, lev == IoHigh ? HIGH : LOW);
		}

};

//template <uint8_t PinIdx, IoLevel Default, bcm2835PUDControl PullUpDown> IoPinOnExit<RpiIoPin<PinIdx, Default, PullUpDown>, Default> RpiIoPin<PinIdx, Default, PullUpDown>::_onExit;

}
}
#endif
