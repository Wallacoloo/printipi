#ifndef DRIVERS_RPI_A4988
#define DRIVERS_RPI_A4988

#include <cstdint> //for uint8_t

#include "rpi.h"
#include "bcm2835.h"
#include "drivers/iodriver.h"
#include "logging.h"

namespace drv {
namespace rpi {

template <uint8_t STEPPIN, uint8_t DIRPIN> class A4988 : public IODriver {
	public:
		A4988() {
			initIO();
			bcm2835_gpio_fsel(STEPPIN, BCM2835_GPIO_FSEL_OUTP); //configure these pins as output
			bcm2835_gpio_fsel(DIRPIN, BCM2835_GPIO_FSEL_OUTP);
		}
		//A4988 is directed by putting a direction on the DIRPIN, and then
		//sending a pulse on the STEPPIN.
		void stepForward() {
			bcm2835_gpio_write(DIRPIN, HIGH); //set direction as forward
			cycleStepPin();
		}
		void stepBackward() {
			bcm2835_gpio_write(DIRPIN, LOW); //set direction as backward
			cycleStepPin();
		}
		void cycleStepPin() {
			LOGV("cycling pin %i\n", DIRPIN);
			bcm2835_gpio_write(STEPPIN, HIGH); 
			bcm2835_gpio_write(STEPPIN, LOW); //note: may need a (SHORT!) delay here.
		}
};


}
}

#endif
