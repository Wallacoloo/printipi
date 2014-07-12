#ifndef DRIVERS_RPI_A4988
#define DRIVERS_RPI_A4988

/*
 * The A4988 is a current-chopping stepper motor driver IC.
 * It is used in the StepStick, Pololu stepper motor drivers, etc.
 * It consists of 2 control pins: STEP and DIRECTION.
 * Documentation: http://www.pololu.com/file/download/a4988_DMOS_microstepping_driver_with_translator.pdf?file_id=0J450
 * Minimum STEP high pulse: 1uS
 * Minimum STEP low pulse:  1uS
 * Low -> High transition on STEP pin trigger the step.
*/

#include <cstdint> //for uint8_t

#include "rpi.h"
#include "bcm2835.h"
#include "drivers/iodriver.h"
#include "logging.h"

namespace drv {
namespace rpi {

template <uint8_t STEPPIN, uint8_t DIRPIN> class A4988 : public IODriver {
	public:
		A4988() : IODriver(this) {
			initIO();
			bcm2835_gpio_fsel(STEPPIN, BCM2835_GPIO_FSEL_OUTP); //configure these pins as output
			bcm2835_gpio_fsel(DIRPIN, BCM2835_GPIO_FSEL_OUTP);
		}
		//A4988 is directed by putting a direction on the DIRPIN, and then
		//sending a pulse on the STEPPIN.
		/*static void deactivate() {
			bcm2835_gpio_fsel(STEPPIN, BCM2835_GPIO_FSEL_INPT); //unmount pins.
			bcm2835_gpio_fsel(DIRPIN, BCM2835_GPIO_FSEL_INPT);
		}*/
		void stepForward() {
			bcm2835_gpio_write(DIRPIN, HIGH); //set direction as forward
			cycleStepPin();
		}
		void stepBackward() {
			bcm2835_gpio_write(DIRPIN, LOW); //set direction as backward
			cycleStepPin();
		}
		void cycleStepPin() {
			//LOGV("cycling pin %i\n", DIRPIN);
			bcm2835_gpio_write(STEPPIN, HIGH); 
			bcm2835_delayMicroseconds(2);
			bcm2835_gpio_write(STEPPIN, LOW); //note: may need a (SHORT!) delay here.
			bcm2835_delayMicroseconds(1);
		}
};


}
}

#endif
