#ifndef DRIVERS_A4988
#define DRIVERS_A4988

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

#include "drivers/iodriver.h"
#include "drivers/enabledisabledriver.h"
#include "common/logging.h"
#include "drivers/iopin.h"

namespace drv {

template <typename StepPin=NoPin, typename DirPin=NoPin, typename Enabler=NullEnabler> class A4988 : public IODriver {
	Enabler enabler;
	StepPin stepPin;
	DirPin dirPin;
	public:
		A4988() : IODriver(this) {
			//initIO();
			//bcm2835_gpio_fsel(STEPPIN, BCM2835_GPIO_FSEL_OUTP); //configure these pins as output
			//bcm2835_gpio_fsel(DIRPIN, BCM2835_GPIO_FSEL_OUTP);
			stepPin.makeDigitalOutput(IoLow);
			dirPin.makeDigitalOutput(IoLow);
			//Enabler::enable();
			enabler.enable();
		}
		//A4988 is directed by putting a direction on the DIRPIN, and then
		//sending a pulse on the STEPPIN.
		/*static void deactivate() {
			Enabler::disable(); //will be called directly on the Enabler.
			//enabler.disable();
			//bcm2835_gpio_fsel(STEPPIN, BCM2835_GPIO_FSEL_INPT); //unmount pins.
			//bcm2835_gpio_fsel(DIRPIN, BCM2835_GPIO_FSEL_INPT);
		}*/
		void lockAxis() {
			//Enabler::enable();
			enabler.enable();
		}
		void unlockAxis() {
			//Enabler::disable();
			enabler.disable();
		}
		void stepForward() {
			dirPin.digitalWrite(IoHigh);
			//bcm2835_gpio_write(DIRPIN, HIGH); //set direction as forward
			cycleStepPin();
		}
		void stepBackward() {
			dirPin.digitalWrite(IoLow);
			//bcm2835_gpio_write(DIRPIN, LOW); //set direction as backward
			cycleStepPin();
		}
	private:
		void cycleStepPin() {
			//LOGV("cycling pin %i\n", DIRPIN);
			stepPin.digitalWrite(IoHigh);
			//bcm2835_gpio_write(STEPPIN, HIGH); 
			bcm2835_delayMicroseconds(2); //delayMicroseconds(n) can delay anywhere from (n-1) to n. Need to delay 2 uS to get minimum of 1 uS. Note, this is a waste of 700-1400 cycles.
			stepPin.digitalWrite(IoLow);
			//bcm2835_gpio_write(STEPPIN, LOW); //note: may need a (SHORT!) delay here.
			//bcm2835_delayMicroseconds(1);
		}
};


}

#endif
