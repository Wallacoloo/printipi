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
//#include "drivers/enabledisabledriver.h"
#include "drivers/iopin.h" //for NoPin
#include "common/logging.h"
#include "drivers/iopin.h"

namespace drv {

template <typename StepPin=NoPin, typename DirPin=NoPin, typename EnablePin=NoPin> class A4988 : public IODriver {
	EnablePin enablePin;
	StepPin stepPin;
	DirPin dirPin;
	public:
		A4988() : IODriver() {
			//initIO();
			//bcm2835_gpio_fsel(STEPPIN, BCM2835_GPIO_FSEL_OUTP); //configure these pins as output
			//bcm2835_gpio_fsel(DIRPIN, BCM2835_GPIO_FSEL_OUTP);
			stepPin.makeDigitalOutput(IoLow);
			dirPin.makeDigitalOutput(IoLow);
			//Enabler::enable();
			//enabler.enable();
			enablePin.makeDigitalOutput(IoHigh); //set as output and enable.
		}
		void lockAxis() {
			//Enabler::enable();
			//enabler.enable();
			//enablePin.makeDigitalOutput(IoHigh); //enable.
			enablePin.digitalWrite(IoHigh);
		}
		void unlockAxis() {
			//Enabler::disable();
			//enabler.disable();
			//enablePin.makeDigitalOutput(IoLow); //disable.
			enablePin.digitalWrite(IoLow);
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
		inline bool isEventOutputSequenceable(const Event&) {
	        //Both stepForward and stepBackward events are sequenceable.
	        return true;
	    }
	private:
		//A4988 is directed by putting a direction on the DIRPIN, and then
		//sending a pulse on the STEPPIN.
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
