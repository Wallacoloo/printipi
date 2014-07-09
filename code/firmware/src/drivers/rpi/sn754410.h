#ifndef DRIVERS_RPI_SN754410_H
#define DRIVERS_RPI_SN754410_H

#include <cstdint> //for uint8_t
#include <array>

#include "rpi.h"
#include "bcm2835.h"
#include "drivers/iodriver.h"

namespace drv {
namespace rpi {

template <uint8_t A1, uint8_t A2, uint8_t B1, uint8_t B2> class SN754410 : public IODriver {
	int index;
	public:
		static const std::array<uint8_t, 8> cycleInversions;
		//output cycle looks like: (1, 0, 0, 1), (0, 0, 0, 1), (0, 1, 0, 1), (0, 1, 0, 0), (0, 1, 1, 0), (0, 0, 1, 0), (1, 0, 1, 0), (1, 0, 0, 0)
		//This is a form of greycode; only 1 bit is changed between any adjacent steps.
		//pattern is: (#0-#1) -A1, +A2, -B2, +B1, -A2, +A1, -B1, +B2
		//index               000  001  010  011  100  101  110  111
		//shifted             001  010  011  100  101  110  111  000
		//shifted             111  000  001  010  011  100  101  110
		//notice the alternating set, clr, set clr
		SN754410() {
			initIO();
			bcm2835_gpio_fsel(A1, BCM2835_GPIO_FSEL_OUTP); //configure these pins as output
			bcm2835_gpio_fsel(A2, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(B1, BCM2835_GPIO_FSEL_OUTP); 
			bcm2835_gpio_fsel(B2, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_set(A1); //, HIGH); //set to first cycle:  
			bcm2835_gpio_clr(A2); //, LOW); 
			bcm2835_gpio_clr(B1); //, LOW); 
			bcm2835_gpio_set(B2); //, HIGH); 
		}
		
		void stepForward() {
			uint8_t pinSwap = cycleInversions[index];
			index = (index == 7) ? 0 : index+1; //increment index, or wrap around.
			if (index & 1) { //note: new index
				bcm2835_gpio_clr(pinSwap);
			} else { //on steps 1->2, 3->4, 5->6, 7->0: set pin
				bcm2835_gpio_set(pinSwap);
			}
		}
		void stepBackward() {
			index = (index == 0) ? 7 : index-1; //decrement index, or wrap around.
			uint8_t pinSwap = cycleInversions[index];
			if (index & 1) { //note: new index
				bcm2835_gpio_clr(pinSwap);
			} else { //on steps 1->0, 3->2, 5->4, 7->6: set pin
				bcm2835_gpio_set(pinSwap);
			}
		}
};

template <uint8_t A1, uint8_t A2, uint8_t B1, uint8_t B2> const std::array<uint8_t, 8> SN754410<A1, A2, B1, B2>::cycleInversions = {A1, A2, B2, B1, A2, A1, B1, B2};

}
}


#endif
