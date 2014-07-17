#ifndef DRIVERS_RPI_RCTHERMISTOR_H
#define DRIVERS_RPI_RCTHERMISTOR_H

/* This file provides code to approximate a temperature via first determining the resistance of a thermistor (resistor that varies its resistance according to temperature) via only a digital IO pin.
 * The raspberry pi doesn't have any ADC pins, so we must use the method outlined here: http://www.robotshop.com/media/files/pdf/RCtime_App_Note.pdf
 */
 
#include "drivers/rpi/rpi.h"

#include <time.h> //for timespec
#include "timeutil.h" //for timespecSub, etc
#include "logging.h"

namespace drv {
namespace rpi {

template <uint8_t PIN, unsigned R_OHMS, unsigned C_PICO> class RCThermistor {
	public:
		RCThermistor() {
			initIO();
		}
		float readTemperature() const {
			struct timespec t1, t2, tDelta;
			bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_set(PIN); //output high to drain the capacitor
			bcm2835_delayMicroseconds(1000);
			bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
			
			clock_gettime(CLOCK_MONOTONIC, &t1);
			do {} while (bcm2835_gpio_lev(PIN));
			clock_gettime(CLOCK_MONOTONIC, &t2);
			tDelta = timespecSub(t1, t2);
			float time = timespecToFloat(tDelta);
			LOG("time to read resistor: %f\n", time);
			return time;
		}
};



}
}

#endif
