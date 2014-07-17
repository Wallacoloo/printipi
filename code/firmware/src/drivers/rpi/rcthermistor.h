#ifndef DRIVERS_RPI_RCTHERMISTOR_H
#define DRIVERS_RPI_RCTHERMISTOR_H

/* This file provides code to approximate a temperature via first determining the resistance of a thermistor (resistor that varies its resistance according to temperature) via only a digital IO pin.
 * The raspberry pi doesn't have any ADC pins, so we must use the method outlined here: http://www.robotshop.com/media/files/pdf/RCtime_App_Note.pdf
 */
 
#include "drivers/rpi/rpi.h"

#include <time.h> //for timespec
#include <cmath>
#include "timeutil.h" //for timespecSub, etc
#include "logging.h"

namespace drv {
namespace rpi {

template <uint8_t PIN, unsigned R_OHMS, unsigned C_PICO> class RCThermistor {
	static constexpr float C = C_PICO * 1.0e-12;
	static constexpr float Vcc = 3.3;
	static constexpr float Va = 1.65;
	static constexpr float Ra = R_OHMS;
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
			tDelta = timespecSub(t2, t1);
			float duration = timespecToFloat(tDelta);
			LOG("time to read resistor: %f\n", duration);
			//now try to guess the resistance:
			float resistance = guessRFromTime(duration);
			LOG("Resistance guess: %f\n", resistance);
			return resistance;
			//return time;
		}
		float guessRFromTime(float time) const {
			//equation is: Va = Vcc (1 - Ra/(Ra+Rt)) e^-t/(Rt C)
			//where Va is the minimum voltage sensed as HIGH,
			//  Ra is resistance connecting IO pin to cap,
			//  Rt is resistance of thermistor
			//  C is value of capacitor.
			//equation cannot be solved for Rt symbolically. But it can be solved for t:
			//t = C*Rt*ln(Rt*Vcc/ ((Ra+Rt)*Va));
			//do a binary search for the value of Rt by judging to proximity to t.
			//if the calculated t is < measured t, then Rt is too low. else too high.
			float lower = 0;
			float upper = 100000;
			while (upper-lower > 2) {
				float Rt = 0.5*(upper+lower);
				float calcT = C*Rt*log(Rt*Vcc / ((Ra+Rt)*Va));
				if (calcT < time) {
					lower = Rt;
				} else {
					upper = Rt;
				}
			}
			return 0.5*(lower+upper);
		}
};



}
}

#endif
