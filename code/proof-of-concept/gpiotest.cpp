/*
Can use WiringPi (Arduino-like) library: https://projects.drogon.net/raspberry-pi/wiringpi/
Claims that UART is only accessible as GPIO when booting with serial console disabled (allowing for the full 17 GPIOs, not 15).
After install WiringPi, can begin with the `gpio readall` command.
  inputs are responsive to my fingers.
  gpio mode 0 out
  gpio write 0 1 #output GPIO 0 high. Works.
WiringPi simple example: http://elinux.org/RPi_Low-level_peripherals#C_.2B_wiringPi
  
To install wiringPi:
git clone git://git.drogon.net/wiringPi
cd wiringPi
git pull origin
./build #will also install it.

IO benchmarks: http://codeandlife.com/2012/07/03/benchmarking-raspberry-pi-gpio-speed/
  wiringPi can toggle pins at 7 MHz; Should be plenty fast enough, and hardware limit is 21 MHz.
  
WiringPi functions doc: https://projects.drogon.net/raspberry-pi/wiringpi/functions/
  piHiPri(...) can change the program priority (niceness). Must be root.
*/

#include <wiringPi.h>



int main() {
	if (wiringPiSetup() == -1) {
		return 1;
    }
    pinMode(0, OUTPUT);
    digitalWrite(0, 1); //set GPIO 1 (BCM 17) to output HIGH
	return 0;
}
