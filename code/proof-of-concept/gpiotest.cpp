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
*/

#include <wiringPi.h>



int main() {
	if (wiringPiSetup () == -1) {
		return 1;
    }
	return 0;
}
