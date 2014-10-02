#ifndef DRIVERS_RPI_MITPI_H
#define DRIVERS_RPI_MITPI_H

#include <stdint.h> //for uint32_t


namespace mitpi {

volatile uint32_t* mapPeripheral(int memfd, int addr);

void init();

void makeOutput(int pin);
void makeInput(int pin);
void setPinHigh(int pin);
void setPinLow(int pin);
void setPinState(int pin, int state);
bool readPinState(int pin);

}


#endif
