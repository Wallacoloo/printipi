#ifndef IODRIVERS_SERVO_H
#define IODRIVERS_SERVO_H

#include "iodriver.h"
#include "iopin.h"

namespace iodrv {

//Controls a Servo motor.
//
//Servos are controlled by periodically sending a pulse of a specified length.
//The length of that pulse determines the position at which the servo should be placed, and the servo will attempt to stay at that location until the next command.
//Typical pulse length varies from 1ms to 2ms for the full control range,
// while the pulses must occur between 40-200 times per second.
//
//See discussion @ https://github.com/Wallacoloo/printipi/issues/62
class Servo : public IODriver {

};

}

#endif