/**
 * Arduino - Accelerometer driver
 * 
 * Accelerometer.h
 * 
 * The implementation file for the accelerometer driver
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_CPP__
#define __ARDUINO_DRIVER_ACCELEROMETER_CPP__ 1

#include "Accelerometer.h"
    
Accelerometer::Accelerometer(int xPin, int yPin, int zPin) {
    this->xPin = xPin;
    this->yPin = yPin;
    this->zPin = zPin;
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_CPP__ */
