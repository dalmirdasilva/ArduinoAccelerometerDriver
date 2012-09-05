/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerVirtual.cpp
 * 
 * Virtual implementation of the accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_VIRTUAL_CPP__
#define __ARDUINO_DRIVER_ACCELEROMETER_VIRTUAL_CPP__ 1

#include "AccelerometerVirtual.h"

AccelerometerVirtual::AccelerometerVirtual() : Accelerometer(0, 0, 0) {
}

float AccelerometerVirtual::readXg() {
    return 0.0;
}

float AccelerometerVirtual::readYg() {
    return 0.0;
}

float AccelerometerVirtual::readZg() {
    return 0.0;
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_VIRTUAL_CPP__ */
