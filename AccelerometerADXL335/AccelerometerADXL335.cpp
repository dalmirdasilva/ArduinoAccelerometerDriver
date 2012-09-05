/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerADXL335.cpp
 * 
 * The implementation of the ADXL335 accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_ADXL335_CPP__
#define __ARDUINO_DRIVER_ACCELEROMETER_ADXL335_CPP__ 1

#include "AccelerometerADXL335.h"
#include <Arduino.h>

AccelerometerADXL335::AccelerometerADXL335(int xPin, int yPin, int zPin) : Accelerometer(xPin, yPin, zPin) {
    arduinoPowerSupply = 5.0;
    sensorPowerSupply = 3.3;
    zeroGBias = sensorPowerSupply / 2;
}

float AccelerometerADXL335::readPin(int pin) {
    float voltage = (analogRead(pin)) * arduinoPowerSupply / 1024.0;
    return ((voltage - zeroGBias) * 1000.0 / 330.0);
}

float AccelerometerADXL335::readXg() {
    return readPin(xPin);
}

float AccelerometerADXL335::readYg() {
    return readPin(yPin);
}

float AccelerometerADXL335::readZg() {
    return readPin(zPin);
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_ADXL335_CPP__ */
