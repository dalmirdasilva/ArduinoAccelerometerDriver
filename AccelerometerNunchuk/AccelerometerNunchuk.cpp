/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerNunchuk.cpp
 * 
 * The implementation of the NUNCHUK accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_NUNCHUK_CPP__
#define __ARDUINO_DRIVER_ACCELEROMETER_NUNCHUK_CPP__ 1

#include "AccelerometerNunchuk.h"

AccelerometerNunchuk::AccelerometerNunchuk() {
    Wire.begin();
    address = 0x52;
    initializationSequence[0] = 0xf0;
    initializationSequence[1] = 0xaa;
}

float AccelerometerNunchuk::readXg(bool updateFrame) {
    return convertToG(readAcceleration(AXIS_X, updateFrame));
}

float AccelerometerNunchuk::readYg(bool updateFrame) {
    return convertToG(readAcceleration(AXIS_Y, updateFrame));
}

float AccelerometerNunchuk::readZg(bool updateFrame) {
    return convertToG(readAcceleration(AXIS_Z, updateFrame));
}

bool AccelerometerNunchuk::readZButton(bool updateFrame) {
    if (updateFrame) {
        readFrame();
    }
    return (frame[5] & 0x01) == 0;
}

bool AccelerometerNunchuk::readCButton(bool updateFrame) {
    if (updateFrame) {
        readFrame();
    }
    return (frame[5] & 0x02) == 0;
}

unsigned char AccelerometerNunchuk::readXJoystick(bool updateFrame) {
    if (updateFrame) {
        readFrame();
    }
    return frame[0];
}

unsigned char AccelerometerNunchuk::readYJoystick(bool updateFrame) {
    if (updateFrame) {
        readFrame();
    }
    return frame[1];
}

void AccelerometerNunchuk::begin() {
    Wire.beginTransmission(address);
    for (unsigned char i = 0; i < sizeof (initializationSequence); i++) {
        Wire.write(initializationSequence[i]);
    }
    Wire.endTransmission();
}

unsigned int AccelerometerNunchuk::readAcceleration(Axis axis, bool updateFrame) {
    if (updateFrame) {
        readFrame();
    }
    unsigned int aux = 0x0000;
    aux |= frame[2 + axis];
    aux <<= 2;
    aux |= ((frame[5] >> 2) + (2 * axis)) & 0x03;
    return aux;
}

void AccelerometerNunchuk::readFrame() {
    Wire.beginTransmission(address);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(1);
    Wire.requestFrom(address, (unsigned char) 6);
    for (int i = 0; i < 6; i++) {
        while (!Wire.available());
        frame[i] = decode(Wire.read());
    }
}

unsigned char AccelerometerNunchuk::decode(unsigned char b) {
    return (b ^ 0x17) + 0x17;
}
    
float AccelerometerNunchuk::convertToG(unsigned int i) {
    int aux = i & 0x03ff;
    aux -= 512;
    return (float) aux / 256.0;
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_NUNCHUK_CPP__ */
