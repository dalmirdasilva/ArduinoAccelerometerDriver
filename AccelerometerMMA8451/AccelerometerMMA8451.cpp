/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerMMA8451.cpp
 * 
 * The implementation of the MMA8451 accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_CPP__
#define __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_CPP__ 1

#include "AccelerometerMMA8451.h"

AccelerometerMMA8451::AccelerometerMMA8451(unsigned char sa0, int sclPin, int sdaPin, int int1Pin, int int2Pin) {
    this->address = 0x1c | (sa0 & 0x01);
    this->sclPin = sclPin;
    this->sdaPin = sdaPin;
    this->int1Pin = int1Pin;
    this->int2Pin = int2Pin;
    Wire.begin();
}

void AccelerometerMMA8451::deviceActivation(DeviceActivation activation) {
    configureRegisterBits(CTRL_REG1, ACTIVE_MASK, activation);
    this->activation = activation;
}

void AccelerometerMMA8451::setDynamicRange(DynamicRange range) {
    standby();
    configureRegisterBits(XYZ_DATA_CFG, FS_MASK, range);
    this->range = range;
    active();
}

void AccelerometerMMA8451::setOutputDataRate(OutputDataRate rate) {
    unsigned char v = (unsigned char ) rate;
    v <<= 3;
    standby();
    configureRegisterBits(CTRL_REG1, ODR_MASK, v);
    active();
}

void AccelerometerMMA8451::setOversamplingMode(OversamplingMode mode) {
    standby();
    configureRegisterBits(CTRL_REG2, MODS_MASK, mode);
    active();
}

void AccelerometerMMA8451::setHighPassFilterCutoffFrequency(HighPassFilterCutoffFrequency frequency) {
    standby();
    configureRegisterBits(HP_FILTER_CUTOFF, SEL_MASK, frequency);
    active();
}

void AccelerometerMMA8451::highPassFilteredData(boolean filtered) {
    unsigned char v = 0x00;
    if (filtered) {
        v = HPF_OUT_MASK;
    }
    configureRegisterBits(XYZ_DATA_CFG, HPF_OUT_MASK, v);
}

boolean AccelerometerMMA8451::isDataReady() {
    return ((readRegister(STATUS) & ZYXDR_MASK) != 0);
}

float AccelerometerMMA8451::readXg() {
    unsigned char buf[2];
    readRegisterBlock(OUT_X_MSB, buf, 2);
    return convertToG(buf);
}

float AccelerometerMMA8451::readYg() {
    unsigned char buf[2];
    readRegisterBlock(OUT_Y_MSB, buf, 2);
    return convertToG(buf);
}

float AccelerometerMMA8451::readZg() {
    unsigned char buf[2];
    readRegisterBlock(OUT_Z_MSB, buf, 2);
    return convertToG(buf);
}

void AccelerometerMMA8451::readXYZ(unsigned char buf[6]) {
    readRegisterBlock(OUT_X_MSB, buf, 6);
}

float AccelerometerMMA8451::convertToG(unsigned char buf[2]) {
    float g = 0.0;
    int aux = 0;
    int frac_mask = 0x3fff >> range;
    aux |= buf[1];
    aux <<= 8;
    aux |= buf[0];
    g += ((buf[1] & 0x70) >> 6 - range);
    g +=  (aux & frac_mask) / (float)(frac_mask + 1);
    if (buf[1] & 0x80) {
        return -(g);
    }
    return g;
}

void AccelerometerMMA8451::configureRegisterBits(Register reg, Mask mask, unsigned char v) {
    unsigned char n;
    n = readRegister(reg);
    n &= ~mask;
    n |= v & mask;
    writeRegister(reg, n);
}

void AccelerometerMMA8451::writeRegister(Register reg, unsigned char v) {
    writeRegisterBlock((unsigned char) reg, &v, 1);
}

unsigned char AccelerometerMMA8451::readRegister(Register reg) {
    unsigned char v;
    readRegisterBlock((unsigned char) reg, &v, 1);
    return v;
}

void AccelerometerMMA8451::writeRegisterBlock(unsigned char to, unsigned char* buf, unsigned char len) {
    Wire.beginTransmission(address);
    Wire.write(to);
    for (int i = 0; i < len; i++) {
        Wire.write(buf[i]);
    }
    Wire.endTransmission();
}

void AccelerometerMMA8451::readRegisterBlock(unsigned char from, unsigned char* buf, unsigned char len) {
    Wire.beginTransmission(address);
    Wire.write(from);
    Wire.endTransmission();
    Wire.requestFrom(address, len);
    for (int i = 0; i < len; i++) {
        while (!Wire.available());
        buf[i] = Wire.read();
    }
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_CPP__ */
