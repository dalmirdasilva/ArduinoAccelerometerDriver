/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerMMA7455.cpp
 * 
 * The implementation of the MMA7455 accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_CPP__
#define __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_CPP__ 1

#include "AccelerometerMMA7455.h"

AccelerometerMMA7455::AccelerometerMMA7455() {
    this->address = 0x1d;
    mctl.value = 0x00;
    use8bit = false;
    Wire.begin();
}

void AccelerometerMMA7455::setUse8bit(bool use) {
    use8bit = use;
    if (!use8bit) {
        setDynamicRange(DR_8G);
    }
}

void AccelerometerMMA7455::setDeviceMode(DeviceMode mode) {
    configureRegisterBits(MCTL, MCTL_MODE, (unsigned char) mode);
    mctl.MODE = mode;
}

bool AccelerometerMMA7455::isDataReady() {
    STATUSbits status;
    status.value = readRegister(STATUS);
    return (bool) status.DRDY;
}

float AccelerometerMMA7455::readXg() {
    Location location = use8bit ? XOUT8 : XOUTL;
    unsigned char size = use8bit ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(location, buf, size);
    return convertToG(buf, use8bit);
}

float AccelerometerMMA7455::readYg() {
    Location location = use8bit ? YOUT8 : YOUTL;
    unsigned char size = use8bit ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(location, buf, size);
    return convertToG(buf, use8bit);
}

float AccelerometerMMA7455::readZg() {
    Location location = use8bit ? ZOUT8 : ZOUTL;
    unsigned char size = use8bit ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(location, buf, size);
    return convertToG(buf, use8bit);
}

void AccelerometerMMA7455::readXYZ(unsigned char* buf) {
    Location location = use8bit ? XOUT8 : XOUTL;
    unsigned char size = use8bit ? 3 : 6;
    readRegisterBlock(location, buf, size);
}

void AccelerometerMMA7455::setDetectionCondition(DetectionCondition condition) {
    configureRegisterBits(CTL2, CTL2_LDPL, (unsigned char) condition);
}

void AccelerometerMMA7455::setDynamicRange(DynamicRange range) {
    configureRegisterBits(MCTL, MCTL_GLVL, (unsigned char) range << 2);
    mctl.GLVL = range;
    if (mctl.GLVL != DR_8G) {
        use8bit = true;
    }
}

void AccelerometerMMA7455::enableInterrupt(Axis axis) {
    Mask mask = CTL1_XDA;
    if (axis == AXIS_Y) {
        mask = CTL1_YDA;
    } else if (axis == AXIS_Z) {
        mask = CTL1_YDA;
    }
    configureRegisterBits(CTL1, mask, (unsigned char) 0xff);
}

void AccelerometerMMA7455::disableInterrupt(Axis axis) {
    Mask mask = CTL1_XDA;
    if (axis == AXIS_Y) {
        mask = CTL1_YDA;
    } else if (axis == AXIS_Z) {
        mask = CTL1_YDA;
    }
    configureRegisterBits(CTL1, mask, (unsigned char) 0x00);
}

void AccelerometerMMA7455::setInterruptConfiguration(InterruptConfiguration configuration) {
    configureRegisterBits(CTL1, CTL1_INTREG, (unsigned char) configuration << 2);
}

float AccelerometerMMA7455::convertToG(unsigned char* buf, bool is8bit) {
    float counts8bit[] = {16.0, 64.0, 32.0};
    float counts10bit[] = {64.0, 256.0, 128.0};
    if (is8bit) {
        return (float) ((char) buf[0]) / counts8bit[mctl.GLVL & 0x03];
    } else {
        int aux = 0;
        if (buf[1] & 0x02) {
            buf[1] |= 0xfc;
        }
        aux = buf[1];
        aux <<= 8;
        aux |= buf[0];
        return (float) aux / counts10bit[mctl.GLVL & 0x03];
    }
}

void AccelerometerMMA7455::configureRegisterBits(Location location, Mask mask, unsigned char v) {
    unsigned char n;
    n = readRegister(location);
    n &= ~((unsigned char) mask);
    n |= v & ((unsigned char) mask);
    writeRegister(location, n);
}

void AccelerometerMMA7455::writeRegister(Location location, unsigned char v) {
    writeRegisterBlock((unsigned char) location, &v, 1);
}

unsigned char AccelerometerMMA7455::readRegister(Location location) {
    unsigned char v;
    readRegisterBlock((unsigned char) location, &v, 1);
    return v;
}

void AccelerometerMMA7455::writeRegisterBlock(unsigned char to, unsigned char* buf, unsigned char len) {
    Wire.beginTransmission(address);
    Wire.write(to);
    for (int i = 0; i < len; i++) {
        Wire.write(buf[i]);
    }
    Wire.endTransmission();
}

void AccelerometerMMA7455::readRegisterBlock(unsigned char from, unsigned char* buf, unsigned char len) {
    Wire.beginTransmission(address);
    Wire.write(from);
    Wire.endTransmission(false);
    Wire.requestFrom(address, len);
    for (int i = 0; i < len; i++) {
        while (!Wire.available());
        buf[i] = Wire.read();
    }
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_CPP__ */
