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
    Wire.begin();
    this->address = 0x1d;

    // Set to default values.
    mctl.value = 0x00;
    use8bit = false;
}

void AccelerometerMMA7455::setUse8bit(bool use) {
    use8bit = use;

    // In 10bit mode, only 8g is allowed.
    if (!use8bit) {
        setDynamicRange(DR_8G);
    }
}

void AccelerometerMMA7455::setDeviceMode(DeviceMode mode) {
    configureRegisterBits(MCTL, MCTL_MODE, (unsigned char) mode);
    mctl.MODE = mode;
}

void AccelerometerMMA7455::calibrate0gOffset(unsigned char samples) {

    char counts[] = {16, 64, 32};
    char xyz[3], i = 0;
    unsigned char buf[6];
    int avg[3] = {0}, adjust[3] = {0};

    // For now, only in 8bit mode.
    setUse8bit(true);

    fillRegisterBlock(AccelerometerMMA7455::XOFFL, 0, 6);
    delay(100);

    for (; i < samples; i++) {
        readXYZ((unsigned char *) xyz);
        avg[0] += xyz[0];
        avg[1] += xyz[1];
        avg[2] += xyz[2];
        delay(1);
    }
    
    adjust[0] = -(avg[0] / samples) * 2;
    adjust[1] = -(avg[1] / samples) * 2;
    adjust[2] = -(avg[2] / samples) * 2;
    
    buf[0] = adjust[0] & 0xff;
    buf[1] = (adjust[0] >> 8) & 0xff;
    
    buf[2] = adjust[1] & 0xff;
    buf[3] = (adjust[1] >> 8) & 0xff;
    
    buf[4] = adjust[2] & 0xff;
    buf[5] = (adjust[2] >> 8) & 0xff;
    
    writeRegisterBlock(XOFFL, buf, 6);
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

    // In 2g and 4g, only 8bit mode is allowed.
    if (range != DR_8G) {
        use8bit = true;
    }
    mctl.GLVL = range;
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
    if (is8bit) {
        return (float) ((char) buf[0]) / counts8bit[mctl.GLVL];
    } else {
        int aux = 0;
        if (buf[1] & 0x02) {

            // Expand all left bits to 1 if it is negative.
            buf[1] |= 0xfc;
        }
        aux = buf[1];
        aux <<= 8;
        aux |= buf[0];

        // In 10bit mode, only 8g is allowed.
        return (float) aux / 64.0;
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

void AccelerometerMMA7455::fillRegisterBlock(unsigned char to, unsigned char b, unsigned char len) {
    Wire.beginTransmission(address);
    Wire.write(to);
    for (int i = 0; i < len; i++) {
        Wire.write(b);
    }
    Wire.endTransmission();
}

void AccelerometerMMA7455::readRegisterBlock(unsigned char from, unsigned char* buf, unsigned char len) {
    Wire.beginTransmission(address);
    Wire.write(from);
    Wire.endTransmission(true);
    Wire.requestFrom(address, len);
    for (int i = 0; i < len; i++) {
        while (!Wire.available());
        buf[i] = Wire.read();
    }
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_CPP__ */
