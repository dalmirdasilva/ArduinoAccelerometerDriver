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
    configureRegisterBits(Register::Location::CTRL_REG1, Register::Mask::ACTIVE_MASK, (unsigned char) activation);
}

bool AccelerometerMMA8451::isDataReady() {
    Register::STATUSbits status = (Register::STATUSbits) readRegister(Register::Location::STATUS);
    return (bool) status.ZYXDR;
}

float AccelerometerMMA8451::readXg() {
    unsigned char buf[2];
    readRegisterBlock(Register::Location::OUT_X_MSB, buf, 2);
    return convertToG(buf);
}

float AccelerometerMMA8451::readYg() {
    unsigned char buf[2];
    readRegisterBlock(Register::Location::OUT_Y_MSB, buf, 2);
    return convertToG(buf);
}

float AccelerometerMMA8451::readZg() {
    unsigned char buf[2];
    readRegisterBlock(Register::Location::OUT_Z_MSB, buf, 2);
    return convertToG(buf);
}

void inline AccelerometerMMA8451::readXYZ(unsigned char buf[6]) {
    readRegisterBlock(Register::Location::OUT_X_MSB, buf, 6);
}

void AccelerometerMMA8451::setDynamicRange(DynamicRange range) {
    configureRegisterBits(Register::Location::XYZ_DATA_CFG, 
            Register::Mask::XYZ_DATA_CFG_FS, (unsigned char) range);
    xyzDataCfg.FS = (unsigned char) range;
}

// AN4068
void inline AccelerometerMMA8451::setOutputDataRate(OutputDataRate rate) {
    configureRegisterBits(Register::Location::CTRL_REG1, 
            Register::Mask::CTRL_REG1_DR, ((unsigned char)rate << 3));
}

// AN4068
void inline AccelerometerMMA8451::setPortraitLandscapeDetection(bool enable) {
    configureRegisterBits(Register::Location::PL_CFG, 
            Register::Mask::PL_CFG_PL_EN, (enable ? Register::Mask::PL_CFG_PL_EN : 0x00));
}

// AN4068
void inline AccelerometerMMA8451::setBackFrontTrip(BackFrontTrip trip) {
    configureRegisterBits(Register::Location::PL_BF_ZCOMP, 
            Register::Mask::PL_BF_ZCOMP_BKFR, ((unsigned char) trip << 6));
}

// AN4068
void inline AccelerometerMMA8451::setZLockThresholdAngle(ZLockThresholdAngle angle) {
    configureRegisterBits(Register::Location::PL_BF_ZCOMP, 
            Register::Mask::PL_BF_ZCOMP_ZLOCK, (unsigned char) angle);
}

// AN4068
void inline AccelerometerMMA8451::setPortraitLandscapeThresholdAngle(unsigned char angle) {
    configureRegisterBits(Register::Location::P_L_THS_REG, 
            Register::Mask::P_L_THS_REG_P_L_THS, ((unsigned char) angle << 3));
}

// AN4068
void inline AccelerometerMMA8451::setHysteresisAngle(HysteresisAngle angle) {
    configureRegisterBits(Register::Location::P_L_THS_REG, 
            Register::Mask::P_L_THS_REG_HYS, (unsigned char) angle);
}

// AN4068
void inline AccelerometerMMA8451::enableInterrupt(Interrupt interrupt) {
    configureRegisterBits(Register::Location::CTRL_REG4, 
            (Register::Mask) interrupt, (unsigned char) interrupt);
}

// AN4068
void inline AccelerometerMMA8451::disableInterrupt(Interrupt interrupt) {
    configureRegisterBits(Register::Location::CTRL_REG4, 
            (Register::Mask) interrupt, 0);
}

// AN4068
void inline AccelerometerMMA8451::routeInterruptToInt1(Interrupt interrupt) {
    configureRegisterBits(Register::Location::CTRL_REG5, 
            (Register::Mask) interrupt, (unsigned char) interrupt);
}

// AN4068
void inline AccelerometerMMA8451::routeInterruptToInt2(Interrupt interrupt) {
    configureRegisterBits(Register::Location::CTRL_REG5, 
            (Register::Mask) interrupt, 0);
}

void inline AccelerometerMMA8451::setAslpOutputDataRate(AslpOutputDataRate rate) {
    configureRegisterBits(Register::Location::CTRL_REG1, 
            Register::Mask::CTRL_REG1_ASLP_RATE, ((unsigned char)rate << 6));
}

void inline AccelerometerMMA8451::setOversamplingMode(OversamplingMode mode) {
    configureRegisterBits(Register::Location::CTRL_REG2, 
            Register::Mask::CTRL_REG2_MODS, (unsigned char) mode);
}

void inline AccelerometerMMA8451::setHighPassFilterCutoffFrequency(HighPassFilterCutoffFrequency frequency) {
    configureRegisterBits(Register::Location::HP_FILTER_CUTOFF, 
            Register::Mask::HP_FILTER_CUTOFF_SEL, frequency);
}

void AccelerometerMMA8451::highPassFilteredData(bool filtered) {
    Register::Mask m = 0x00;
    if (filtered) {
        m = Register::Mask::XYZ_DATA_CFG_HPF_OUT;
    }
    configureRegisterBits(Register::Location::XYZ_DATA_CFG, 
            Register::Mask::XYZ_DATA_CFG_HPF_OUT, (unsigned char) m);
}

void AccelerometerMMA8451::setFifoBufferOverflowMode(FifoBufferOverflowMode mode) {
    configureRegisterBits(Register::Location::F_SETUP, 
            Register::Mask::F_SETUP_F_MODE, (unsigned char) FIFO_DISABLED);
    configureRegisterBits(Register::Location::F_SETUP, 
            Register::Mask::F_SETUP_F_MODE, (unsigned char)(mode << 6));
}

void AccelerometerMMA8451::setFifoWatermark(unsigned char watermark) {
    configureRegisterBits(Register::Location::F_SETUP, 
            Register::Mask::F_SETUP_F_WMRK, watermark);
}

bool AccelerometerMMA8451::getFifoGateError() {
    Register::SYSMODbits sysmod = (Register::SYSMODbits) readRegister(Register::Location::SYSMOD);
    return (bool) sysmod.FGERR;
}

unsigned char AccelerometerMMA8451::getFifoFgt() {
    Register::SYSMODbits sysmod = (Register::SYSMODbits) readRegister(Register::Location::SYSMOD);
    return (unsigned char) sysmod.FGT;
}

unsigned char AccelerometerMMA8451::getSysmod() {
    Register::SYSMODbits sysmod = (Register::SYSMODbits) readRegister(Register::Location::SYSMOD);
    return (unsigned char) sysmod.SYSMOD;
}

float AccelerometerMMA8451::convertToG(unsigned char buf[2]) {
    // Needs refactoring
    float g = 0.0;
    int aux = 0;
    int frac_max = 0x3fff >> ((unsigned char)xyzDataCfg.FS);
    aux |= buf[1];
    aux <<= 8;
    aux |= buf[0];
    g += ((buf[1] & 0x70) >> 6 - ((unsigned char)xyzDataCfg.FS));
    g +=  (aux & frac_max) / (float)(frac_max + 1);
    if (buf[1] & 0x80) {
        return -(g);
    }
    return g;
}

void AccelerometerMMA8451::configureRegisterBits(Register::Location location, Register::Mask mask, unsigned char v) {
    unsigned char n;
    n = readRegister(location);
    n &= ~((unsigned char)mask);
    n |= v & ((unsigned char)mask);
    writeRegister(location, n);
}

void AccelerometerMMA8451::writeRegister(Register::Location location, unsigned char v) {
    writeRegisterBlock((unsigned char) location, &v, 1);
}

unsigned char AccelerometerMMA8451::readRegister(Register::Location location) {
    unsigned char v;
    readRegisterBlock((unsigned char) location, &v, 1);
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
