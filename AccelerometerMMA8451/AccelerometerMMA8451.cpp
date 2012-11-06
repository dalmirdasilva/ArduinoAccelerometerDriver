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

AccelerometerMMA8451::AccelerometerMMA8451(bool sa0) {
    this->address = 0x1c | (sa0 & 0x01);
    xyzDataCfg.FS = 0x00;
    ctrlReg1.F_READ = 0;
    Wire.begin();
}

void AccelerometerMMA8451::deviceActivation(DeviceActivation activation) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_ACTIVE, (unsigned char)activation);
}

bool AccelerometerMMA8451::isDataReady() {
    STATUSbits status;
    status.value = readRegister(STATUS);
    return (bool) status.ZYXDR;
}

float AccelerometerMMA8451::readXg() {
    unsigned char size = (ctrlReg1.F_READ) ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(OUT_X_MSB, buf, size);
    return convertToG(buf, size);
}

float AccelerometerMMA8451::readYg() {
    unsigned char size = (ctrlReg1.F_READ) ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(OUT_Y_MSB, buf, size);
    return convertToG(buf, size);
}

float AccelerometerMMA8451::readZg() {
    unsigned char size = (ctrlReg1.F_READ) ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(OUT_Z_MSB, buf, size);
    return convertToG(buf, size);
}

void AccelerometerMMA8451::readXYZ(unsigned char* buf) {
    unsigned char size = (ctrlReg1.F_READ) ? 3 : 6;
    readRegisterBlock(OUT_X_MSB, buf, size);
}

void AccelerometerMMA8451::setDynamicRange(DynamicRange range) {
    configureRegisterBits(XYZ_DATA_CFG, XYZ_DATA_CFG_FS, (unsigned char)range);
    xyzDataCfg.FS = (unsigned char)range;
}

void AccelerometerMMA8451::setOutputDataRate(OutputDataRate rate) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_DR, (unsigned char)rate << 3);
}

void AccelerometerMMA8451::setPortraitLandscapeDetection(bool enable) {
    configureRegisterBits(PL_CFG, PL_CFG_PL_EN, (enable ? PL_CFG_PL_EN : 0x00));
}

void AccelerometerMMA8451::setBackFrontTrip(BackFrontTrip trip) {
    configureRegisterBits(PL_BF_ZCOMP, PL_BF_ZCOMP_BKFR, (unsigned char)trip << 6);
}

void AccelerometerMMA8451::setZLockThresholdAngle(ZLockThresholdAngle angle) {
    configureRegisterBits(PL_BF_ZCOMP, PL_BF_ZCOMP_ZLOCK, (unsigned char)angle);
}

void AccelerometerMMA8451::setPortraitLandscapeThresholdAngle(PortraitLandscapeThresholdAngle angle) {
    configureRegisterBits(P_L_THS_REG, P_L_THS_REG_P_L_THS, (unsigned char)angle << 3);
}

void AccelerometerMMA8451::setHysteresisAngle(HysteresisAngle angle) {
    configureRegisterBits(P_L_THS_REG, P_L_THS_REG_HYS, (unsigned char) angle);
}

void AccelerometerMMA8451::enableInterrupt(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG4, (Mask) interrupt, (unsigned char)interrupt);
}

void AccelerometerMMA8451::disableInterrupt(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG4, (Mask) interrupt, 0);
}

void AccelerometerMMA8451::routeInterruptToInt1(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG5, (Mask) interrupt, (unsigned char)interrupt);
}

void AccelerometerMMA8451::routeInterruptToInt2(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG5, (Mask) interrupt, 0);
}

void AccelerometerMMA8451::setInterruptPolarity(InterruptPolarity polarity) {
    configureRegisterBits(CTRL_REG3, CTRL_REG3_IPOL, (unsigned char)polarity << 1);
}

void AccelerometerMMA8451::setAslpOutputDataRate(AslpOutputDataRate rate) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_ASLP_RATE, (unsigned char)rate << 6);
}

void AccelerometerMMA8451::setReadMode(ReadMode mode) {
    ctrlReg1.F_READ = (unsigned char)mode;
    configureRegisterBits(CTRL_REG1, CTRL_REG1_F_READ, (unsigned char)mode << 1);
}

void AccelerometerMMA8451::setOversamplingMode(OversamplingMode mode) {
    configureRegisterBits(CTRL_REG2, CTRL_REG2_MODS, (unsigned char)mode);
}

void AccelerometerMMA8451::setHighPassFilterCutoffFrequency(HighPassFilterCutoffFrequency frequency) {
    configureRegisterBits(HP_FILTER_CUTOFF, HP_FILTER_CUTOFF_SEL, frequency);
}

void AccelerometerMMA8451::highPassFilteredData(bool filtered) {
    Mask m;
    if (filtered) {
        m = XYZ_DATA_CFG_HPF_OUT;
    }
    configureRegisterBits(XYZ_DATA_CFG, XYZ_DATA_CFG_HPF_OUT, (unsigned char) m);
}

void AccelerometerMMA8451::setFifoBufferOverflowMode(FifoBufferOverflowMode mode) {
    configureRegisterBits(F_SETUP, F_SETUP_F_MODE, (unsigned char) FIFO_DISABLED);
    configureRegisterBits(F_SETUP, F_SETUP_F_MODE, (unsigned char)mode << 6);
}

void AccelerometerMMA8451::setFifoWatermark(unsigned char watermark) {
    configureRegisterBits(F_SETUP, F_SETUP_F_WMRK, watermark);
}

bool AccelerometerMMA8451::getFifoGateError() {
    SYSMODbits sysmod;
    sysmod.value = readRegister(SYSMOD);
    return (bool) sysmod.FGERR;
}

unsigned char AccelerometerMMA8451::getFifoFgt() {
    SYSMODbits sysmod;
    sysmod.value = readRegister(SYSMOD);
    return (unsigned char) sysmod.FGT;
}

unsigned char AccelerometerMMA8451::getSysmod() {
    SYSMODbits sysmod;
    sysmod.value = readRegister(SYSMOD);
    return (unsigned char) sysmod.SYSMOD;
}

float AccelerometerMMA8451::convertToG(unsigned char* buf, unsigned char len) {
    unsigned char negative = 0;
    unsigned char shift = 14;
    float g = 0.0;
    int mantissaMask = 0x3fff;
    int integerMask = 0xf000;
    int aux = 0;
    int mantissaMax;
    aux |= buf[0];
    if (len == 1) {
        mantissaMask >>= 8;
        integerMask >>= 8;
        shift -= 8;
    } else {
        aux <<= 8;
        aux |= buf[1];
    }
    mantissaMax = mantissaMask >> (unsigned char)xyzDataCfg.FS;
    if (buf[0] & 0x80) {
      aux = ~aux + 1;
      negative = 1;
    }
    g += ((aux & integerMask) >> shift - (unsigned char)xyzDataCfg.FS);
    g +=  (aux & mantissaMax) / (float)(mantissaMax + 1);
    if (negative) {
        return -(g);
    }
    return g;
}

void AccelerometerMMA8451::configureRegisterBits(Location location, Mask mask, unsigned char v) {
    unsigned char n;
    n = readRegister(location);
    n &= ~((unsigned char)mask);
    n |= v & ((unsigned char)mask);
    writeRegister(location, n);
}

void AccelerometerMMA8451::setPushPullOpenDrain(PushPullOpenDrain ppod) {
    configureRegisterBits(CTRL_REG3, CTRL_REG3_PP_OD, (unsigned char)ppod << 1);
}

void AccelerometerMMA8451::writeRegister(Location location, unsigned char v) {
    writeRegisterBlock((unsigned char) location, &v, 1);
}

unsigned char AccelerometerMMA8451::readRegister(Location location) {
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
    Wire.endTransmission(false);
    Wire.requestFrom(address, len);
    for (int i = 0; i < len; i++) {
        while (!Wire.available());
        buf[i] = Wire.read();
    }
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_CPP__ */
