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
#include <RegisterBasedWiredDevice.h>

AccelerometerMMA8451::AccelerometerMMA8451(bool sa0)
        : RegisterBasedWiredDevice(MMA8451_ADDRESS | (sa0 & 0x01)), lastError(NO_ERROR) {
    xyzDataCfg.FS = 0x00;
    ctrlReg1.F_READ = 0;
}

void AccelerometerMMA8451::deviceActivation(DeviceActivation activation) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_ACTIVE, (unsigned char) activation);
}

bool AccelerometerMMA8451::isDataReady() {
    STATUSbits status = { 0 };
    int v = readRegister(STATUS);
    if (v < 0) {
        lastError = (CommunicationError) -(v);
        return false;
    }
    status.value = v & 0xff;
    return (bool) status.ZYXDR;
}

float AccelerometerMMA8451::readAxisGravity(unsigned char axisRegister) {
    bool fastRead = (bool) ctrlReg1.F_READ;
    unsigned char size = fastRead ? 1 : 2;
    unsigned char buf[size];
    readRegisterBlock(axisRegister, buf, size);
    return convertToG(buf, fastRead);
}

float AccelerometerMMA8451::readXg() {
    return readAxisGravity(OUT_X_MSB);
}

float AccelerometerMMA8451::readYg() {
    return readAxisGravity(OUT_Y_MSB);
}

float AccelerometerMMA8451::readZg() {
    return readAxisGravity(OUT_Z_MSB);
}

void AccelerometerMMA8451::readXYZ(unsigned char* buf) {
    bool fastRead = (bool) ctrlReg1.F_READ;
    unsigned char size = fastRead ? 3 : 6;
    readRegisterBlock(OUT_X_MSB, buf, size);
}

void AccelerometerMMA8451::setDynamicRange(DynamicRange range) {
    configureRegisterBits(XYZ_DATA_CFG, XYZ_DATA_CFG_FS, (unsigned char) range);
    xyzDataCfg.FS = (unsigned char) range;
}

void AccelerometerMMA8451::setOutputDataRate(OutputDataRate rate) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_DR, (unsigned char) rate << 3);
}

void AccelerometerMMA8451::setPortraitLandscapeDetection(bool enable) {
    configureRegisterBits(PL_CFG, PL_CFG_PL_EN, (enable ? PL_CFG_PL_EN : 0x00));
}

void AccelerometerMMA8451::setTransientDetection(bool enable, unsigned char axis, bool bypass) {
    TRANSIENT_CFGbits config = { 0 };
    config.ELE = enable;
    config.TEFE = axis & TRANSIENT_CFG_TEFE;
    config.HPF_BYP = bypass;
    writeRegister(TRANSIENT_CFG, config.value);
}

void AccelerometerMMA8451::setTransientThreshold(bool debounceCounterMode, unsigned char threshold) {
    TRANSIENT_THSbits config = { 0 };
    config.THS = threshold & TRANSIENT_THS_THS;
    config.DBCNTM = debounceCounterMode & 0x01;
    writeRegister(TRANSIENT_THS, config.value);
}

void AccelerometerMMA8451::setTransientCount(unsigned char count) {
    writeRegister(TRANSIENT_COUNT, count);
}

void AccelerometerMMA8451::setMotionDetectionDetection(bool ele, bool oae, unsigned char axis) {
    FF_MT_CFGbits config = { 0 };
    config.ELE = ele;
    config.EFE = axis & FF_MT_CFG_EFE;
    config.OAE = oae;
    writeRegister(FF_MT_CFG, config.value);
}

void AccelerometerMMA8451::setMotionDetectionThreshold(bool debounceCounterMode, unsigned char threshold) {
    FF_MT_THSbits config = { 0 };
    config.THS = threshold & FF_MT_THS_THS;
    config.DBCNTM = debounceCounterMode & 0x01;
    writeRegister(FF_MT_THS, config.value);
}

void AccelerometerMMA8451::setMotionDetectionCount(unsigned char count) {
    writeRegister(FF_MT_COUNT, count);
}

void AccelerometerMMA8451::setBackFrontTrip(BackFrontTrip trip) {
    configureRegisterBits(PL_BF_ZCOMP, PL_BF_ZCOMP_BKFR, (unsigned char) trip << 6);
}

void AccelerometerMMA8451::setZLockThresholdAngle(ZLockThresholdAngle angle) {
    configureRegisterBits(PL_BF_ZCOMP, PL_BF_ZCOMP_ZLOCK, (unsigned char) angle);
}

void AccelerometerMMA8451::setPortraitLandscapeThresholdAngle(PortraitLandscapeThresholdAngle angle) {
    configureRegisterBits(P_L_THS_REG, P_L_THS_REG_P_L_THS, (unsigned char) angle << 3);
}

void AccelerometerMMA8451::setHysteresisAngle(HysteresisAngle angle) {
    configureRegisterBits(P_L_THS_REG, P_L_THS_REG_HYS, (unsigned char) angle);
}

void AccelerometerMMA8451::enableInterrupt(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG4, (Mask) interrupt, (unsigned char) interrupt);
}

void AccelerometerMMA8451::enableInterrupt(Interrupt interrupt, unsigned char routePin) {
    enableInterrupt(interrupt);
    routeInterrupt(interrupt, routePin);
}

void AccelerometerMMA8451::disableInterrupt(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG4, (Mask) interrupt, 0);
}

void AccelerometerMMA8451::routeInterruptToInt1(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG5, (Mask) interrupt, (unsigned char) interrupt);
}

void AccelerometerMMA8451::routeInterruptToInt2(Interrupt interrupt) {
    configureRegisterBits(CTRL_REG5, (Mask) interrupt, 0);
}

void AccelerometerMMA8451::routeInterrupt(Interrupt interrupt, unsigned char routePin) {
    configureRegisterBits(CTRL_REG5, (Mask) interrupt, (routePin == 1) ? interrupt : 0);
}

void AccelerometerMMA8451::setInterruptPolarity(InterruptPolarity polarity) {
    configureRegisterBits(CTRL_REG3, CTRL_REG3_IPOL, (unsigned char) polarity << 1);
}

void AccelerometerMMA8451::setAslpOutputDataRate(AslpOutputDataRate rate) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_ASLP_RATE, (unsigned char) rate << 6);
}

void AccelerometerMMA8451::setReadMode(ReadMode mode) {
    configureRegisterBits(CTRL_REG1, CTRL_REG1_F_READ, (unsigned char) mode << 1);
    ctrlReg1.F_READ = (unsigned char) mode;
}

void AccelerometerMMA8451::setOversamplingMode(OversamplingMode mode) {
    configureRegisterBits(CTRL_REG2, CTRL_REG2_MODS, (unsigned char) mode);
}

void AccelerometerMMA8451::setHighPassFilterCutoffFrequency(HighPassFilterCutoffFrequency frequency) {
    configureRegisterBits(HP_FILTER_CUTOFF, HP_FILTER_CUTOFF_SEL, frequency);
}

void AccelerometerMMA8451::highPassFilteredData(bool filtered) {
    unsigned char v;
    if (filtered) {
        v = 0xff;
    }
    configureRegisterBits(XYZ_DATA_CFG, XYZ_DATA_CFG_HPF_OUT, v);
}

void AccelerometerMMA8451::setFifoBufferOverflowMode(FifoBufferOverflowMode mode) {
    configureRegisterBits(F_SETUP, F_SETUP_F_MODE, (unsigned char) FIFO_DISABLED);
    configureRegisterBits(F_SETUP, F_SETUP_F_MODE, (unsigned char) mode << 6);
}

void AccelerometerMMA8451::setFifoWatermark(unsigned char watermark) {
    configureRegisterBits(F_SETUP, F_SETUP_F_WMRK, watermark);
}

bool AccelerometerMMA8451::getFifoGateError() {
    SYSMODbits sysmod = { 0 };
    sysmod.value = readRegister(SYSMOD);
    return (bool) sysmod.FGERR;
}

unsigned char AccelerometerMMA8451::getFifoFgt() {
    SYSMODbits sysmod = { 0 };
    sysmod.value = readRegister(SYSMOD);
    return (unsigned char) sysmod.FGT;
}

unsigned char AccelerometerMMA8451::getSysmod() {
    SYSMODbits sysmod = { 0 };
    sysmod.value = readRegister(SYSMOD);
    return (unsigned char) sysmod.SYSMOD;
}

float AccelerometerMMA8451::convertToG(unsigned char* buf, bool fastRead) {
    float g;
    if (fastRead) {
        float counts[] = { 64.0, 32.0, 16.0 };
        g = ((char) buf[0]) / counts[xyzDataCfg.FS];
    } else {
        int aux = 0;
        float counts[] = { 16384.0, 8192.0, 4096.0 };
        aux = buf[0];
        aux <<= 8;
        aux |= buf[1];
        g = aux / counts[xyzDataCfg.FS];
    }
    return g;
}

float AccelerometerMMA8451::convertToG(unsigned char* buf) {
    return convertToG(buf, ctrlReg1.F_READ);
}

void AccelerometerMMA8451::setPushPullOpenDrain(PushPullOpenDrain ppod) {
    configureRegisterBits(CTRL_REG3, CTRL_REG3_PP_OD, (unsigned char) ppod << 1);
}

AccelerometerMMA8451::CommunicationError AccelerometerMMA8451::gerLastError() {
    return lastError;
}

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_CPP__ */
