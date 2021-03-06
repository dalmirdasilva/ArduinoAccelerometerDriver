#include "AccelerometerMPU9250.h"

AccelerometerMPU9250::AccelerometerMPU9250(bool ad0)
        : RegisterBasedWiredDevice(MPU9250_ADDRESS | (ad0 & 0x01)) {
    config.value = 0x00;
}

float AccelerometerMPU9250::readAxisGravity(unsigned char axisRegister) {
    unsigned char buf[2];
    if (readRegisterBlock(axisRegister, buf, 2) != 2) {
        return 0.0;
    }
    return convertToG(buf);
}

float AccelerometerMPU9250::readXg() {
    return readAxisGravity(ACCEL_XOUT_H);
}

float AccelerometerMPU9250::readYg() {
    return readAxisGravity(ACCEL_YOUT_H);
}

float AccelerometerMPU9250::readZg() {
    return readAxisGravity(ACCEL_ZOUT_H);
}

void AccelerometerMPU9250::readXYZ(unsigned char* buf) {
    readRegisterBlock(ACCEL_XOUT_H, buf, 6);
}

void AccelerometerMPU9250::setFullScaleRange(FullScaleRange fsr) {
    configureRegisterBits(ACCEL_CONFIG, ACCEL_CONFIG_ACCEL_FS_SEL, (unsigned char) fsr);
    config.ACCEL_FS_SEL = fsr >> 3;
}

void AccelerometerMPU9250::setLowPassFilter(bool enable, LowPassFilter lpf) {
    unsigned char v = enable ? 0x08 : 0x00;
    v |= (lpf & ACCEL_CONFIG2_A_DLPF_CFG);
    writeRegister(ACCEL_CONFIG2, v);
}

void AccelerometerMPU9250::setOutputDataRate(OutputDataRate odr) {
    writeRegister(LP_ACCEL_ODR, odr & LP_ACCEL_ODR_LPOSC_CLKSEL);
}

void AccelerometerMPU9250::enableAxis(Axis axis) {
    configureRegisterBits(PWR_MGMT_2, PWR_MGMT_2_DISABLE_A, ~axis);
}

void AccelerometerMPU9250::setWakeOnMotionThreshold(unsigned char womt) {
    writeRegister(WOM_THR, womt);
}

void AccelerometerMPU9250::enableWakeOnMotionDetection(bool enable) {
    configureRegisterBits(MOT_DETECT_CTRL, MOT_DETECT_CTRL_ACCEL_INTEL_EN, enable ? 0xff : 0x00);
}

void AccelerometerMPU9250::setAccelerationIntelligenceMode(AccelerationIntelligenceMode mode) {
    configureRegisterBits(MOT_DETECT_CTRL, MOT_DETECT_CTRL_ACCEL_INTEL_MODE, (mode == ON) ? 0xff : 0x00);
}

void AccelerometerMPU9250::enableCycle(bool enable) {
    configureRegisterBits(PWR_MGMT_1, PWR_MGMT_1_CYCLE, enable ? 0xff : 0x00);
}

void AccelerometerMPU9250::reset() {
    configureRegisterBits(PWR_MGMT_1, PWR_MGMT_1_H_RESET, 0xff);
}

void AccelerometerMPU9250::sleep() {
    configureRegisterBits(PWR_MGMT_1, PWR_MGMT_1_SLEEP, 0xff);
}

void AccelerometerMPU9250::awake() {
    configureRegisterBits(PWR_MGMT_1, PWR_MGMT_1_SLEEP, 0x00);
}

void AccelerometerMPU9250::enableFifo(bool en) {
    configureRegisterBits(FIFO_EN, FIFO_EN_ACCEL, en ? 0xff : 0x00);
}

void AccelerometerMPU9250::enableInterrupt(Interrupt interrupt) {
    configureRegisterBits(INT_ENABLE, interrupt, 0xff);
}

void AccelerometerMPU9250::disableInterrupt(Interrupt interrupt) {
    configureRegisterBits(INT_ENABLE, interrupt, 0x00);
}

float AccelerometerMPU9250::convertToG(unsigned char buf[2]) {
    int raw = (buf[0] << 8) | buf[1];
    float counts[] = { 16384.0, 8192.0, 4096.0, 2048.0 };
    return raw / counts[config.ACCEL_FS_SEL];
}
