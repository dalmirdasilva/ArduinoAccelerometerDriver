#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMPU9250.h>

AccelerometerMPU9250 acc(0);
volatile bool ready = false;
unsigned char buf[6];
unsigned long now;

void isr() {
    ready = true;
}

void setup() {
    Serial.begin(9600);
    acc.enableAxis(AccelerometerMPU9250::AXIS_XYZ);
    acc.setLowPassFilter(true, AccelerometerMPU9250::DR_184_5_80_250_1);
    acc.enableInterrupt(AccelerometerMPU9250::WOM_EN);
    acc.setWakeOnMotionThreshold(0x30);
    acc.setOutputDataRate(AccelerometerMPU9250::ODR_15_63HZ);
    attachInterrupt(0, isr, FALLING);
}

void loop() {
    acc.readXYZ(buf);
    Serial.write(0xaa);
    Serial.write(buf, 6);
    Serial.write(ready ? 0x01 : 0x00);
    now = millis();
    Serial.write((unsigned char *) &now, 4);
    Serial.write(0xbb);
}