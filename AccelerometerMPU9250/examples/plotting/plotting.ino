#include <Arduino.h>
#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMPU9250.h>

AccelerometerMPU9250 acc(0);
unsigned char buf[6];

void setup() {
    Serial.begin(9600);
    acc.setFullScaleRange(AccelerometerMPU9250::FS_SEL_8G);
}

void loop() {
    acc.readXYZ(buf);
    Serial.write(0xaa);
    Serial.write(buf, 6);
    Serial.write(0x00);
    unsigned long now = millis();
    Serial.write((unsigned char *) &now, 4);
    Serial.write(0xbb);
}