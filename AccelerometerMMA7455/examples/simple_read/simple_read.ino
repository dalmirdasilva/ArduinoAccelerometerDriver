#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA7455.h>

AccelerometerMMA7455 acc;
unsigned char buf[6];

void setup() {
    Serial.begin(9600);
    acc.standby();
    acc.measurementMode();
    acc.setUse8bit(false);
    acc.setDynamicRange(AccelerometerMMA7455::DR_2G);
}

void loop() {
    Serial.print("x: ");
    Serial.println(acc.readXg());
    Serial.print("y: ");
    Serial.println(acc.readYg());
    Serial.print("z: ");
    Serial.println(acc.readZg());
    Serial.println("-----------");
    delay(1000);
}