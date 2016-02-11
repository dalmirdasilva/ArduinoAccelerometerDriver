#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMPU9250.h>

AccelerometerMPU9250 acc(0);

void setup() {
    Serial.begin(9600);
    acc.setFullScale(AccelerometerMPU9250::FS_SEL_8G);
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