#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA7455.h>

AccelerometerMMA7455 acc;

void setup() {
    Serial.begin(9600);
    acc.measurementMode();
    acc.calibrate0gOffset(32);
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