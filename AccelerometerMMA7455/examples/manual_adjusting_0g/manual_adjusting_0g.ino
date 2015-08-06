#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA7455.h>

AccelerometerMMA7455 acc;

void setup() {
    Serial.begin(9600);
    acc.measurementMode();
    unsigned char adjust[] = {0x0b, 0x00, 0x1e, 0x00, 0xdf, 0xff};
    acc.writeRegisterBlock(AccelerometerMMA7455::XOFFL, adjust, 6);
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