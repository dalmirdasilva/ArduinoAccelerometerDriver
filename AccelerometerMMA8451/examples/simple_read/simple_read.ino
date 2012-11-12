#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);

void setup() {
    Serial.begin(9600);
    acc.standby();
    acc.setDynamicRange(AccelerometerMMA8451::DR_4G);
    acc.activate();
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