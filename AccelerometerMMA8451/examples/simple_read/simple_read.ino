#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);

void setup() {
    Serial.begin(9600);
}

void loop() {
    unsigned char buf[6];
    int sample, x, y, z;
    acc.activate();
    acc.readRegisterBlock(AccelerometerMMA8451::OUT_X_MSB, buf, 6);
    x = buf[0] << 2 | buf[1] >> 6 & 0x3;
    y = buf[2] << 2 | buf[3] >> 6 & 0x3;
    z = buf[4] << 2 | buf[5] >> 6 & 0x3;
    if (x > 511) x = x - 1024;
    if (y > 511) y = y - 1024 ;
    if (z > 511) z = z - 1024;
        
    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("z: ");
    Serial.println(z);
    Serial.println("---------------");
    delay(200);
}