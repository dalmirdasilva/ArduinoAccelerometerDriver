#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerNunchuk.h>

AccelerometerNunchuk nunchuk;
char b;

void setup() {
    Serial.begin(9600);
    nunchuk.begin();
    while (!Serial.available());
    b = Serial.read();
    if (b == 'a') {
        Serial.write('a');
    }
}

void loop() {

    while (!Serial.available());
    char b = Serial.read();
    
    nunchuk.readFrame();

    switch (b) {
        case 'b':
            Serial.println(nunchuk.readZButton());
            break;
        case 'c':
            Serial.println(nunchuk.readCButton());
            break;
        case 'd':
            Serial.println(nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_X, false));
            break;
        case 'e':
            Serial.println(nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Y, false));
            break;
        case 'f':
            Serial.println(nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Z, false));
            break;
        case 'g':
            Serial.println(nunchuk.readXJoystick());
            break;
        case 'h':
            Serial.println(nunchuk.readYJoystick());
            break;
    }
}