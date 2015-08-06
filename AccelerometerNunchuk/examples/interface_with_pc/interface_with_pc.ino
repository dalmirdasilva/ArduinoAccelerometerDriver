#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerNunchuk.h>

AccelerometerNunchuk nunchuk;
char b;

void setup() {
    Serial.begin(9600);
    nunchuk.begin();
}

void loop() {

    while (!Serial.available());
    char b = Serial.read();
    
    nunchuk.readFrame();

    switch (b) {
        
        case '+':
            unsigned char *f;
            f = nunchuk.getFrame();
            for (int i = 0; i < 6; i++) {
                Serial.write(f[i]);
            }
            break;
        case '_':
            unsigned char out[2];
            if (nunchuk.readZButton()) {
                out[0] |= 0x80;
            }
            if (nunchuk.readCButton()) {
                out[0] |= 0x40;
            }
            if (nunchuk.readXJoystick() > 140 + 20) {
                out[0] |= 0x20;
            }
            if (nunchuk.readXJoystick() < 140 - 20) {
                out[0] |= 0x10;
            }
            if (nunchuk.readYJoystick() > 140 + 20) {
                out[0] |= 0x08;
            }
            if (nunchuk.readYJoystick() < 140 - 20) {
                out[0] |= 0x04;
            }
            if (nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_X, false) > 512 + 20) {
                out[0] |= 0x02;
            }
            if (nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_X, false) < 512 - 20) {
                out[0] |= 0x01;
            }
            if (nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Y, false) < 512 - 20) {
                out[1] |= 0x80;
            }
            if (nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Y, false) < 512 - 20) {
                out[1] |= 0x40;
            }
            if (nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Z, false) < 512 - 20) {
                out[1] |= 0x20;
            }
            if (nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Z, false) < 512 - 20) {
                out[1] |= 0x10;
            }
            Serial.write(out[0]);
            Serial.write(out[1]);
            break;
        case 'd':
            Serial.println(nunchuk.readZButton());
            break;
        case 'c':
            Serial.println(nunchuk.readCButton());
            break;
        case 'x':
            Serial.println(nunchuk.readXg());
            break;
        case 'y':
            Serial.println(nunchuk.readYg());
            break;
        case 'z':
            Serial.println(nunchuk.readZg());
            break;
        case 'w':
            Serial.println(nunchuk.readXJoystick());
            break;
        case 's':
            Serial.println(nunchuk.readYJoystick());
            break;
    }
}