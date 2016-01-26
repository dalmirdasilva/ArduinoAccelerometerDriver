#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerNunchuck.h>

AccelerometerNunchuck nunchuck;
char b;

void setup() {
    Serial.begin(9600);
    nunchuck.begin();
}

void loop() {

    while (!Serial.available());
    char b = Serial.read();
    
    nunchuck.readFrame();

    switch (b) {
        
        case '+':
            unsigned char *f;
            f = nunchuck.getFrame();
            for (int i = 0; i < 6; i++) {
                Serial.write(f[i]);
            }
            break;
        case '_':
            unsigned char out[2];
            if (nunchuck.readZButton()) {
                out[0] |= 0x80;
            }
            if (nunchuck.readCButton()) {
                out[0] |= 0x40;
            }
            if (nunchuck.readXJoystick() > 140 + 20) {
                out[0] |= 0x20;
            }
            if (nunchuck.readXJoystick() < 140 - 20) {
                out[0] |= 0x10;
            }
            if (nunchuck.readYJoystick() > 140 + 20) {
                out[0] |= 0x08;
            }
            if (nunchuck.readYJoystick() < 140 - 20) {
                out[0] |= 0x04;
            }
            if (nunchuck.readAcceleration(AccelerometerNunchuck::AXIS_X, false) > 512 + 20) {
                out[0] |= 0x02;
            }
            if (nunchuck.readAcceleration(AccelerometerNunchuck::AXIS_X, false) < 512 - 20) {
                out[0] |= 0x01;
            }
            if (nunchuck.readAcceleration(AccelerometerNunchuck::AXIS_Y, false) < 512 - 20) {
                out[1] |= 0x80;
            }
            if (nunchuck.readAcceleration(AccelerometerNunchuck::AXIS_Y, false) < 512 - 20) {
                out[1] |= 0x40;
            }
            if (nunchuck.readAcceleration(AccelerometerNunchuck::AXIS_Z, false) < 512 - 20) {
                out[1] |= 0x20;
            }
            if (nunchuck.readAcceleration(AccelerometerNunchuck::AXIS_Z, false) < 512 - 20) {
                out[1] |= 0x10;
            }
            Serial.write(out[0]);
            Serial.write(out[1]);
            break;
        case 'd':
            Serial.println(nunchuck.readZButton());
            break;
        case 'c':
            Serial.println(nunchuck.readCButton());
            break;
        case 'x':
            Serial.println(nunchuck.readXg());
            break;
        case 'y':
            Serial.println(nunchuck.readYg());
            break;
        case 'z':
            Serial.println(nunchuck.readZg());
            break;
        case 'w':
            Serial.println(nunchuck.readXJoystick());
            break;
        case 's':
            Serial.println(nunchuck.readYJoystick());
            break;
    }
}
