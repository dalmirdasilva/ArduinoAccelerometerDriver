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
    Serial.println("loop");
    unsigned int minX = 0xffff, minY = 0xffff, minZ = 0xffff;
    unsigned int maxX = 0x0000, maxY = 0x0000, maxZ = 0x0000;
    unsigned int aux;
    long ms = millis();
    while (ms + 10000 > millis()) {
        nunchuk.readFrame();
        aux = nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_X, false);
        
        if (aux < minX) {
            minX = aux;
        }
        if (aux > maxX) {
            maxX = aux;
        }
        aux = nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Y, false);
        
        if (aux < minY) {
            minY = aux;
        }
        if (aux > maxY) {
            maxY = aux;
        }
        aux = nunchuk.readAcceleration(AccelerometerNunchuk::AXIS_Z, false);
        
        if (aux < minZ) {
            minZ = aux;
        }
        if (aux > maxZ) {
            maxZ = aux;
        }
        Serial.println(".");
        delay(1000);
    }
    
    Serial.print("X: ");
    Serial.print(minX);
    Serial.print(" ");
    Serial.println(maxX);
    
    Serial.print("Y: ");
    Serial.print(minY);
    Serial.print(" ");
    Serial.println(maxY);
    
    Serial.print("Z: ");
    Serial.print(minZ);
    Serial.print(" ");
    Serial.println(maxZ);
    
    while (true);
}