#include <Accelerometer.h>
#include <AccelerometerADXL335.h>

/**
 * Pinout
 * 
 * <pre>
 * Sensor   -> Arduino
 * -------------------
 * xPin     -> A0
 * yPin     -> A1
 * zPin     -> A2
 * 
 * VCC      -> 3v3
 * GND      -> GND
 * </pre>
 */

#define X_PIN   A0
#define Y_PIN   A1
#define Z_PIN   A2

AccelerometerADXL335 acc(X_PIN, Y_PIN, Z_PIN);

void setup() {
    Serial.begin(9600);
    pinMode(X_PIN, INPUT);
    pinMode(Y_PIN, INPUT);
    pinMode(Z_PIN, INPUT);
}

void loop() {
    Serial.print("x: ");
    Serial.println(acc.readXg());
    Serial.print("y: ");
    Serial.println(acc.readYg());
    Serial.print("z: ");
    Serial.println(acc.readZg());
    Serial.println("-----------\n\n");
    delay(1000);
}