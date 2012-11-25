#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerNunchuk.h>

AccelerometerNunchuk nunchuk;

void setup() {
    Serial.begin(9600);
    nunchuk.begin();
}

void loop() {
    
    nunchuk.readFrame();
    
    Serial.print("Z button: ");
    Serial.println(nunchuk.readZButton());
    
    Serial.print("C button: ");
    Serial.println(nunchuk.readCButton());
    
    Serial.print("Joystick X: ");
    Serial.println(nunchuk.readXJoystick());
    
    
    Serial.print("Joystick Y: ");
    Serial.println(nunchuk.readYJoystick());
    
    Serial.print("x: ");
    Serial.println(nunchuk.readXg());
    
    Serial.print("y: ");
    Serial.println(nunchuk.readYg());
    
    Serial.print("z: ");
    Serial.println(nunchuk.readZg());
    
    Serial.println("-----------");
    delay(500);
}