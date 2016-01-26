#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerNunchuck.h>

AccelerometerNunchuck nunchuck;

void setup() {
    Serial.begin(9600);
    nunchuck.begin();
}

void loop() {
    
    nunchuck.readFrame();
    
    Serial.print("Z button: ");
    Serial.println(nunchuck.readZButton());
    
    Serial.print("C button: ");
    Serial.println(nunchuck.readCButton());
    
    Serial.print("Joystick X: ");
    Serial.println(nunchuck.readXJoystick());
    
    
    Serial.print("Joystick Y: ");
    Serial.println(nunchuck.readYJoystick());
    
    Serial.print("x: ");
    Serial.println(nunchuck.readXg());
    
    Serial.print("y: ");
    Serial.println(nunchuck.readYg());
    
    Serial.print("z: ");
    Serial.println(nunchuck.readZg());
    
    Serial.println("-----------");
    delay(500);
}
