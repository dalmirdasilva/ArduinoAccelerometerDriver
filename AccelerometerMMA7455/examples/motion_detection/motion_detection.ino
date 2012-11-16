#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA7455.h>

/**
 * Motion Detection Using the AccelerometerMMA7455
 * 
 * Example Steps for Configuring Motion Detection
 */

AccelerometerMMA7455 acc;
bool ready = false;
unsigned char buf[6];

void processXYZ(unsigned char* buf) {
    char axis[] = {'x', 'y', 'z'};
    for (int i = 0; i < 6; i++) {
        Serial.print(i, DEC);
        Serial.print(": 0x");
        Serial.println(buf[i], HEX);
    }
    for (int i = 0; i < 6; i += 2) {
        Serial.print(axis[i / 2]);
        Serial.print(": ");
        Serial.println(acc.convertToG(&buf[i], false));
    }
}

void isr() {
    ready = true;
}

void setup() {

    Serial.begin(9600);
    Serial.print("Setup...");

    acc.standby();

    acc.setDetectionCondition(AccelerometerMMA7455::MOTION_DETECTION);
    
    acc.enableInterrupt(AccelerometerMMA7455::AXIS_X);
    
    acc.setInterruptConfiguration(AccelerometerMMA7455::INT1_LEVEL_INT2_PULSE);

    acc.levelDetectionMode();

    attachInterrupt(0, isr, RISING);

    Serial.println("done.");
}

void loop() {

    if (ready) {
        
        Serial.println("ready");

        ready = false;
        
        acc. clearInterruptLatch();

        AccelerometerMMA7455::DETSRCbits source;

        source.value = acc.readRegister(AccelerometerMMA7455::DETSRC);
    
        Serial.print("source: ");
        Serial.println(source.value, HEX);

        // Set up Case statement here to service all of the possible interrupts
        if (source.LDX) {

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);
        }
    }
}

