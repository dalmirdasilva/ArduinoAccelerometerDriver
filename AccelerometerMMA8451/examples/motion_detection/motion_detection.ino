#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

/**
 * Motion Detection Using the AccelerometerMMA8451
 * 
 * Example Steps for Configuring Motion Detection
 * 
 * Changes provide by: Chuck Todd <ctodd@cableone.net>
 */

AccelerometerMMA8451 acc(0);
bool ready = false;
unsigned char buf[6];
unsigned long tm = 0;

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

    // Step 1: Put the device into Standby Mode: Register 0x2A CTRL_REG1
    acc.standby();

    // Step 2: Set Configuration Register for Motion Detection by setting the 
    // "OR" condition OAE = 1, enabling X, Y, and the latch
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_OAE, 0x40);

    // Event flag enable on X, Y and Z event.
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_ZEFE, 0x20);
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_YEFE, 0x10);
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_XEFE, 0x08);

    // Step 3: Threshold Setting Value for the Motion detection of > 3g
    // Note: The step count is 0.063g/ count
    // 3g/0.063g = 47.6; 
    // Round up to 48 (Use 16 for weaker acceleration)
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_THS, AccelerometerMMA8451::FF_MT_THS_THS, 0x30);

    // Step 4: Set the debounce counter to eliminate false readings for 100 Hz 
    // sample rate with a requirement of 100 ms timer.
    // Note: 100 ms/10 ms (steps) = 10 counts (Use 0x04 for shorter)
    acc.writeRegister(AccelerometerMMA8451::FF_MT_COUNT, 0x0a);

    // Configure the INT pins for Open Drain
    acc.setPushPullOpenDrain(AccelerometerMMA8451::PUSH_PULL);

    // Configure the INT pins for Active Low
    acc.setInterruptPolarity(AccelerometerMMA8451::ACTIVE_LOW);

    // Step 5: Enable Motion/Freefall Interrupt Function in the System
    // And route to the pin 1
    acc.enableInterrupt(AccelerometerMMA8451::INT_FF_MT, 1);

    // Step 7: Put the device in Active Mode
    acc.activate();

    // Step 8: Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);

    Serial.println("done.");
}

void loop() {
    
    AccelerometerMMA8451::INT_SOURCEbits source;
    AccelerometerMMA8451::STATUSbits status;
    
    if ((millis() - tm) > 10000) { // timeout
        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);
        status.value = acc.readRegister(AccelerometerMMA8451::STATUS);
        byte b1 = acc.readRegister(AccelerometerMMA8451::CTRL_REG3);
        byte b2 = acc.readRegister(AccelerometerMMA8451::CTRL_REG4);
        byte b3 = acc.readRegister(AccelerometerMMA8451::CTRL_REG5);
        char buf[80];
        sprintf(buf, "timeout int_src %2x, Data_status %2x, Int_type %2x, int_src %2x, int_pin %2x",
                source.value, status.value, b1, b2, b3);
        Serial.println(buf);
        tm = (unsigned long) millis();

        // Process interrupts even though Arduino never received hardware interrupt
        if (source.value) ready = true;
    }

    if (ready) {

        Serial.println("ready");

        ready = false;

        // Determine the source of the interrupt by first reading the system interrupt register
        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);

        Serial.print("source: ");
        Serial.println(source.value, HEX);

        // Set up Case statement here to service all of the possible interrupts
        if (source.SRC_FF_MT) {

            // Clears the interrupt
            int src = acc.readRegister(AccelerometerMMA8451::FF_MT_SRC);
            Serial.println(src, BIN);
        }

        // Clear Data Ready to enable next interrupt.
        if (source.SRC_DRDY) {
            acc.readXYZ(buf);
            processXYZ(buf);
        }

        tm = (unsigned long) millis();
    }
}
