#define MMA8451 AccelerometerMMA8451

#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

/**
 * Motion and Freefall Detection Using the MMA8451
 * 
 * 6.2 - Example Steps for Configuring Linear Freefall Detection
 */

MMA8451 acc(0);
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
        Serial.println(acc.convertToG(&buf[i], 2));
    }
}

void isr() {
    ready = true;
}

void setup() {

    Serial.begin(9600);

    // Step 1: Put the device into Standby Mode: Register 0x2A CTRL_REG1
    acc.standby();

    // Step 2: Configuration Register set for Freefall Detection enabling "AND" 
    // condition, OAE = 0, Enabling X, Y, Z and the Latch
    acc.configureRegisterBits(MMA8451::FF_MT_CFG, MMA8451::FF_MT_CFG_OAE, 0x00);

    // Step 3: Threshold Setting Value for the resulting acceleration < 0.2g
    // Note: The step count is 0.063g/count
    // 0.2g/0.063g = 3.17 counts 
    // Round to 3 counts
    acc.configureRegisterBits(MMA8451::FF_MT_THS, MMA8451::FF_MT_THS_THS, 0x03);

    // Step 4: Set the debounce counter to eliminate false positive readings for 
    // 50Hz sample rate with a requirement of 120 ms timer, assuming Normal Mode.
    // Note: 120 ms/20 ms (steps) = 6 counts
    acc.writeRegister(MMA8451::FF_MT_COUNT, 0x06);

    // Step 5: Enable Motion/Freefall Interrupt Function in the System
    acc.enableInterrupt(MMA8451::INT_FF_MT);

    // Step 6: Route the Motion/Freefall Interrupt Function to INT2 hardware pin
    acc.routeInterruptToInt2(MMA8451::INT_FF_MT);

    // Step 7: Put the device in Active Mode
    acc.activate();

    // Step 8: Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);
}

void loop() {

    if (ready) {

        // Determine the source of the interrupt by first reading the system interrupt register
        MMA8451::INT_SOURCEbits source;

        source.value = acc.readRegister(MMA8451::INT_SOURCE);

        // Set up Case statement here to service all of the possible interrupts
        if (source.SRC_FF_MT) {

            // Perform an Action since Orientation Flag has been set
            // Update Image on Display Screen based on the data stored

            // Read the PL State from the Status Register, clear the interrupt, PL Status Register
            acc.readRegister(MMA8451::PL_STATUS);

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);

            ready = false;
        }
    }
}

