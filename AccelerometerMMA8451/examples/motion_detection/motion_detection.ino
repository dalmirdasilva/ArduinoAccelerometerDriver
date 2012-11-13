#define MMA8451 AccelerometerMMA8451

#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

/**
 * Motion and Freefall Detection Using the MMA8451
 * 
 * Example Steps for Configuring Motion Detection
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

    // Step 2: Set Configuration Register for Motion Detection by setting the 
    // "OR" condition OAE = 1, enabling X, Y, and the latch
    acc.configureRegisterBits(MMA8451::FF_MT_CFG, MMA8451::FF_MT_CFG_OAE, 0x04);

    // Step 3: Threshold Setting Value for the Motion detection of > 3g
    // Note: The step count is 0.063g/ count
    // 3g/0.063g = 47.6; //Round up to 48
    acc.configureRegisterBits(MMA8451::FF_MT_THS, MMA8451::FF_MT_THS_THS, 0x30);

    // Step 4: Set the debounce counter to eliminate false readings for 100 Hz 
    // sample rate with a requirement of 100 ms timer.
    // Note: 100 ms/10 ms (steps) = 10 counts
    acc.writeRegister(MMA8451::FF_MT_COUNT, 0x0a);

    // Step 5: Enable Motion/Freefall Interrupt Function in the System
    acc.enableInterrupt(MMA8451::INT_FF_MT);

    // Step 6: Route the Motion/Freefall Interrupt Function to INT1 hardware pin
    acc.routeInterruptToInt1(MMA8451::INT_FF_MT);

    // Step 7: Put the device in Active Mode
    acc.activate();

    // Step 8: Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);
}

void loop() {

    if (ready) {

        // Register 0x0C gives the status of any of the interrupts that are enabled in the entire device.
        // • An interrupt service routine must be set to handle enabling and then clearing of the interrupts. 
        //   Register 0x0C will be read to determine which interrupt caused the event.
        // • When bit 4 is set in Register 0x0C "SRC_LNDPRT" this is the indication that a new orientation has been detected.
        // • The interrupt source (0x0C) register and the PL_Status (0x10) register are 
        //   cleared and the new portrait/landscape detection can occur.

        //Determine the source of the interrupt by first reading the system interrupt register
        MMA8451::INT_SOURCEbits source;

        source.value = acc.readRegister(MMA8451::INT_SOURCE);

        // Set up Case statement here to service all of the possible interrupts
        if (source.SRC_FF_MT) {

            //Perform an Action since Orientation Flag has been set
            //Update Image on Display Screen based on the data stored

            //Read the PL State from the Status Register, clear the interrupt, PL Status Register
            acc.readRegister(MMA8451::PL_STATUS);

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);

            ready = false;
        }
    }
}

