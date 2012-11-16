#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

/**
 * Motion and Freefall Detection Using the AccelerometerMMA8451
 * 
 * 6.2 - Example Steps for Configuring Linear Freefall Detection
 */

AccelerometerMMA8451 acc(0);
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

    Serial.print("Setup...");

    // Step 1: Put the device into Standby Mode: Register 0x2A CTRL_REG1
    acc.standby();
    
    acc.setDynamicRange(AccelerometerMMA8451::DR_4G);

    // Step 2: Configuration Register set for Freefall Detection enabling "AND" 
    // condition, OAE = 0, Enabling X, Y, Z and the Latch
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_OAE, 0x00);
    
    // Event flag enable on X, Y and Z event.
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_ZEFE, 0x20);
    
    // Step 3: Threshold Setting Value for the resulting acceleration < 0.2g
    // Note: The step count is 0.063g/count
    // 0.2g/0.063g = 3.17 counts 
    // Round to 3 counts
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_THS, AccelerometerMMA8451::FF_MT_THS_THS, 0x03);

    // Step 4: Set the debounce counter to eliminate false positive readings for 
    // 50Hz sample rate with a requirement of 120 ms timer, assuming Normal Mode.
    // Note: 120 ms/20 ms (steps) = 6 counts
    acc.writeRegister(AccelerometerMMA8451::FF_MT_COUNT, 0x06);

    // Configure the INT pins for Open Drain
    acc.setPushPullOpenDrain(AccelerometerMMA8451::PUSH_PULL);

    // Configure the INT pins for Active Low
    acc.setInterruptPolarity(AccelerometerMMA8451::ACTIVE_LOW);

    // Step 5: Enable Motion/Freefall Interrupt Function in the System
    acc.enableInterrupt(AccelerometerMMA8451::INT_FF_MT);

    // Step 6: Route the Motion/Freefall Interrupt Function to INT2 hardware pin
    acc.routeInterruptToInt1(AccelerometerMMA8451::INT_FF_MT);

    // Step 7: Put the device in Active Mode
    acc.activate();

    // Step 8: Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);

    Serial.println("done.");
}

void loop() {

    if (ready) {
        
        Serial.println("ready");

        ready = false;

        // Determine the source of the interrupt by first reading the system interrupt register
        AccelerometerMMA8451::INT_SOURCEbits source;

        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);
    
        Serial.print("source: ");
        Serial.println(source.value, HEX);

        // Set up Case statement here to service all of the possible interrupts
        if (source.SRC_FF_MT) {

            // Perform an Action since Orientation Flag has been set
            // Update Image on Display Screen based on the data stored

            // Read the PL State from the Status Register, clear the interrupt, PL Status Register
            acc.readRegister(AccelerometerMMA8451::PL_STATUS);

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);
        }
    }
}

