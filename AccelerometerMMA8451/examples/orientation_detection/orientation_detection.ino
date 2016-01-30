#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

/**
 * Embedded Orientation Detection Using the MMA8451
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

    // We cannot read i2c inside an interrupt (Wire is interrupt oriented)
    ready = true;
}

void setup() {

    Serial.begin(9600);

    Serial.println("Setup...");

    // Step 1: Put the part into Standby Mode
    acc.standby();

    // Step 2: Set the data rate to 50 Hz (for example, but can choose any sample rate). 
    acc.setOutputDataRate(AccelerometerMMA8451::ODR_50HZ_20_MS);

    // Step 3: Set the PL_EN bit in Register 0x11 PL_CFG. This will enable the orientation detection.
    acc.setPortraitLandscapeDetection(true);

    // Step 4: Set the Back/Front Angle trip points in register 0x13 following the table in the data sheet.
    acc.setBackFrontTrip(AccelerometerMMA8451::BKFR_70);

    // Step 5: Set the Z-Lockout angle trip point in register 0x13 following the table in the data sheet.
    acc.setZLockThresholdAngle(AccelerometerMMA8451::ZLOCK_33);

    // Step 6: Set the Trip Threshold Angle
    acc.setPortraitLandscapeThresholdAngle(AccelerometerMMA8451::P_L_THS_75);

    // Step 7: Set the Hysteresis Angle
    acc.setHysteresisAngle(AccelerometerMMA8451::HYS_14);

    // Step 8: Register 0x2D, Control Register 4 configures all embedded features for interrupt detection.
    // To set this device up to run an interrupt service routine: 
    // Program the Orientation Detection bit in Control Register 4. 
    // Set bit 4 to enable the orientation detection “INT_EN_LNDPRT”.
    acc.enableInterrupt(AccelerometerMMA8451::INT_LNDPRT);

    // Step 9: Register 0x2E is Control Register 5 which gives the option of routing the interrupt to either INT1 or INT2
    acc.routeInterruptToInt1(AccelerometerMMA8451::INT_LNDPRT);

    // Step 10: Set the debounce counter in register 0x12
    acc.writeRegister(AccelerometerMMA8451::PL_COUNT, 0x05);

    // Step 11: Put the device in Active Mode
    acc.activate();

    // Step 12: Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);

    Serial.println("done.");
}

void loop() {

    if (ready) {

        ready = false;

        // Register 0x0C gives the status of any of the interrupts that are enabled in the entire device.
        // • An interrupt service routine must be set to handle enabling and then clearing of the interrupts. 
        //   Register 0x0C will be read to determine which interrupt caused the event.
        // • When bit 4 is set in Register 0x0C "SRC_LNDPRT" this is the indication that a new orientation has been detected.
        // • The interrupt source (0x0C) register and the PL_Status (0x10) register are 
        //   cleared and the new portrait/landscape detection can occur.

        // Determine the source of the interrupt by first reading the system interrupt register
        AccelerometerMMA8451::INT_SOURCEbits source;

        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);
    
        Serial.print("source: ");
        Serial.println(source.value, HEX);

        // Set up Case statement here to service all of the possible interrupts
        if (source.SRC_LNDPRT) {

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

