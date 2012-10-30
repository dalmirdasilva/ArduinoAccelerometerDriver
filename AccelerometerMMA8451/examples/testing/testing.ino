#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);

void setup() {
    
    unsigned char b;
    
    Serial.begin(9600);  

    acc.standby();

    b = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);




    // Step 2: Set the data rate to 50 Hz (for example, but can choose any sample rate). 
    acc.setOutputDataRate(AccelerometerMMA8451::ODR_50HZ_20_MS);

    // Step 3: Set the PL_EN bit in Register 0x11 PL_CFG. This will enable the orientation detection.
    acc.setPortraitLandscapeDetection(true);

    // Step 4: Set the Back/Front Angle trip points in register 0x13 following the table in the data sheet.
    acc.setBackFrontTrip(AccelerometerMMA8451::BKFR_70);

    // Step 5: Set the Z-Lockout angle trip point in register 0x13 following the table in the data sheet.
    acc.setZLockThresholdAngle(AccelerometerMMA8451::ZLOCK_33);

    // Step 6: Set the Trip Threshold Angle
    acc.setPortraitLandscapeThresholdAngle(0x19);

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
    attachInterrupt(0, accelerometerHandler, RISING);
}

void loop() {
}