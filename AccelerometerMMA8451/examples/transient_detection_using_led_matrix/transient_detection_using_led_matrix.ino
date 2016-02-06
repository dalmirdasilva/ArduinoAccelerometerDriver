#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

#include <MAX7219Driver.h>
#include <LedMatrixDriver.h>
#include <LedMatrixMAX7219Driver.h>

#include <SoftwareSerial.h>

#define DEBOUNCE_INPUT_PIN A2
#define THRESHOLD_INPUT_PIN A3
#define MIN_TIME_BETWEEN_STEPS 1000

MAX7219Driver driver(10, 11, 12);
LedMatrixMAX7219Driver matrix(&driver, 8, 8);
unsigned long resistor, lastStepTime;
unsigned char threshold = 0x08, debounce = 0x05;

/**
 * Embedded Transient Detection Using the MMA8451
 */

AccelerometerMMA8451 acc(0);
volatile bool ready = false;
unsigned char buf[6];

void processXYZ(unsigned char* buf) {
    char axis[] = { 'x', 'y', 'z' };
    for (int i = 0; i < 6; i += 2) {
        Serial.print(axis[i / 2]);
        Serial.print(": ");
        Serial.println(acc.convertToG(&buf[i]));
    }
}

void processTransientData(AccelerometerMMA8451::TRANSIENT_SRCbits transientSource) {
    Serial.print("x: ");
    Serial.print(transientSource.XTRANSE, HEX);
    Serial.print(" px: ");
    Serial.println(transientSource.X_TRANS_POL);
    Serial.print("y: ");
    Serial.print(transientSource.YTRANSE, HEX);
    Serial.print(" py: ");
    Serial.println(transientSource.Y_TRANS_POL);
    Serial.print("z: ");
    Serial.print(transientSource.ZTRANSE, HEX);
    Serial.print(" pz: ");
    Serial.println(transientSource.Z_TRANS_POL);

    unsigned long now = millis();
    if (now - MIN_TIME_BETWEEN_STEPS > lastStepTime && (transientSource.X_TRANS_POL + transientSource.Y_TRANS_POL + transientSource.Z_TRANS_POL) == 0) {
        lastStepTime = now;
        matrix.fill();
        delay(100);
        matrix.clear();
        Serial.println("STEP");
    }
}

void isr() {

    // We cannot read i2c inside an interrupt (Wire is interrupt oriented)
    ready = true;
}

void setup() {

    Serial.begin(9600);
    Serial.println("Setup...");
    
    lastStepTime = millis();

    pinMode(THRESHOLD_INPUT_PIN, INPUT);

    matrix.clear();

    // Put the part into Standby Mode
    acc.standby();

    // Set the data rate to 50 Hz (for example, but can choose any sample rate). 
    acc.setOutputDataRate(AccelerometerMMA8451::ODR_100HZ_10_MS);

    // This will enable the transient detection.
    acc.setTransientDetection(true, 0x07, 0x00);

    // Set the transient threshold.
    acc.setTransientThreshold(true, threshold);

    // Set the debounce counter
    acc.writeRegister(AccelerometerMMA8451::TRANSIENT_COUNT, debounce);

    // Register 0x2D, Control Register 4 configures all embedded features for interrupt detection.
    // To set this device up to run an interrupt service routine: 
    // Program the Transient Detection bit in Control Register 4. 
    // Set bit 5 to enable the transient detection "INT_TRANS".
    acc.enableInterrupt(AccelerometerMMA8451::INT_TRANS);

    // Register 0x2E is Control Register 5 which gives the option of routing the interrupt to either INT1 or INT2
    acc.routeInterruptToInt1(AccelerometerMMA8451::INT_TRANS);

    // Put the device in Active Mode
    acc.activate();

    // Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);

    Serial.println("done.");
}

void loop() {
    resistor = analogRead(THRESHOLD_INPUT_PIN);
    unsigned char newThreshold = map(resistor, 0, 1023, 0, 0x7f) & 0xff;
    if (abs(newThreshold - threshold) > 4) {
        acc.setTransientThreshold(true, newThreshold);
        threshold = newThreshold;
        Serial.print("new threshold: ");
        Serial.println(threshold, HEX);
    }

    resistor = analogRead(DEBOUNCE_INPUT_PIN);
    unsigned char newDebounce = map(resistor, 0, 1023, 0, 255) & 0xff;
    if (abs(newDebounce - debounce) > 4) {
        acc.writeRegister(AccelerometerMMA8451::TRANSIENT_COUNT, newDebounce);
        debounce = newDebounce;
        Serial.print("new debounce: ");
        Serial.println(debounce, HEX);
    }

    if (ready) {

        ready = false;

        // Register INT_SOURCE(0x0C) gives the status of any of the interrupts that are enabled in the entire device.
        //
        // • An interrupt service routine must be set to handle enabling and then clearing of the interrupts. 
        //   Register INT_SOURCE will be read to determine which interrupt caused the event.
        // • When bit 5 is set in Register INT_SOURCE "SRC_TRANS" this is the indication that a new transient has been detected.
        // • This bit is asserted whenever “EA” bit in the TRANS_SRC is asserted and the interrupt has been enabled. This bit is
        //   cleared by reading the TRANS_SRC register

        // Determine the source of the interrupt by first reading the system interrupt register
        AccelerometerMMA8451::INT_SOURCEbits source;

        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);

        Serial.print("source: ");
        Serial.println(source.value, HEX);

        // Test if we got trasient interruption
        if (source.SRC_TRANS) {

            // 0x1E: TRANSIENT_SRC Register
            // The Transient Source register provides the status of the enabled axes and the polarity (directional) information. When this
            // register is read it clears the interrupt for the transient detection. When new events arrive while EA = 1, additional *TRANSE bits
            // may get set, and the corresponding *_Trans_Pol flag become updated. However, no *TRANSE bit may get cleared before the
            // TRANSIENT_SRC register is read. 

            // Determine the source of the interrupt by first reading the system interrupt register
            AccelerometerMMA8451::TRANSIENT_SRCbits transientSource;

            // Clears the TRANS_SRC by reading the TRANS_SRC register
            transientSource.value = acc.readRegister(AccelerometerMMA8451::TRANSIENT_SRC);

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);

            processTransientData(transientSource);
        }
    }
}