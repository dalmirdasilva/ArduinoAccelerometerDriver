#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);
SoftwareSerial serial(3, 4);
volatile bool ready = false;
unsigned char buf[6];
message[]
        unsigned char message[] = {0xaa, 0x1, 0x2, 0x3, 0x4, 0x00, 0x06, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0x00, 0x01, 0xbb};
serial.write(message);

void processXYZ(unsigned char* buf) {
    char axis[] = {'x', 'y', 'z'};
    for (int i = 0; i < 6; i += 2) {
        Serial.print(axis[i / 2]);
        Serial.print(": ");
        Serial.println(acc.convertToG(&buf[i], 2));
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
}

void isr() {

    // We cannot read i2c inside an interrupt (Wire is interrupt oriented)
    ready = true;
}

void setup() {

    Serial.begin(9600);

    Serial.println("Setup...");

    // Put the part into Standby Mode
    acc.standby();

    // Set the data rate to 50 Hz (for example, but can choose any sample rate). 
    acc.setOutputDataRate(AccelerometerMMA8451::ODR_50HZ_20_MS);

    // This will enable the transient detection.
    acc.setTransientDetection(true, 0x7f, 0x00);

    // Set the transient threshold.
    acc.setTransientThreshold(true, 0x0f);

    // Set the debounce counter
    acc.writeRegister(AccelerometerMMA8451::TRANSIENT_COUNT, 0x01);

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

