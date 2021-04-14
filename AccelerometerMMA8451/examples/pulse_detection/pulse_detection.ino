#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);
volatile bool ready = false;
unsigned char buf[6];

void processXYZ(unsigned char* buf) {
    char axis[] = {'x', 'y', 'z'};
    for (int i = 0; i < 6; i += 2) {
        Serial.print(axis[i / 2]);
        Serial.print(": ");
        Serial.println(acc.convertToG(&buf[i]));
    }
}

void processPulseData(AccelerometerMMA8451::PULSE_SRCbits pulseSource) {
    Serial.println("Pulse data:");
    Serial.print("x: ");
    Serial.println(pulseSource.AXX, HEX);
    Serial.print("y: ");
    Serial.println(pulseSource.AXY, HEX);
    Serial.print("z: ");
    Serial.println(pulseSource.AXZ, HEX);

    Serial.print("px: ");
    Serial.println(pulseSource.POLX, HEX);
    Serial.print("py: ");
    Serial.println(pulseSource.POLY, HEX);
    Serial.print("pz: ");
    Serial.println(pulseSource.POLZ, HEX);
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

    // Start pulse detection: single tap on all axes
    acc.setPulseDetection(true, AccelerometerMMA8451::PULSE_CFG_XSPEFE | AccelerometerMMA8451::PULSE_CFG_YSPEFE | AccelerometerMMA8451::PULSE_CFG_ZSPEFE, false);
    // Set the same pulse detection threshold on all axes, and other parameters. Copied from AN4072.
    acc.setPulseThreshold(0x19, 0x19, 0x19);
    acc.setPulseFirstTimer(0x50);
    acc.setPulseLatency(0xf0);

    // interrupt settings seem to survive flashing Arduino, this makes sure other events does not raise INTs.
    acc.disableInterrupt(AccelerometerMMA8451::INT_ALL);
    // Register 0x2D, Control Register 4 configures all embedded features for interrupt detection.
    // To set this device up to run an interrupt service routine:
    // Program the Pulse Detection bit in Control Register 4.
    // Set bit 3 to enable the pulse detection "INT_PULSE".
    acc.enableInterrupt(AccelerometerMMA8451::INT_PULSE);

    // Register 0x2E is Control Register 5 which gives the option of routing the interrupt to either INT1 or INT2
    acc.routeInterruptToInt1(AccelerometerMMA8451::INT_PULSE);

    // Put the device in Active Mode
    acc.activate();

    // Write a Service Routine to Service the Interrupt
    attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);

    Serial.println("done.");
}

void loop() {

    if (ready) {

        ready = false;

        // Register INT_SOURCE(0x0C) gives the status of any of the interrupts that are enabled in the entire device.
        //
        // • An interrupt service routine must be set to handle enabling and then clearing of the interrupts.
        //   Register INT_SOURCE will be read to determine which interrupt caused the event.
        // • When bit 3 is set in Register INT_SOURCE "SRC_PULSE" this is the indication that a new pulse has been detected.
        // • This bit is asserted whenever “EA” bit in the PULSE_SRC is asserted and the interrupt has been enabled. This bit is
        //   cleared by reading the PULSE_SRC register

        // Determine the source of the interrupt by first reading the system interrupt register
        AccelerometerMMA8451::INT_SOURCEbits source;

        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);


        Serial.print("source: ");
        Serial.println(source.value, HEX);

        // Test if we got trasient interruption
        if (source.SRC_PULSE) {

            // Determine the source of the interrupt by first reading the system interrupt register
            AccelerometerMMA8451::PULSE_SRCbits pulseSource;

            // Clears the PULSE_SRC by reading the PULSE_SRC register
            pulseSource.value = acc.readRegister(AccelerometerMMA8451::PULSE_SRC);

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);

            processPulseData(pulseSource);
        }
    }
}
