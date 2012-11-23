#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

/**
 * Motion Detection Using the AccelerometerMMA8451
 * 
 Origionally Written by Dalmir da Silva <dalmirdasilva@gmail.com>
 Modified 
 21NOV2012 Chuck Todd 
    o Explicitly set only FF_MT interrupt
    o add code to clear all pending interrupts
     
 
 Connect SCL to AD5/SCL  must have a Voltage convert between Arduino Uno and MMA8451 for each data line.
 Connect SDA to AD4/SDA
 Connect INT1 to D2
  
 * Example Steps for Configuring Motion Detection
 */

AccelerometerMMA8451 acc(0); // SA0 of MMA8451 is LOW so Address is 0x1C
volatile bool ready = false;
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

    // Step 1: Put the device into Standby Mode: Register 0x2A CTRL_REG1
    acc.standby();

    // Step 2: Set Configuration Register for Motion Detection by setting the 
    // "OR" condition OAE = 1, enabling X, Y, and the latch
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_OAE, 0x00);

    // Event flag enable on X, Y and Z event.
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_ZEFE, 0x20);
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_YEFE, 0x10);
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_XEFE, 0x08);

    // Step 3: Threshold Setting Value for the Motion detection of > 3g
    // Note: The step count is 0.063g/ count
    // 3g/0.063g = 47.6; 
    // Round up to 48
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_THS, AccelerometerMMA8451::FF_MT_THS_THS, 0x10);

    // Step 4: Set the debounce counter to eliminate false readings for 100 Hz 
    // sample rate with a requirement of 100 ms timer.
    // Note: 100 ms/10 ms (steps) = 10 counts
    acc.writeRegister(AccelerometerMMA8451::FF_MT_COUNT, 0x0a);

    // Configure the INT pins for Open Drain
    acc.setPushPullOpenDrain(AccelerometerMMA8451::PUSH_PULL);

    // Step 5: Enable Motion/Freefall Interrupt Function in the System
    // disable all interrupt sources we only want Freefall or Motion interrupt
    acc.disableInterrupt(AccelerometerMMA8451::INT_ALL); 
    acc.enableInterrupt(AccelerometerMMA8451::INT_FF_MT, 1);
    //acc.enableInterrupt(AccelerometerMMA8451::INT_DRDY, 1);

    // Step 7: Put the device in Active Mode
    acc.activate();

    // display attached device id
    unsigned char b;
    b = acc.readRegister(AccelerometerMMA8451::WHO_AM_I);
    switch (b) {
        case(0x2A):
            Serial.println("MMA8452 present");
            break;
        case(0x1A):
            Serial.println("MMA8451 present");
            break;
        default:
            Serial.print("unknown device present 0x");
            Serial.println(b, HEX);
    }

    // Step 8: Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);

    Serial.println("done.");
}
unsigned long tm = millis(); // timeout counter 

void loop() {

    AccelerometerMMA8451::INT_SOURCEbits source;

    // 10 seconds since last interrupt is something wrong? read most of the MMA8451's status regs
    if ((unsigned long) (millis() - tm) > 10000) {
        
        // this section is for debuging purposes.  if there is not activity for 10 seconds something may be wrong, so display
        // the MMA8451's interrupt status / config registers to verify everything is correctly configured.

        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);

        AccelerometerMMA8451::STATUSbits status;
        status.value = acc.readRegister(AccelerometerMMA8451::STATUS);

        AccelerometerMMA8451::CTRL_REG3bits ctrl_reg3;
        ctrl_reg3.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG3);

        AccelerometerMMA8451::CTRL_REG4bits ctrl_reg4;
        ctrl_reg4.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG4);

        AccelerometerMMA8451::CTRL_REG5bits ctrl_reg5;
        ctrl_reg5.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG5);

        char buf[80];
        sprintf(buf, "INT_SOURCE: %2u, Data Status: %2u, Int_cfg: %2u, INT_ENABLE: %2u, INT_OUT: %2u", source.value, status.value, ctrl_reg3.value,
                ctrl_reg4.value, ctrl_reg5.value);

        Serial.print("Time out ");
        Serial.println(buf);

        if (source.value) {
            ready = true;
        } 
        
        // hardware interrupt never triggered, possible hardware fault
        // acknowledge interrupts by reading status value for each interrupt
        // this will allow the MMA8451 to set a new interrupt.

        tm = (unsigned long) millis(); // restart timeout counter
    }

    if (ready) {

        // Serial.println("ready");

        ready = false;

        //Determine the source of the interrupt by first reading the system interrupt register

        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);

        Serial.print("source: ");
        Serial.print(source.value, HEX);
        Serial.print(" ");
        
        // Set up Case statement here to service all of the possible interrupts
        
        // Data Ready / Data Overflow Interrupt (bit 0 of source)
        if (source.SRC_DRDY) {

            // Read 14/12/10-bit XYZ results using a 6 byte IIC access this clear the DRDY interrupt.
            acc.readXYZ(buf);

            // Puts the values.
            processXYZ(buf);
        }
        
        
        // Freefall or Motion Detection Interrupt (bit 2 of source)
        if (source.SRC_FF_MT) { 

            //Read the FF_MT State from the Status Register, clear the interrupt, by reading the FF_MT_SRC Register
            AccelerometerMMA8451::FF_MT_SRCbits ff_mt_src;
            ff_mt_src.value = acc.readRegister(AccelerometerMMA8451::FF_MT_SRC);

            Serial.print("FreeFall Motion Interrupt: ");
            Serial.println(ff_mt_src.value, BIN);
        }
        
        // single or Double Tap Pulse Detected (bit 3 of source)
        if (source.SRC_PULSE) { 
            // Read the PULSE_SRC register to clear Interrupt;
            AccelerometerMMA8451::PULSE_SRCbits pulse_src;
            pulse_src.value = acc.readRegister(AccelerometerMMA8451::PULSE_SRC);

            Serial.print("Tap Interrupt ");
            Serial.println(pulse_src.value, BIN);
        }
        
        // Portrait or landscape orientation Change detected (bit 4 of source)
        if (source.SRC_LNDPRT) {
            // Read the PL_STATUS register to clear Interrupt'

            AccelerometerMMA8451::PL_STATUSbits pl_status;
            pl_status.value = acc.readRegister(AccelerometerMMA8451::PL_STATUS);

            Serial.print("Orientation Change ");
            Serial.println(pl_status.value, BIN);
        }
        
        // Transient interrupt (bit 5 of status)
        if (source.SRC_TRANS) { 
            // Read TRANS_SRC register to clear interrupt

            AccelerometerMMA8451::TRANSIENT_SRCbits trans_src;
            trans_src.value = acc.readRegister(AccelerometerMMA8451::TRANSIENT_SRC);

            Serial.print("Transient Detected ");
            Serial.println(trans_src.value, BIN);
        }
        
        // FIFO Overflow or Watermark hit (bit 6 of status)
        if (source.SRC_FIFO) { 
            // Read F_STATUS register to Clear Interrupt

            AccelerometerMMA8451::F_STATUSbits f_status;
            f_status.value = acc.readRegister(AccelerometerMMA8451::F_STATUS);

            Serial.print("FIFO overflow/watermark ");
            Serial.println(f_status.value, BIN);
        }
        
        // sleep to wake or wake to sleep because of inactivity timeout (bit 7 of status)
        if (source.SRC_ASLP) { 
            
            // Read SYSMOD register to Clear Interrupt
            AccelerometerMMA8451::SYSMODbits sysmod;
            sysmod.value = acc.readRegister(AccelerometerMMA8451::SYSMOD);

            Serial.print("sleep/wake transistion ");
            Serial.println(sysmod.value, BIN);
        }

        // init timeout counter
        tm = (unsigned long) millis();
    }
}