#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);

void accelerometerHandler() {
  /*
  // Register 0x0C gives the status of any of the interrupts that are enabled in the entire device.
  // • An interrupt service routine must be set to handle enabling and then clearing of the interrupts. 
  //   Register 0x0C will be read to determine which interrupt caused the event.
  // • When bit 4 is set in Register 0x0C "SRC_LNDPRT" this is the indication that a new orientation has been detected.
  // • The interrupt source (0x0C) register and the PL_Status (0x10) register are 
  //   cleared and the new portrait/landscape detection can occur.

  //Determine the source of the interrupt by first reading the system interrupt register
  AccelerometerMMA8451::INT_SOURCEbits source;
  
  source.value = 1;//acc.readRegister(AccelerometerMMA8451::INT_SOURCE);
 
  // Set up Case statement here to service all of the possible interrupts
  if (source.SRC_LNDPRT) {
   
    //Perform an Action since Orientation Flag has been set
    //Update Image on Display Screen based on the data stored

    //Read the PL State from the Status Register, clear the interrupt, PL Status Register
    //acc.readRegister(AccelerometerMMA8451::PL_STATUS);
    
  }*/
    digitalWrite(13, HIGH);
    
    //Serial.println("got");
}

void setup() {
  
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  
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
  
  Serial.println("Activating...");
  // Step 12: Write a Service Routine to Service the Interrupt
  
  //attachInterrupt(0, accelerometerHandler, CHANGE);
  //attachInterrupt(1, accelerometerHandler, CHANGE);
}

void loop() {
  
    unsigned char buf[6];
    int sample, x, y, z;
    acc.readRegisterBlock(AccelerometerMMA8451::OUT_X_MSB, buf, 6);
    x = buf[0] << 2 | buf[1] >> 6 & 0x3;
    y = buf[2] << 2 | buf[3] >> 6 & 0x3;
    z = buf[4] << 2 | buf[5] >> 6 & 0x3;
    if (x > 511) x = x - 1024;
    if (y > 511) y = y - 1024 ;
    if (z > 511) z = z - 1024;
        
    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("z: ");
    Serial.println(z);
    Serial.println("---------------");
    digitalWrite(13, LOW);
    Serial.println(acc.readRegister(AccelerometerMMA8451::P_L_THS_REG), HEX);
    Serial.println(acc.readRegister(AccelerometerMMA8451::PL_BF_ZCOMP), HEX);
    Serial.println(acc.readRegister(AccelerometerMMA8451::PL_CFG), HEX);
    Serial.println(acc.readRegister(AccelerometerMMA8451::CTRL_REG1), HEX);
    Serial.println(acc.readRegister(AccelerometerMMA8451::CTRL_REG4), HEX);
    Serial.println(acc.readRegister(AccelerometerMMA8451::CTRL_REG5), HEX);
    delay(2000);
}