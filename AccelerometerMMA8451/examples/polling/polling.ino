#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);

void processXYZ(unsigned char* buf) {
  for (int i = 0; i < 6; i++) {
    Serial.print(i, DEC);
    Serial.print(": 0x");
    Serial.println(buf[i], HEX);
  }
  delay(500);
}

void setup() {
  
  Serial.begin(9600);
  
  // Go to the Standby Mode
  acc.standby();
  
  // Clear the F_Read bit to ensure both MSB's and LSB's are indexed
  acc.setReadMode(AccelerometerMMA8451::NORMAL_READ);

  // Go back to Active Mode
  acc.activate();
}

void loop() {

  AccelerometerMMA8451::STATUSbits status;
  unsigned char buf[6];
  
  // Using a basic control loop, continuously poll the sensor.
  for (;;) {
    
    // Poll the ZYXDR status bit and wait for it to set.
    status.value = acc.readRegister(AccelerometerMMA8451::STATUS);
    
    if (status.ZYXDR) {

      // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
      acc.readRegisterBlock(AccelerometerMMA8451::OUT_X_MSB, buf, 6);
      
      // Go process the XYZ data.
      processXYZ(buf);
    }
  }
}

