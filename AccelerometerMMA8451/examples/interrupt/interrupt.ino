#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

AccelerometerMMA8451 acc(0);
bool ready = false;
AccelerometerMMA8451::INT_SOURCEbits intSource;
unsigned char buf[6];

void processXYZ(unsigned char* buf) {
  for (int i = 0; i < 6; i++) {
    Serial.print(i, DEC);
    Serial.print(": 0x");
    Serial.println(buf[i], HEX);
  }
  for (int i = 0; i < 6; i += 2) {
    Serial.print("> ");
    Serial.println(acc.convertToG(&buf[i], 2));
  }
}

void isr() {
  ready = true;
}

void setup() {

  Serial.begin(9600);
  
  // Go to the Standby Mode
  acc.standby();
  
  // Clear the F_Read bit to ensure both MSB's and LSB's are indexed
  acc.setReadMode(AccelerometerMMA8451::NORMAL_READ);

  // Configure the INT pins for Open Drain
  acc.setPushPullOpenDrain(AccelerometerMMA8451::OPEN_DRAIN);
  
  // Configure the INT pins for Active Low
  acc.setInterruptPolarity(AccelerometerMMA8451::ACTIVE_LOW);
  
  // Enable the Data Ready Interrupt
  acc.enableInterrupt(AccelerometerMMA8451::INT_DRDY);
  
  // Route it to INT1.
  acc.routeInterruptToInt1(AccelerometerMMA8451::INT_DRDY);
  
  // Go back to Active Mode
  acc.activate();
  
  attachInterrupt(0, isr, FALLING);
}

void loop() {
  
  if (ready) {
    // Go read the Interrupt Source Register
    intSource.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);
    
    if (intSource.SRC_DRDY == 1) {
    
      // Read 14/12/10-bit XYZ results using a 6 byte IIC access.
      acc.readXYZ(buf);
      
      processXYZ(buf);
    }
    ready = false;
  }
}

