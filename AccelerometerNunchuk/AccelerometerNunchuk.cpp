
#include <Wire.h>
uint8_t ctrlr_type[6];
uint8_t outbuf[6] = {};
int cnt = 0;

void setup() {
  
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  
  Serial.begin(9600);
  Serial.println("Setup...");
  Wire.begin();
  
  /*
  Wire.beginTransmission(0x52);      // device address
  Wire.write(0x40);		// sends memory address
  Wire.write(0x00);		// sends sent a zero.  
  Serial.println(Wire.endTransmission());  
  */
  delay(1);
  
  Wire.beginTransmission(0x52);      // device address
  Wire.write(0xf0);
  Wire.write(0xaa);
  Serial.println(Wire.endTransmission());  
  
  Wire.beginTransmission(0x52);      // device address
  Wire.write(0xFA);                    // 1st initialisation register
  Serial.println(Wire.endTransmission());  
  
  Wire.beginTransmission(0x52);
  Wire.requestFrom(0x52, 6);               // request data from controller
  for (cnt = 0; cnt < 6; cnt++) {
    while (!Wire.available()); 
    if (Wire.available()) {
      Serial.println(Wire.read(), HEX);
    }
  }
  Serial.println(Wire.endTransmission());  
}

void loop() {

  
  Wire.beginTransmission(0x52);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.requestFrom(0x52, 6);
  for (cnt = 0; cnt < 6; cnt++) {
    while (!Wire.available()); 
    if (Wire.available()) {
      outbuf[cnt] = Wire.read();
      //Serial.println(outbuf[cnt], HEX);
    }
  }
  Wire.endTransmission();
  puts();
  delay(100);
}


void
puts ()
{
  
  
  int joy_x_axis = outbuf[0];
  int joy_y_axis = outbuf[1];
  int accel_x_axis = outbuf[2] * 2 * 2; 
  int accel_y_axis = outbuf[3] * 2 * 2;
  int accel_z_axis = outbuf[4] * 2 * 2;

  int z_button = 0;
  int c_button = 0;

 // byte outbuf[5] contains bits for z and c buttons
 // it also contains the least significant bits for the accelerometer data
 // so we have to check each bit of byte outbuf[5]
  if ((outbuf[5] >> 0) & 1)
    {
      z_button = 1;
    }
  if ((outbuf[5] >> 1) & 1)
    {
      c_button = 1;
    }

  if ((outbuf[5] >> 2) & 1)
    {
      accel_x_axis += 2;
    }
  if ((outbuf[5] >> 3) & 1)
    {
      accel_x_axis += 1;
    }

  if ((outbuf[5] >> 4) & 1)
    {
      accel_y_axis += 2;
    }
  if ((outbuf[5] >> 5) & 1)
    {
      accel_y_axis += 1;
    }

  if ((outbuf[5] >> 6) & 1)
    {
      accel_z_axis += 2;
    }
  if ((outbuf[5] >> 7) & 1)
    {
      accel_z_axis += 1;
    }

  Serial.print (joy_x_axis, DEC);
  Serial.print ("\t");

  Serial.print (joy_y_axis, DEC);
  Serial.print ("\t");

  Serial.print (accel_x_axis, DEC);
  Serial.print ("\t");

  Serial.print (accel_y_axis, DEC);
  Serial.print ("\t");

  Serial.print (accel_z_axis, DEC);
  Serial.print ("\t");

  Serial.print (z_button, DEC);
  Serial.print ("\t");

  Serial.print (c_button, DEC);
  Serial.print ("\t");

  Serial.print ("\r\n");
}

