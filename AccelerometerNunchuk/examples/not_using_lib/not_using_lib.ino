
#include <Wire.h>

uint8_t outbuf[6];
int cnt = 0;

void setup() {
    Serial.begin(9600);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
    digitalWrite(A2, LOW);
    digitalWrite(A3, HIGH);
    Wire.begin();
    nunchuck_init();
}

void nunchuck_init() {
    Wire.beginTransmission(0x52);
    Wire.write(0xf0);
    Wire.write(0xaa);
    Serial.println(Wire.endTransmission());
}

void send_zero() {
    Wire.beginTransmission(0x52);
    Wire.write(0x00);
    Wire.endTransmission();
}

void loop() {
    Wire.requestFrom(0x52, 6);
    while (Wire.available()) {
        outbuf[cnt] = nunchuk_decode_byte(Wire.read());
        cnt++;
    }

    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) {
        puts();
    } else {
        Serial.println("Not received.");
    }

    cnt = 0;
    send_zero();
    delay(100);
}

void puts() {
    
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

char nunchuk_decode_byte(char x) {
    x = (x ^ 0x17) + 0x17;
    return x;
}
