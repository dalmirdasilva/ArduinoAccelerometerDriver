#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

#define SAO_PIN 0
#define SCL_PIN 0
#define SDA_PIN 0
#define INT1_PIN 0
#define INT2_PIN 0

AccelerometerMMA8451 acc(SAO_PIN, SCL_PIN, SDA_PIN, INT1_PIN, INT2_PIN);

void setup() {
  acc.standby();
  acc.setOutputDataRate(AccelerometerMMA8451::ODR_50HZ_20_MS);
  acc.setPortraitLandscapeDetection(true);
  acc.setBackFrontTrip(AccelerometerMMA8451::BKFR_70);
  acc.setZLockThresholdAngle(AccelerometerMMA8451::ZLOCK_33);
  acc.setPortraitLandscapeThresholdAngle(0x19);
  acc.setHysteresisAngle(AccelerometerMMA8451::HYS_14);
  acc.enableInterrupt(AccelerometerMMA8451::INT_LNDPRT);
  acc.routeInterruptToInt1(AccelerometerMMA8451::INT_LNDPRT);
  acc.writeRegister(AccelerometerMMA8451::PL_COUNT, 0x05);
  acc.activate();
}

void loop() {
}