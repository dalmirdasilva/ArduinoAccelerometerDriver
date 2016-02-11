#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMPU9250.h>

AccelerometerMPU9250 acc(0);
volatile bool ready = false;

/**
 * Configuration Wake-on-Motion Interrupt using low power Accel mode 
 * 
 * Make Sure Accel is running:
 *      In PWR_MGMT_1 (0x6B) make CYCLE =0, SLEEP = 0 and STANDBY = 0
 *      In PWR_MGMT_2 (0x6C) set DIS_XA, DIS_YA, DIS_ZA = 0 and DIS_XG, DIS_YG, DIS_ZG = 1
 *  
 * Set Accel setting to 184 Hz Bandwidth:
 *      In ACCEL_CONFIG 2 (0x1D) set ACCEL_FCHOICE_B 1 and A_DLPFCFG[2:]=1(b001)
 * 
 * Enable Motion Interrupt:
 *      In INT_ENABLE (0x38), set the whole register to 0x40 to enable motion interrupt only.
 * 
 * Enable Accel Hardware Intelligence:
 *      In MOT_DETECT_CTRL (0x69), set ACCEL_INTEL_EN = 1 and ACCEL_INTEL_MODE = 1
 * 
 * Set Motion Threshold:
 *      In WOM_THR (0x1F), set the WOM_Threshold [7:0] to 1~255 LSBs (0~1020mg)
 * 
 * Set Frequency of Wake-up:
 *      In LP_ACCEL_ODR (0x1E), set Lposc_clksel [3:0] = 0.24Hz ~ 500Hz
 * 
 * Enable Cycle Mode (Accel Low Power Mode):
 *      In PWR_MGMT_1 (0x6B) make CYCLE =1
 * 
 * Motion Interrupt Configuration Completed
 */

void setup() {

    Serial.begin(9600);

    // Step #1 - Make Sure Accel is running
    acc.reset();
    acc.enableCycle(false);
    acc.awake();
    acc.enableAxis(AccelerometerMPU9250::AXIS_XYZ);

    // Step #2 - Set Accel LPF setting to 184 Bandwidth:
    acc.setLowPassFilter(true, AccelerometerMPU9250::DR_184_5_80_250_1);

    // Step #3 - Enable Motion Interrupt:
    acc.enableInterrupt(AccelerometerMPU9250::WOM_EN);

    // Step #4 - Enable Accel Hardware Intelligence
    acc.enableWakeOnMotionDetection(true);
    acc.setAccelerationIntelligenceMode(AccelerometerMPU9250::ON);

    // Step #5 - Set Motion Threshold (any value)
    acc.setWakeOnMotionThreshold(0x55);

    // Step #6 - Set Frequency of Wake-up
    acc.setOutputDataRate(AccelerometerMPU9250::ODR_15_63HZ);

    // Step #7 - Enable Cycle Mode (Accel Low Power Mode)
    acc.enableCycle(true);

    // Write a Service Routine to Service the Interrupt
    attachInterrupt(0, isr, FALLING);
}

void isr() {
    ready = true;
}

void loop() {
    if (ready) {
        cli();
        ready = false;
        Serial.print("x: ");
        Serial.println(acc.readXg());
        Serial.print("y: ");
        Serial.println(acc.readYg());
        Serial.print("z: ");
        Serial.println(acc.readZg());
        Serial.println("-----------");
        sei();
    }
}
