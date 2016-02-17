/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerADXL335.h
 * 
 * The implementation of the ADXL335 accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_ADXL335_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_ADXL335_H__ 1

#include "Accelerometer.h"

/**
 * The ADXL335 is a complete 3-axis acceleration measurement
 * system. The ADXL335 has a measurement range of ±3 g minimum.
 * It contains a polysilicon surface-micromachined sensor
 * and signal conditioning circuitry to implement an open-loop
 * acceleration measurement architecture. The output signals are
 * analog voltages that are proportional to acceleration. The
 * accelerometer can measure the static acceleration of gravity
 * in tilt-sensing applications as well as dynamic acceleration
 * resulting from motion, shock, or vibration.
 *
 * The sensor is a polysilicon surface-micromachined structure
 * built on top of a silicon wafer. Polysilicon springs suspend the
 * structure over the surface of the wafer and provide a resistance
 * against acceleration forces. Deflection of the structure is measured
 * using a differential capacitor that consists of independent
 * fixed plates and plates attached to the moving mass. The fixed
 * plates are driven by 180° out-of-phase square waves. Acceleration
 * deflects the moving mass and unbalances the differential capacitor
 * resulting in a sensor output whose amplitude is proportional to
 * acceleration. Phase-sensitive demodulation techniques are then
 * used to determine the magnitude and direction of the
 * acceleration.
 *
 * The demodulator output is amplified and brought off-chip
 * through a 32 kΩ resistor. The user then sets the signal
 * bandwidth of the device by adding a capacitor. This filtering
 * improves measurement resolution and helps prevent aliasing.
 */
class AccelerometerADXL335: public Accelerometer {

    /**
     * The x pin.
     */
    int xPin;

    /**
     * The y pin.
     */
    int yPin;

    /**
     * The z pin.
     */
    int zPin;

    /**
     * The Arduino power supply voltage.
     */
    float arduinoPowerSupply;

    /**
     * The sensor power supply voltage.
     */
    float sensorPowerSupply;

    /**
     * The zero g bias output is also ratiometric, thus the zero g
     * output is nominally equal to VS/2 at all supply voltages.
     */
    float zeroGBias;

    /**
     * Reads an analog value from pin.
     * 
     * @param pin
     * @return 
     */
    float readPin(int pin);

public:

    /**
     * Public constructor.
     * 
     * @param xPin
     * @param yPin
     * @param zPin
     */
    AccelerometerADXL335(int xPin, int yPin, int zPin);

    /**
     * Reads the x axis from the accelerometer device.
     * 
     * @return   The x result.
     */
    virtual float readXg();

    /**
     * Reads the y axis from the accelerometer device.
     * 
     * @return   The y result.
     */
    virtual float readYg();

    /**
     * Reads the z axis from the accelerometer device.
     * 
     * @return   The z result.
     */
    virtual float readZg();
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_ADXL335_H__ */
