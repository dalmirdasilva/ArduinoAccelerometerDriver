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

class AccelerometerADXL335 : public Accelerometer {

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
