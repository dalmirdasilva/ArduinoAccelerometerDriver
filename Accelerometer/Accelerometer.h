/**
 * Arduino - Accelerometer driver
 * 
 * Accelerometer.h
 * 
 * The header file for the accelerometer driver
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_H__ 1

/**
 * An accelerometer is a device that measures proper acceleration ("g-force").
 * Proper acceleration is not the same as coordinate acceleration (rate of change of velocity).
 * For example, an accelerometer at rest on the surface of the Earth will measure an
 * acceleration g= 9.81 m/s2 straight upwards. By contrast, accelerometers in free fall
 * orbiting and accelerating due to the gravity of Earth will measure zero.
 */
class Accelerometer {

public:

    /**
     * Reads the x axis from the accelerometer device.
     * 
     * @retun   The x result.
     */
    virtual float readXg() = 0;
    
    /**
     * Reads the y axis from the accelerometer device.
     * 
     * @retun   The y result.
     */
    virtual float readYg() = 0;
    
    /**
     * Reads the z axis from the accelerometer device.
     * 
     * @retun   The z result.
     */
    virtual float readZg() = 0;
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_H__ */
