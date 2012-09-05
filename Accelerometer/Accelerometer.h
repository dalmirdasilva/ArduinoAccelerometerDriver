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

class Accelerometer {

protected:

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
     * Protected constructor.
     * 
     * @param xPin      The x pin.
     * @param yPin      The y pin.
     * @param zPin      The z pin.
     */
    Accelerometer(int xPin, int yPin, int zPin);

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
