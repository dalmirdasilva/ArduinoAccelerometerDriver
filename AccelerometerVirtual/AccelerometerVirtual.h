/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerVirtual.h
 * 
 * Virtual implementation of the accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_VIRTUAL_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_VIRTUAL_H__ 1

#include "Accelerometer.h"

class AccelerometerVirtual : public Accelerometer {

public:

    /**
     * Public constructor.
     * 
     * @param xPin
     * @param yPin
     * @param zPin
     */
    AccelerometerVirtual();

    /**
     * Reads the x axis from the accelerometer device.
     * 
     * @retun   The x result.
     */
    virtual float readXg();

    /**
     * Reads the y axis from the accelerometer device.
     * 
     * @retun   The y result.
     */
    virtual float readYg();

    /**
     * Reads the z axis from the accelerometer device.
     * 
     * @retun   The z result.
     */
    virtual float readZg();
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_VIRTUAL_H__ */
