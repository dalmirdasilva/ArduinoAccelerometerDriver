/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerNunchuk.h
 * 
 * The implementation of the Nunchuk accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_NUNCHUK_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_NUNCHUK_H__ 1

#define ZEROX 0
#define ZEROY 0
#define ZEROZ 0

#include <Wire.h>
#include <Arduino.h>
#include <Accelerometer.h>

class AccelerometerNunchuk : public Accelerometer {
public:
    
    enum Axis {
        AXIS_X = 0x00,
        AXIS_Y = 0x01,
        AXIS_Z = 0x02
    };

    /**
     * Public constructor.
     * 
     * @param sa0           The LSBit of the address.
     */
    AccelerometerNunchuk();

    /**
     * Reads the z axis from the accelerometer device.
     * 
     * @retun               The z result.
     */
    float readZg(bool updateFrame);

    /**
     * Reads the z axis from the accelerometer device without refresh the frame.
     */
    virtual float readZg() {
        return readZg(false);
    }

    /**
     * Reads the x axis from the accelerometer device.
     * 
     * @param updateFrame   If the frame must be updated.
     * @retun               The x result.
     */
    float readXg(bool updateFrame);

    /**
     * Reads the x axis from the accelerometer device without refresh the frame.
     */
    virtual float readXg() {
        return readXg(false);
    }

    /**
     * Reads the y axis from the accelerometer device.
     * 
     * @param updateFrame   If the frame must be updated.
     * @retun               The y result.
     */
    float readYg(bool updateFrame);

    /**
     * Reads the y axis from the accelerometer device without refresh the frame.
     */
    virtual float readYg() {
        return readYg(false);
    }

    /**
     * Reads the Z button state.
     * 
     * @param updateFrame   If the frame must be updated.
     * @retun               The Z button state.
     */
    bool readZButton(bool updateFrame);
    
    /** 
     * Reads the Z button state without refresh the frame.
     */
    bool readZButton() {
        return readZButton(false);
    }

    /**
     * Reads the C button state.
     * 
     * @retun               The C result state.
     */
    bool readCButton(bool updateFrame);
    
    /** 
     * Reads the C button state without refresh the frame.
     */
    bool readCButton() {
        return readCButton(false);
    }
    
    /**
     * Reads the value of the Joystick X.
     * 
     * @param updateFrame          If the frame should be read to get fresh data.
     * @return                     The 0..255 data from Joystick X
     */
    unsigned char readXJoystick(bool updateFrame);
    
    /** 
     * Reads the value of the Joystick X without refresh the frame.
     */
    unsigned char readXJoystick() {
        return readXJoystick(false);
    }
    
    /**
     * Reads the value of the Joystick Y.
     * 
     * @param updateFrame          If the frame should be read to get fresh data.
     * @return                     The 0..255 data from Joystick Y
     */
    unsigned char readYJoystick(bool updateFrame);
    
    /** 
     * Reads the value of the Joystick Y without refresh the frame.
     */
    unsigned char readYJoystick() {
        return readYJoystick(false);
    }
    
    /**
     * Converts to G;
     * 
     * @param i             The integer to be converted.
     */
    float convertToG(unsigned int i);
    
    /**
     * Read a frame from the device.
     * 
     * Frame layout:
     * 
     * <pre>
     * 
     * Byte  |Function
     * ------|---------------------------------------------------
     * 0     |Joystick X
     * 1     |Joystick Y
     * 2     |Accelerometer X Bits 9 a 2 (10 bits)
     * 3     |Accelerometer Y Bits 9 a 2 (10 bits)
     * 4     |Accelerometer Z Bits 9 a 2 (10 bits)
     *
     * 5     |Acel Z|Acel Z|Acel Y|Acel Y|Acel X|Acel X|Bt C|Bt Z|
     *       |bit 1 |bit 0 |bit 1 |bit 0 |bit 1 |bit 0 |    |    |
     * 
     * </pre>
     */
    void readFrame();
    
    /**
     * Initializes the device.
     */
    void begin();
    
    /**
     * Reads the acceleration of the given axis.
     * 
     * @param axis                  The axis to read.
     * @param updateFrame           If the frame must be updated.
     * @return                      The acceleration.
     */
    unsigned int readAcceleration(Axis axis, bool updateFrame);

protected:

    /**
     * The device address.
     */
    unsigned char address;
    
    /**
     * A frame.
     */
    unsigned char initializationSequence[2];
    
    /**
     * A frame.
     */
    unsigned char frame[6];
    
    /**
     * Decodes the data from device.
     */
    unsigned char decode(unsigned char b);
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_NUNCHUK_H__ */
