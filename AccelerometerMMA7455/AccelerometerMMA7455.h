/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerMMA7455.h
 * 
 * The implementation of the MMA7455 accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_H__ 1

#include <Wire.h>
#include <Arduino.h>
#include <Accelerometer.h>

#undef INT1

class AccelerometerMMA7455 : public Accelerometer {
public:

    /**
     * Status Register
     * 
     * 0x09: STATUS Status Register
     */
    union STATUSbits {

        struct {
            unsigned char DRDY : 1;
            unsigned char DOVR : 1;
            unsigned char PERR : 1;
            unsigned char : 5;
        };
        unsigned char value;
    };

    /**
     * Detection Source Register
     * 
     * 0x0a: DETSRC Detection Source Register
     */
    union DETSRCbits {

        struct {
            unsigned char INT1 : 1;
            unsigned char INT2 : 1;
            unsigned char PDZ : 1;
            unsigned char PDY : 1;
            unsigned char PDX : 1;
            unsigned char LDZ : 1;
            unsigned char LDY : 1;
            unsigned char LDX : 1;
        };
        unsigned char value;
    };

    /**
     * Device Address
     * 
     * 0x0d: I2CAD Device Address
     */
    union I2CADbits {

        struct {
            unsigned char DVAD : 7;
            unsigned char I2CDIS : 1;
        };
        unsigned char value;
    };

    /**
     * Mode Control Register
     * 
     * 0x0d: MCTL Mode Control Register
     */
    union MCTLbits {

        struct {
            unsigned char MODE : 2;
            unsigned char GLVL : 2;
            unsigned char STON : 1;
            unsigned char SPI3W : 1;
            unsigned char DRPD : 1;
            unsigned char : 1;
        };
        unsigned char value;
    };
    
    /**
     * Interrupt Latch Reset
     * 
     * 0x0d: INTRST Interrupt Latch Reset
     */
    union INTRSTbits {

        struct {
            unsigned char CLR_INT1 : 1;
            unsigned char CLR_INT2 : 1;
            unsigned char : 6;
        };
        unsigned char value;
    };
    
    /**
     * Control 1
     * 
     * 0x0d: CTL1 Control 1
     */
    union CTL1bits {

        struct {
            unsigned char INTPIN : 1;
            unsigned char INTREG : 2;
            unsigned char XDA : 1;
            unsigned char YDA : 1;
            unsigned char ZDA : 1;
            unsigned char THOPT : 1;
            unsigned char DFBW : 1;
        };
        unsigned char value;
    };
    
    /**
     * Control 2
     * 
     * 0x0d: CTL2 Control 2
     */
    union CTL2bits {

        struct {
            unsigned char LDPL : 1;
            unsigned char PDPL : 1;
            unsigned char DRVO : 1;
            unsigned char : 5;
        };
        unsigned char value;
    };

    /**
     * Internal registers.
     */
    enum Location {
        
        XOUTL = 0x00,
        XOUTH = 0x01,
        YOUTL = 0x02,
        YOUTH = 0x03,
        ZOUTL = 0x04,
        ZOUTH = 0x05,
        XOUT8 = 0x06,
        YOUT8 = 0x07,
        ZOUT8 = 0x08,
        STATUS = 0x09,
        DETSRC = 0x0a,
        TOUT = 0x0b,
        RESERVED1 = 0x0c,
        I2CAD = 0x0d,
        USRINF = 0x0e,
        WHOAMI = 0x0f,
        XOFFL = 0x10,
        XOFFH = 0x11,
        YOFFL = 0x12,
        YOFFH = 0x13,
        ZOFFL = 0x14,
        ZOFFH = 0x15,
        MCTL = 0x16 ,
        INTRST = 0x17,
        CTL1 = 0x18,
        CTL2 = 0x19,
        LDTH = 0x1a,
        PDTH = 0x1b,
        PD = 0x1c,
        LT = 0x1d,
        TW = 0x1e,
        RESERVED2 = 0x1f
    };

    /**
     * Some useful masks.
     */
    enum Mask {
        STATUS_DRDY = 0x01,
        STATUS_DOVR = 0x02,
        STATUS_PERR = 0x04,
        
        DETSRC_INT1 = 0x01,
        DETSRC_INT2 = 0x02,
        DETSRC_PDZ = 0x04,
        DETSRC_PDY = 0x08,
        DETSRC_PDX = 0x10,
        DETSRC_LDZ = 0x20,
        DETSRC_LDY = 0x40,
        DETSRC_LDX = 0x80,
        
        I2CAD_DVAD = 0x7f,
        I2CAD_I2CDIS = 0x80,
        
        MCTL_MODE = 0x03,
        MCTL_GLVL = 0x0c,
        MCTL_STON = 0x10,
        MCTL_SPI3W = 0x20,
        MCTL_DRPD = 0x40,
        
        INTRST_CLR_INT1 = 0x01,
        INTRST_CLR_INT2 = 0x02,
        
        CTL1_INTPIN = 0x01,
        CTL1_INTREG = 0x06,
        CTL1_XDA = 0x08,
        CTL1_YDA = 0x10,
        CTL1_ZDA = 0x20,
        CTL1_THOPT = 0x40,
        CTL1_DFBW = 0x80,
        
        CTL2_LDPL = 0x01,
        CTL2_PDPL = 0x02,
        CTL2_DRVO = 0x04
    };

    /**
     * Device mode.
     * 
     * <pre>
     * MODE[1:0]    Function
     * 00           Standby Mode
     * 01           Measurement Mode
     * 10           Level Detection Mode
     * 11           Pulse Detection Mode
     * </pre>
     */
    enum DeviceMode {
        STANDBY_MODE = 0x00,
        MEASUREMENT_MODE = 0x01,
        LEVEL_DETECTION_MODE = 0x02,
        PULSE_DETECTION_MODE = 0x03
    };

    /**
     * Configuring the g-Select for 8-bit output using 
     * Register $16 with GLVL[1:0] bits
     * 
     * <pre>
     * GLVL[1:0]    g Range
     *      00      ±8g
     *      01      ±2g
     *      10      ±4g
     * </pre>
     */
    enum DynamicRange {
        DR_8G = 0x00,
        DR_2G = 0x01,
        DR_4G = 0x02
    };
    
    /**
     * DigitalFilterBandWidth
     */
    enum DigitalFilterBandWidth {
        DFBW_62_5_HZ = 0x00,
        DFBW_125_HZ = 0x01
    };

    /**
     * Configuring the Interrupt settings using 
     * Register $18 with INTREG[1:0] bits
     */
    enum InterruptConfiguration {
        INT1_LEVEL_INT2_PULSE = 0x00,
        INT1_PULSE_INT2_LEVEL = 0x01,
        INT1_S_PULSE_INT2_D_PULSE = 0x02
    };

    /**
     * Axis list
     */
    enum Axis {
        AXIS_X = 0x00,
        AXIS_Y = 0x01,
        AXIS_Z = 0x02
    };

    /**
     * Control 2 (Read/Write): Motion Detection (OR condition) or Freefall 
     * Detection (AND condition)
     * 
     * <pre>
     * PDPL
     * 0: Pulse detection polarity is positive and detecting condition is OR 3 axes.
     * 1: Pulse detection polarity is negative and detecting condition is AND 3 axes.
     * </pre>
     */
    enum DetectionCondition {
        MOTION_DETECTION = 0x00,
        FREEFALL_DETECTION = 0x01
    };
    
    /**
     * Public constructor.
     */
    AccelerometerMMA7455();

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

    /**
     */
    void readXYZ(unsigned char buf[6]);

    /**
     * Return true if the data is ready to be read.
     * 
     * @return 
     */
    bool isDataReady();

    /**
     * Device mode.
     * 
     * Activate or deactivate the device.
     * 
     * @param mode          A possible device mode.
     */
    void setDeviceMode(DeviceMode mode);

    /**
     * Put sensor into Standby Mode
     */
    void standby() {
        setDeviceMode(STANDBY_MODE);
    }
    
    /**
     * Uses 8 bit measurement
     */
    void setUse8bit(bool use);

    /**
     * Put sensor into Measurement Mode
     * 
     * Measurement Mode
     * 
     * The device can read XYZ measurements in this mode. The pulse and 
     * threshold interrupts are not active. During measurement mode, continuous 
     * measurements on all three axes enabled. The g-range for 2g, 4g, or 8g are
     * selectable with 8-bit data and the g-range of 8g is selectable with 
     * 10-bit data. The sample rate during measurement mode is 125 Hz with 62.5 
     * BW filter selected. 
     * 
     * The sample rate is 250 Hz with the 125 Hz filter selected. Therefore, 
     * when a conversion is complete (signaled by the DRDY flag), the next 
     * measurement will be ready. When measurements on all three axes are 
     * completed, a logic high level is output to the DRDY pin, indicating 
     * "measurement data is ready." The DRDY status can be monitored by the DRDY
     * bit in Status Register (Address: $09). The DRDY pin is kept high until 
     * one of the three Output Value Registers are read. If the next measurement
     * data is written before the previous data is read, the DOVR bit in the 
     * Status Register will be set. Also note that in measurement mode, level 
     * detection mode and pulse detection mode are not available.
     * 
     * By default all three axes are enabled. X and/or Y and/or Z can be 
     * disabled. There is a choice between detecting an absolute signal or a 
     * positive or negative only signal on the enabled axes. There is also a 
     * choice between doing a detection for motion where X or Y or Z > Threshold
     * vs. doing a detection for freefall where X & Y & Z < Threshold.
     */
    void measurementMode() {
        setDeviceMode(MEASUREMENT_MODE);
    }

    /**
     * Put sensor into Level Detection Mode
     * 
     * The user can access XYZ measurements and can use the level interrupt 
     * only. The level detection mechanism has no timers associated with it. 
     * Once a set acceleration level is reached the interrupt pin will go high 
     * and remain high until the interrupt pin is cleared (See Assigning, 
     * Clearing & Detecting Interrupts).
     */
    void levelDetectionMode() {
        setDeviceMode(LEVEL_DETECTION_MODE);
    }

    /**
     * Put sensor into Pulse Detection Mode
     * 
     * The user can access XYZ measurements and can use the level interrupt 
     * only. The level detection mechanism has no timers associated with it. 
     * Once a set acceleration level is reached the interrupt pin will go high 
     * and remain high until the interrupt pin is cleared (See Assigning, 
     * Clearing & Detecting Interrupts).
     */
    void pulseDetectionMode() {
        setDeviceMode(PULSE_DETECTION_MODE);
    }

    /**
     * Sets the detection condition.
     * 
     * Control 2 (Read/Write): 
     * Motion Detection (OR condition) or 
     * Freefall Detection (AND condition)
     * 
     * <pre>
     * PDPL
     * 0: Pulse detection polarity is positive and detecting condition is OR 3 axes.
     * 1: Pulse detection polarity is negative and detecting condition is AND 3 axes.
     * </pre>
     * 
     * @param condition         The detection condition.
     */
    void setDetectionCondition(DetectionCondition condition);
    
    /**
     * Set sensor to work in Ng range.
     * 
     * <pre>
     * 00: 8g is selected for measurement range.
     * 10: 4g is selected for measurement range.
     * 01: 2g is selected for measurement range
     * 
     * GLVL [1:0]   g-Range Sensitivity
     *      00      8g      16 LSB/g
     *      01      2g      64 LSB/g
     *      10      4g      32 LSB/g
     * </pre>
     */
    void setDynamicRange(DynamicRange range);

    /**
     * Disable interrupt.
     * 
     * @param axis              The axis to disable.
     */
    void enableInterrupt(Axis axis);

    /**
     * Enables interrupt.
     * 
     * @param axis              The axis to enable.
     */
    void disableInterrupt(Axis axis);

    /**
     * Sets the interrupt configuration;
     * 
     * @param configuration     The interrupt configuration.
     */
    void setInterruptConfiguration(InterruptConfiguration configuration);
    
    /**
     * After interrupt has triggered due to a detection, the interrupt pin 
     * (INT1 or INT2) need to be cleared by writing a logic 1. Then the 
     * interrupt pin should be enabled to trigger the next detection by setting 
     * it to a logic 0.
     */
    void clearInterruptLatch() {
        writeRegister(INTRST, 0x03);
        writeRegister(INTRST, 0x00);
    }

    /**
     * Writes into the sensor register.
     * 
     * @param reg       The new register.
     */
    void writeRegister(Location location, unsigned char v);

    /**
     * Reads the sensor register.
     * 
     * @return          The current register value.
     */
    unsigned char readRegister(Location location);

    /**
     * Writes a block of data into the device starting at the 'to' register.
     * 
     * @param to                The address to write.
     * @param buf               The buffer to be used.
     * @param len               The number of bytes to write.
     */
    void writeRegisterBlock(unsigned char to, unsigned char* buf, unsigned char len);

    /**
     * Reads a block of data from the device starting at the 'from' register.
     * 
     * @param from              The address to read.
     * @param buf               The buffer to be used.
     * @param len               The number of bytes to read.
     */
    void readRegisterBlock(unsigned char from, unsigned char* buf, unsigned char len);

    /**
     * Converts an array of chars into a float type.
     * 
     * @param buf               1 (8-bit) our 2 (10-bit) bytes to be converted.
     * @param is8bit            boolean indication if the date is 8 bit only
     */
    float convertToG(unsigned char* buf, bool is8bit);

    /**
     * Configures the register.
     * 
     * Basically it reads the register from the device. Applies the given 
     * mask on such register and makes an OR bitwise operation whit the
     * v value. 
     * 
     * (the v value will be masked to only use the bits of the 
     * corresponding mask).
     * 
     * @param reg
     * @param mask
     * @param v
     */
    void configureRegisterBits(Location location, Mask mask, unsigned char v);

protected:

    /**
     * The interruption 1 pin.
     */
    int int1Pin;

    /**
     * The interruption 2 pin.
     */
    int int2Pin;

    /**
     * The device address.
     */
    unsigned char address;
    
    /**
     * The current device mode control.
     */
    MCTLbits mctl;
    
    /**
     * use 8bit mode
     */
    bool use8bit;
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA7455_H__ */
