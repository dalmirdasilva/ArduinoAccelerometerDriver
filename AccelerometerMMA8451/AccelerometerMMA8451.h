/**
 * Arduino - Accelerometer driver
 * 
 * AccelerometerMMA8451.h
 * 
 * The implementation of the MMA8451 accelerometer.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_H__ 1

#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.h"

class AccelerometerMMA8451 : public Accelerometer {

    /**
     * The scl pin.
     */
    int sclPin;

    /**
     * The sda pin.
     */
    int sdaPin;

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
     * Internal registers.
     */
    enum Register {
        STATUS = 0x00,
        OUT_X_MSB = 0x01,
        OUT_X_LSB = 0x02,
        OUT_Y_MSB = 0x03,
        OUT_Y_LSB = 0x04,
        OUT_Z_MSB = 0x05,
        OUT_Z_LSB = 0x06,
        F_SETUP = 0x09,
        TRIG_CFG = 0x0a,
        SYSMOD = 0x0b,
        INT_SOURCE = 0x0c,
        WHO_AM_I = 0x0d,
        XYZ_DATA_CFG = 0x0e,
        HP_FILTER_CUTOFF = 0x0f,
        PL_STATUS = 0x10,
        PL_CFG = 0x11,
        PL_COUNT = 0x12,
        PL_BF_ZCOMP = 0x13,
        P_L_THS_REG = 0x14,
        FF_MT_CFG = 0x15,
        FF_MT_SRC = 0x16,
        FF_MT_THS = 0x17,
        FF_MT_COUNT = 0x18,
        TRANSIENT_CFG = 0x1d,
        TRANSIENT_SRC = 0x1e,
        TRANSIENT_THS = 0x1f,
        TRANSIENT_COUNT = 0x20,
        PULSE_CFG = 0x21,
        PULSE_SRC = 0x22,
        PULSE_THSX = 0x23,
        PULSE_THSY = 0x24,
        PULSE_THSZ = 0x25,
        PULSE_TMLT = 0x26,
        PULSE_LTCY = 0x27,
        PULSE_WIND = 0x28,
        ASLP_COUNT = 0x29,
        CTRL_REG1 = 0x2a,
        CTRL_REG2 = 0x2b,
        CTRL_REG3 = 0x2c,
        CTRL_REG4 = 0x2d,
        CTRL_REG5 = 0x2e,
        OFF_X = 0x2f,
        OFF_Y = 0x30,
        OFF_Z = 0x31
    };

    /**
     * Some used masks.
     */
    enum Mask {
        ACTIVE_MASK = 0x01,
        FS_MASK = 0x03,
        ODR_MASK = 0x38,
        MODS_MASK = 0x03,
        SEL_MASK = 0x03,
        HPF_OUT_MASK = 0x10,
        ZYXDR_MASK = 0x08
    };

    /**
     * Enable/Disable device.
     */
    enum DeviceActivation {
        DEACTIVATE = 0x00,
        ACTIVATE = 0x01
    };

    /**
     * Dynamic range.
     * Table 3. Full Scale Selection
     * FS1 FS0  g Range
     * 0    0   ±2g
     * 0    1   ±4g
     * 1    0   ±8g
     * 1    1   —
     */
    enum DynamicRange {
        DR_2G = 0x00,
        DR_4G = 0x01,
        DR_8G = 0x02
    };

    /**
     * Table 5. Output Data Rates
     * 
     * DR2  DR1 DR0 Output      Data Rate (ODR) Time Between Data Samples
     * 0    0   0   800 Hz      1.25 ms
     * 0    0   1   400 Hz      2.5 ms
     * 0    1   0   200 Hz      5 ms
     * 0    1   1   100 Hz      10 ms
     * 1    0   0   50  Hz      20  ms
     * 1    0   1   12.5 Hz     80 ms
     * 1    1   0   6.25 Hz     160 ms
     * 1    1   1   1.563 Hz    640 ms
     * 
     */
    enum OutputDataRate {
        ODR_800HZ_1_25_MS = 0x00,
        ODR_400HZ_2_5_MS = 0x01,
        ODR_200HZ_5_MS = 0x02,
        ODR_100HZ_10_MS = 0x03,
        ODR_50HZ_20_MS = 0x04,
        ODR_12_5HZ_80_MS = 0x05,
        ODR_6_25HZ_1_160_MS = 0x06,
        ODR_1_563HZ_1_640_MS = 0x07
    };

    /**
     * Oversampling Mode.
     * 
     * MODS1    MODS0 
     * 0        0       Normal
     * 0        1       Low noise, low power
     * 1        0       High resolution
     * 1        1       Low power
     */
    enum OversamplingMode {
        NORMAL_MODS = 0x00,
        LOW_NOISE_LOW_POWER_MODS = 0x01,
        HI_RESOLUTION_MODS = 0x02,
        LOW_POWER_MODS = 0x03
    };

    /**
     * See Table 8. HP_FILTER_CUTOFF Setting Options.
     */
    enum HighPassFilterCutoffFrequency {
        PH_FILTER_CUTOFF_0 = 0x00,
        PH_FILTER_CUTOFF_1 = 0x01,
        PH_FILTER_CUTOFF_2 = 0x02,
        PH_FILTER_CUTOFF_3 = 0x03
    };
    
    /**
     * Holds the current device state.
     * 
     * It is important to hold this on the object to avoid
     * unnecessary read operations on the device.
     */
    DeviceActivation activation;
    
    /**
     * Holds the current Dynamic Range of the device.
     */
    DynamicRange range;

public:

    /**
     * Public constructor.
     * 
     * @param sclPin
     * @param sdaPin
     * @param int1Pin
     * @param int2Pin
     */
    AccelerometerMMA8451(unsigned char sa0, int sclPin, int sdaPin, int int1Pin, int int2Pin);

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
     * The MMA8451Q has 14-bit XYZ data. The MMA8452Q has 12-bit XYZ data and 
     * the MMA8453 has 10-bit data. This section is an overview of how to 
     * manipulate the data to continuously burst out 14-bit data in different 
     * data formats from the MCU. The examples will be shown for the 14-bit data
     * but the reader can understand what changes would be made for the 12-bit 
     * data or the 10-bit data. The driver code has all the functions for all 
     * data formats available. The event flag can be monitored by reading the 
     * STATUS register (0x00). This can be done by using either a polling or 
     * interrupt technique, which is discussed later in Section 9.0 of this 
     * document. It is not absolutely necessary to read the STATUS register to 
     * clear it. Reading the data clears the STATUS register. Table 9. 0x00 
     * STATUS: Data Status Registers (Read Only) 
     * Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
     * ZYXOW ZOW YOW XOW ZYXDR ZDR YDR XDR
     * The ZYXDR flag is set whenever there is new data available in any axis. 
     * The following code example monitors this flag and, upon the detection of 
     * new data, reads the 14/12/10-bit XYZ data into an array (value[]) in RAM 
     * with a single, multi-byte I2C access. These values are then copied into 
     * 16-bit variables prior to further processing.
     */
    void readXYZ(unsigned char buf[6]);
    
    /**
     * Return true if the data is ready to be read.
     * 
     * @return 
     */
    boolean isDataReady();

    /**
     * Device activation.
     * 
     * Activate or deactivate the device.
     * 
     * @param activation
     */
    void deviceActivation(DeviceActivation activation);

    /**
     * Put sensor into Standby Mode
     * 
     * Read current value of System Control 1 Register.
     * Put sensor into Standby Mode by clearing the Active bit
     * Return with previous value of System Control 1 Register.
     */
    inline void standby() {
        deviceActivation(DEACTIVATE);
    }

    /**
     * Put sensor into Active Mode
     * 
     * Read current value of System Control 1 Register.
     * Put sensor into Active Mode by setting the Active bit
     * Return with previous value of System Control 1 Register.
     */
    inline void active() {
        deviceActivation(ACTIVATE);
    }

    /**
     * Set sensor to work in Ng range by changing the FS1 and the FS0.
     * 
     * There are 3 different dynamic ranges that can be set (2g, 4g, 8g).
     * The dynamic range is changeable only in the Standby Mode. The dynamic 
     * range is controlled by setting the FS0 and FS1 bits in register 0x0E. 
     * The device changes from Standby to Active Mode via bit 0 in register 
     * 0x2A.
     * 
     * Table 3. Full Scale Selection
     * 
     * FS1  FS0 g Range
     * 0    0   ±2g
     * 0    1   ±4g
     * 1    0   ±8g
     * 1    1   —
     */
    void setDynamicRange(DynamicRange range);

    /**
     * Sets the Output Data Rate.
     * 
     * The active mode Output Data Rate (ODR) and Sleep Mode Data Rate are 
     * programmable via other control bits in the CTRL_REG1 register, seen in 
     * Table 4. Unless the sleep mode is enabled the active mode data rate is 
     * the data rate that will always be enabled. Table 5 shows how the DR2:DR0 
     * bits affect the ODR. These are the active mode data rates available. The 
     * default data rate is DR = 000, 800 Hz.
     * 
     * @param odr               The Output Data Rate
     */
    void setOutputDataRate(OutputDataRate rate);

    /**
     * Sets the oversampling mode.
     * 
     * There are four different oversampling modes. There is a normal mode, a 
     * low noise + power mode, a high-resolution mode and a low-power mode. 
     * The difference between these are the amount of averaging of the sampled 
     * data, which is done internal to the device. The following chart shows 
     * the amount of averaging at each data rate, which is the OSRatio 
     * (oversampling ratio). There is a trade-off between the oversampling and 
     * the current consumption at each ODR value.
     * 
     * @param om
     */
    void setOversamplingMode(OversamplingMode mode);

    /**
     * Sets the High-Pass Filter Cutoff Frequency.
     * 
     * The HP_FILTER_CUTOFF register (at 0x0F) sets the high-pass cutoff 
     * frequency, Fc, for the data. The output of this filter is provided in the
     * output data registers (0x01 to 0x06). Note that the high-pass filtered 
     * output data is available for the MMA8451Q and the MMA8452Q only. 
     * The MMA8453Q has the internal high-pass filter for the embedded functions
     * but does not have access to the output data. The available cutoff 
     * frequencies change depending upon the set Output Data Rate.
     * 
     * @param hpf                   The High-Pass Filter Cutoff Frequency.
     */
    void setHighPassFilterCutoffFrequency(HighPassFilterCutoffFrequency frequency);

    /**
     * Registers 0x01 through 0x06 are used to read the X, Y, Z data. The device
     * can be configured to produce high-pass filtered data or low-pass filtered
     * data by setting or clearing the HPF_Out bit in the XYZ_Data_Cfg Register 
     * 0x0E. The following code example shows how to set the HPF_Out bit.
     */
    void highPassFilteredData(boolean filtered);
    
    /**
     * Writes into the sensor register.
     * 
     * @param reg       The new register.
     */
    void writeRegister(Register reg, unsigned char v);

    /**
     * Reads the sensor register.
     * 
     * @return          The current register value.
     */
    unsigned char readRegister(Register reg);

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

private:

    /**
     * Converts an array of chars into a float type.
     * 
     * Assuming:
     * rep:  S III FFFF FFFF FFFF
     * bits: b bbb bbbb bbbb bbbb
     * 
     * S = Signal
     * I = Integer part
     * F = Fractional part
     * 
     * With:
     * S III FFFF FFFF FFFF
     * 1 101 1001 1100 1010
     * 
     * We have:
     * S = -
     * I = 5
     * F = 1/2 + 1/16 + 1/32 + 1/64 + 1/512 + 1/2048 = 611816406
     * 
     * Result: -5.611816406
     */
    float convertToG(unsigned char buf[2]);

    /**
     * 
     * @param reg
     * @param mask
     * @param v
     */
    void configureRegisterBits(Register reg, Mask mask, unsigned char v);};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_H__ */
