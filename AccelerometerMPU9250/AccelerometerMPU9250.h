/**
 * Arduino - Accelerometer driver
 *
 * AccelerometerMPU9250.h
 *
 * The implementation of the MPU9250 accelerometer.
 *
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_MPU9250_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_MPU9250_H__ 1

#include <Wire.h>
#include <Arduino.h>
#include <Accelerometer.h>
#include <RegisterBasedWiredDevice.h>

#define MPU9250_ADDRESS 0x68

/**
 * 2.2 Accelerometer Features
 * The triple-axis MEMS accelerometer in MPU-9250 includes a wide range of features:
 * • DDigital-output triple-axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g and ±16g and integrated 16-bit ADCs
 * • Accelerometer normal operating current: 450μA
 * • Low power accelerometer mode current: 8.4μA at 0.98Hz, 19.8μA at 31.25Hz
 * • Sleep mode current: 8μA
 * • User-programmable interrupts
 * • Wake-on-motion interrupt for low power operation of applications processor
 * • Self-test
 */
class AccelerometerMPU9250: public Accelerometer, public RegisterBasedWiredDevice {
public:

    enum Register {
        ACCEL_CONFIG = 0x1c,
        ACCEL_CONFIG2 = 0x1d,
        LP_ACCEL_ODR = 0x1e,
        WOM_THR = 0x1f,
        FIFO_EN = 0x23,
        ACCEL_XOUT_H = 0x3b,
        ACCEL_XOUT_L = 0x3c,
        ACCEL_YOUT_H = 0x3d,
        ACCEL_YOUT_L = 0x3e,
        ACCEL_ZOUT_H = 0x3f,
        ACCEL_ZOUT_L = 0x40,
        INT_ENABLE = 0x38,
        INT_STATUS = 0x3a,
        MOT_DETECT_CTRL = 0x69,
        PWR_MGMT_1 = 0x6b,
        PWR_MGMT_2 = 0x6c
    };

    /**
     * Accelerometer Configuration (ACCEL_CONFIG 0x1c)
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union ACCEL_CONFIGbits {
        struct {
            unsigned char :3;
            unsigned char ACCEL_FS_SEL :2;
            unsigned char AZ_ST_EN :1;
            unsigned char AY_ST_EN :1;
            unsigned char AX_ST_EN :1;
        };
        unsigned char value;
    };

    /**
     * Accelerometer Configuration 2 (ACCEL_CONFIG2 0x1d)
     *
     * ACCEL_FCHOICE_B is used to bypass DLPF as shown in table 2 below. NOTE: This register
     * contains accel_fchoice_b (the inverted version of accel_fchoice as described in the table below).
     *
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union ACCEL_CONFIG2bits {
        struct {
            unsigned char A_DLPF_CFG :3;
            unsigned char ACCEL_FCHOICE_B :1;
            unsigned char :4;
        };
        unsigned char value;
    };

    /**
     * Low Power Accelerometer ODR Control (LP_ACCEL_ODR 0x1e)
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union LP_ACCEL_ODRbits {
        struct {
            unsigned char LPOSC_CLKSEL :4;
            unsigned char :4;
        };
        unsigned char value;
    };

    /**
     * FIFO Enable (FIFO_EN 0x23)
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union FIFO_ENbits {
        struct {
            unsigned char SLV_0 :1;
            unsigned char SLV_1 :1;
            unsigned char SLV_2 :1;
            unsigned char ACCEL :1;
            unsigned char GYRO_ZOUT :1;
            unsigned char GYRO_YOUT :1;
            unsigned char GYRO_XOUT :1;
            unsigned char TEMP_OUT :1;
        };
        unsigned char value;
    };

    /**
     * Interrupt Enable (INT_ENABLE 0x38)
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union INT_ENABLEbits {
        struct {
            unsigned char RAW_RDY_EN :1;
            unsigned char :2;
            unsigned char FSYNC_INT_EN :1;
            unsigned char FIFO_OVERFLOW_EN :1;
            unsigned char :1;
            unsigned char WOM_EN :1;
            unsigned char :1;
        };
        unsigned char value;
    };

    /**
     * Interrupt Status (INT_STATUS 0x3a)
     * Serial IF: R/C
     * Reset value: 0x00
     */
    union INT_STATUSbits {
        struct {
            unsigned char RAW_DATA_RDY_INT :1;
            unsigned char :2;
            unsigned char FSYNC_INT :1;
            unsigned char FIFO_OVERFLOW_INT :1;
            unsigned char :1;
            unsigned char WOM_INT :1;
            unsigned char :1;
        };
        unsigned char value;
    };

    /**
     * Accelerometer Interrupt Control
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union MOT_DETECT_CTRLbits {
        struct {
            unsigned char :6;
            unsigned char ACCEL_INTEL_MODE :1;
            unsigned char ACCEL_INTEL_EN :1;
        };
        unsigned char value;
    };

    /**
     * Power Management 1
     * Serial IF: R/W
     * Reset value: (Depends on PU_SLEEP_MODE bit, see below)
     */
    union PWR_MGMT_1bits {
        struct {
            unsigned char CLKSEL :3;
            unsigned char PD_PTAT :1;
            unsigned char GYRO_STANDBY :1;
            unsigned char CYCLE :1;
            unsigned char SLEEP :1;
            unsigned char H_RESET :1;
        };
        unsigned char value;
    };

    /**
     * Power Management 2
     * Serial IF: R/W
     * Reset value: 0x00
     */
    union PWR_MGMT_2bits {
        struct {
            unsigned char DISABLE_ZG :1;
            unsigned char DISABLE_YG :1;
            unsigned char DISABLE_XG :1;
            unsigned char DISABLE_ZA :1;
            unsigned char DISABLE_YA :1;
            unsigned char DISABLE_XA :1;
            unsigned char :2;
        };
        struct {
            unsigned char DISABLE_G :3;
            unsigned char DISABLE_A :3;
            unsigned char :2;
        };
        unsigned char value;
    };

    enum BusMode {
        MASTER_MODE = 0x00,
        PASS_THROUGH_MODE = 0x01
    };

    /**
     * Full Scale Selection
     *
     * <pre>
     * FS1 FS0  g Range
     * 0    0   +/-2g
     * 0    1   +/-4g
     * 1    0   +/-8g
     * 1    1   +/-16g
     * </pre>
     */
    enum FullScaleRange {
        FS_SEL_2G = 0x00,
        FS_SEL_4G = 0x08,
        FS_SEL_8G = 0x10,
        FS_SEL_16G = 0x18
    };

    /**
     * Accelerometer Data Rates and Bandwidths (Normal Mode)
     *
     * ACCEL_FCHOICE A_DLPF_CFG  Bandwidth (Hz)  Delay (ms)  Noise Density (ug/rtHz) Rate (kHz)
     * 0   X   1.13K   0.75    250   4
     * 1   0   460     1.94    250   1
     * 1   1   184     5.80    250   1
     * 1   2   92      7.80    250   1
     * 1   3   41      11.80   250   1
     * 1   4   20      19.80   250   1
     * 1   5   10      35.70   250   1
     * 1   6   5       66.96   250   1
     * 1   7   460     1.94    250   1
     */
    enum LowPassFilter {
        DR_460_1_94_250_1 = 0x00,
        DR_184_5_80_250_1 = 0x01,
        DR_92_7_80_250_1 = 0x01,
        DR_41_11_80_250_1 = 0x03,
        DR_20_19_80_250_1 = 0x04,
        DR_10_35_70_250_1 = 0x05,
        DR_5_66_96_250_1 = 0x06
    };

    /**
     * Low Power Accelerometer ODR Control
     *
     * <pre>
     * DR2  DR1 DR0 Output      Data Rate (ODR) Time Between Data Samples
     * 0    0   0   800 Hz      1.25 ms
     * 0    0   1   400 Hz      2.5 ms
     * 0    1   0   200 Hz      5 ms
     * 0    1   1   100 Hz      10 ms
     * 1    0   0   50  Hz      20  ms
     * 1    0   1   12.5 Hz     80 ms
     * 1    1   0   6.25 Hz     160 ms
     * 1    1   1   1.563 Hz    640 ms
     * </pre>
     */
    enum OutputDataRate {
        ODR_0_24HZ = 0x00,
        ODR_0_49HZ = 0x01,
        ODR_0_98HZ = 0x02,
        ODR_1_95HZ = 0x03,
        ODR_3_91HZ = 0x04,
        ODR_7_81HZ = 0x05,
        ODR_15_63HZ = 0x06,
        ODR_31_25HZ = 0x07,
        ODR_62_50HZ = 0x08,
        ODR_125HZ = 0x09,
        ODR_250HZ = 0x0a,
        ODR_500HZ = 0x0b
    };

    /**
     * Interrupt enable flags.
     */
    enum Interrupt {
        RAW_RDY_EN = 0x01,
        FSYNC_INT_EN = 0x08,
        FIFO_OVERFLOW_EN = 0x10,
        WOM_EN = 0x40
    };

    /**
     * Some useful masks.
     */
    enum Mask {
        ACCEL_CONFIG_ACCEL_FS_SEL = 0x18,
        ACCEL_CONFIG2_A_DLPF_CFG = 0x07,
        LP_ACCEL_ODR_LPOSC_CLKSEL = 0x0f,
        FIFO_EN_ACCEL = 0x80,
        PWR_MGMT_1_H_RESET = 0x80,
        PWR_MGMT_1_SLEEP = 0x40,
        PWR_MGMT_1_CYCLE = 0x20,
        PWR_MGMT_2_DISABLE_A = 0x38,
        MOT_DETECT_CTRL_ACCEL_INTEL_EN = 0x80,
        MOT_DETECT_CTRL_ACCEL_INTEL_MODE = 0x40
    };

    /**
     * Axis
     */
    enum Axis {
        AXIS_NONE = 0x00,
        AXIS_X = 0x20,
        AXIS_Y = 0x10,
        AXIS_Z = 0x08,
        AXIS_XY = AXIS_X | AXIS_Y,
        AXIS_XZ = AXIS_X | AXIS_Z,
        AXIS_YZ = AXIS_Y | AXIS_Z,
        AXIS_XYZ = AXIS_X | AXIS_Y | AXIS_Z
    };

    /**
     * Accelerometer Hardware Intelligence Mode
     */
    enum AccelerationIntelligenceMode {
        OFF = 0x00,
        ON = 0x01
    };

    /**
     * Public constructor.
     *
     * @param ad0   LSBit of the device address.
     */
    AccelerometerMPU9250(bool ad0);

    /**
     * Generic method to convert an axis to g.
     *
     * @param axisRegister              One of 3 possible register which contains the axis data (ACCEL_XOUT_H, ACCEL_YOUT_H or ACCEL_ZOUT_H)
     */
    float readAxisGravity(unsigned char axisRegister);

    /**
     * Reads the x axis from the accelerometer device.
     *
     * @return   The x result.
     */
    float readXg();

    /**
     * Reads the y axis from the accelerometer device.
     *
     * @return   The y result.
     */
    float readYg();

    /**
     * Reads the z axis from the accelerometer device.
     *
     * @return   The z result.
     */
    float readZg();

    /**
     * Reads all 6 register from ACCEL_XOUT_H to ACCEL_ZOUT_L
     *
     * @param buf   Where acceleration data will placed.
     */
    void readXYZ(unsigned char* buf);

    /**
     * Accel Full Scale Select
     *
     * We store the fs info in memory in order to be used
     * when converting raw data in g.
     *
     * @param fs    FullScale
     * @see ACCEL_CONFIG
     */
    void setFullScaleRange(FullScaleRange fsr);

    /**
     * Set low pass filter setting
     *
     * @param enable    Enable low pass filter when 1.
     * @param lpf       A low pass filter setting.
     * @see ACCEL_CONFIG2
     */
    void setLowPassFilter(bool enable, LowPassFilter lpf);

    /**
     * Set the output data rate.
     *
     * @param odr       Output data rate.
     * @see
     */
    void setOutputDataRate(OutputDataRate odr);

    /**
     * Enables/Disable acceleration axis.
     *
     * Each bit corresponds to PWR_MGMT_2. As the bits in PWR_MGMT_2 are
     * disable the acceleration when set, before send axis to the register
     * we invert all bits. Turning on all that are off.
     */
    void enableAxis(Axis axis);

    /**
     * This register holds the threshold value for the Wake on Motion Interrupt for accelerometer
     * x/y/z axes. LSB = 4mg. Range is 0mg to 1020mg.
     */
    void setWakeOnMotionThreshold(unsigned char womt);

    /**
     * Enables the Wake-on-Motion detection logic.
     *
     * @param enable    Enables the Wake-on-Motion detection logic.
     * @see ACCEL_INTEL_CTRL
     */
    void enableWakeOnMotionDetection(bool enable);

    /**
     * Set Hardware Acceleration Intelligence Mode
     *
     * @param mode  ON: Compare the current sample with the previous sample.
     *              OFF: Not used.
     */
    void setAccelerationIntelligenceMode(AccelerationIntelligenceMode mode);

    /**
     * When set, and SLEEP and STANDBY are not set, the chip will cycle
     * between sleep and taking a single sample at a rate determined by
     * LP_ACCEL_ODR register
     *
     * @param enable    True enables, false disables.
     * @see PWR_MGMT_1
     */
    void enableCycle(bool enable);

    /**
     * Reset the internal registers and restores the default settings.
     */
    void reset();

    /**
     * Turn device in sleep mode.
     */
    void sleep();

    /**
     * Recover device from sleep mode.
     */
    void awake();

    /**.
     * Enable and disable FIFO
     *
     * @param en    1 write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H,
     *                  and ACCEL_ZOUT_L to the FIFO at the sample rate;
     *              0 – function is disabled
     */
    void enableFifo(bool en);

    /**
     * Enable interrupt.
     *
     * @param interrupt     One possible interrupt from Interrupt enum.
     */
    void enableInterrupt(Interrupt interrupt);

    /**
     * Disable interrupt.
     *
     * @param interrupt     One possible interrupt from Interrupt enum.
     */
    void disableInterrupt(Interrupt interrupt);

    /**
     * Converts raw acceleration read to g.
     *
     * @param buf       2 bytes from acceleration output.
     */
    float convertToG(unsigned char buf[2]);

private:

    /**
     * Keeps the value of ACCEL_CONFIG register in memory.
     *
     * This register holds the Full Scale information. When converting
     * the raw data in g, we need that information. Keeping in memory
     * avoids an unnecessary read to the device to retrieve the current FS info.
     */
    ACCEL_CONFIGbits config;
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MPU9250_H__ */

