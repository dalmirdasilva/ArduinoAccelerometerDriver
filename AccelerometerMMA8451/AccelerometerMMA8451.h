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

#include <Wire.h>
#include <Arduino.h>
#include <Accelerometer.h>
#include <RegisterBasedWiredDevice.h>

#define MMA8451_ADDRESS 0x1c

class AccelerometerMMA8451: public Accelerometer, public RegisterBasedWiredDevice {
public:

    /**
     * 0x00 STATUS: Data Status Register (Read Only)
     * When F_MODE == 0
     */
    union STATUSbits {

        struct {
            unsigned char XDR :1;
            unsigned char YDR :1;
            unsigned char ZDR :1;
            unsigned char ZYXDR :1;
            unsigned char XOW :1;
            unsigned char YOW :1;
            unsigned char ZOW :1;
            unsigned char ZYXOW :1;
        };
        unsigned char value;
    };

    /**
     * 0x00 F_STATUS: FIFO Status Register
     * When F_MODE > 0
     */
    union F_STATUSbits {

        struct {
            unsigned char F_CNT :6;
            unsigned char F_WMRK_FLAG :1;
            unsigned char F_OVF :1;
        };
        unsigned char value;
    };

    /**
     * FIFO Setup Register
     *
     * 0x09 F_SETUP: FIFO Setup Register (Read/Write)
     */
    union F_SETUPbits {

        struct {
            unsigned char F_WMRK :6;
            unsigned char F_MODE :2;
        };
        unsigned char value;
    };

    /**
     * 0x0A: TRIG_CFG Trigger Configuration Register (Read/Write)
     */
    union TRIG_CFGbits {

        struct {
            unsigned char :2;
            unsigned char TRIG_FF_MT :1;
            unsigned char TRIG_PULSE :1;
            unsigned char TRIG_LNDPRT :1;
            unsigned char TRIG_TRANS :1;
            unsigned char :2;
        };
        unsigned char value;
    };

    /**
     * System Mode Register
     *
     * 0x0B SYSMOD: System Mode Register (Read Only)
     */
    union SYSMODbits {

        struct {
            unsigned char SYSMOD :2;
            unsigned char FGT :5;
            unsigned char FGERR :1;
        };
        unsigned char value;
    };

    /**
     * System Interrupt Status Register
     *
     * 0x0C: INT_SOURCE System Interrupt Status Register
     */
    union INT_SOURCEbits {

        struct {
            unsigned char SRC_DRDY :1;
            unsigned char :1;
            unsigned char SRC_FF_MT :1;
            unsigned char SRC_PULSE :1;
            unsigned char SRC_LNDPRT :1;
            unsigned char SRC_TRANS :1;
            unsigned char SRC_FIFO :1;
            unsigned char SRC_ASLP :1;
        };
        unsigned char value;
    };

    /**
     * 0x0E: XYZ_DATA_CFG (Read/Write)
     */
    union XYZ_DATA_CFGbits {

        struct {
            unsigned char FS :2;
            unsigned char :2;
            unsigned char HPF_OUT :1;
            unsigned char :3;
        };
        unsigned char value;
    };

    /**
     * High Pass Filter Register
     *
     * 0x0F HP_FILTER_CUTOFF: High Pass Filter Register (Read/Write)
     */
    union HP_FILTER_CUTOFFbits {

        struct {
            unsigned char SEL :2;
            unsigned char :2;
            unsigned char PULSE_LPF_EN :1;
            unsigned char PULSE_HPF_BYP :1;
            unsigned char :2;
        };
        unsigned char value;
    };

    /**
     * Portrait/Landscape Status Register
     *
     * 0x10 PL_STATUS Register (Read Only)
     */
    union PL_STATUSbits {

        struct {
            unsigned char BAFRO :1;
            unsigned char LAPO :2;
            unsigned char :3;
            unsigned char LO :1;
            unsigned char NEWLP :1;
        };
        unsigned char value;
    };

    /**
     * Portrait/Landscape Configuration Register
     *
     * 0x11 PL_CFG Register (Read/Write)
     */
    union PL_CFGbits {

        struct {
            unsigned char :6;
            unsigned char PL_EN :1;
            unsigned char DBCNTM :1;
        };
        unsigned char value;
    };

    /**
     * Back/Front and Z Compensation Register
     *
     * 0x13: PL_BF_ZCOMP Register (Read/Write)
     */
    union PL_BF_ZCOMPbits {

        struct {
            unsigned char ZLOCK :3;
            unsigned char :3;
            unsigned char BKFR :2;
        };
        unsigned char value;
    };

    /**
     * Portrait/Landscape Threshold and Hysteresis Register
     *
     * 0x14: P_L_THS_REG Register (Read/Write)
     */
    union P_L_THS_REGbits {

        struct {
            unsigned char HYS :3;
            unsigned char P_L_THS :5;
        };
        unsigned char value;
    };

    /**
     * Freefall/Motion Configuration Register
     *
     * 0x15 FF_MT_CFG Register (Read/Write)
     */
    union FF_MT_CFGbits {

        struct {
            unsigned char :3;
            unsigned char XEFE :1;
            unsigned char YEFE :1;
            unsigned char ZEFE :1;
            unsigned char OAE :1;
            unsigned char ELE :1;
        };
        struct {
            unsigned char :3;
            unsigned char EFE :3;
            unsigned char :2;
        };
        unsigned char value;
    };

    /**
     * Freefall/Motion Source Register
     *
     * 0x16: FF_MT_SRC Freefall and Motion Source Register (Read Only)
     */
    union FF_MT_SRCbits {

        struct {
            unsigned char XHP :1;
            unsigned char XHE :1;
            unsigned char YHP :1;
            unsigned char YHE :1;
            unsigned char ZHP :1;
            unsigned char ZHE :1;
            unsigned char :1;
            unsigned char EA :1;
        };
        unsigned char value;
    };

    /**
     * Freefall and Motion Threshold Register
     *
     * 0x17 FF_MT_THS Register (Read/Write)
     */
    union FF_MT_THSbits {

        struct {
            unsigned char THS :7;
            unsigned char DBCNTM :1;
        };
        unsigned char value;
    };

    /**
     * Transient Config Register
     *
     * 0x1D TRANSIENT_CFG Register (Read/Write)
     */
    union TRANSIENT_CFGbits {

        struct {
            unsigned char HPF_BYP :1;
            unsigned char XTEFE :1;
            unsigned char YTEFE :1;
            unsigned char ZTEFE :1;
            unsigned char ELE :1;
            unsigned char :3;
        };

        struct {
            unsigned char :1;
            unsigned char TEFE :3;
            unsigned char :4;
        };
        unsigned char value;
    };

    /**
     * Transient Source Register
     *
     * 0x1E TRANSIENT_SRC Register (Read Only)
     */
    union TRANSIENT_SRCbits {

        struct {
            unsigned char X_TRANS_POL :1;
            unsigned char XTRANSE :1;
            unsigned char Y_TRANS_POL :1;
            unsigned char YTRANSE :1;
            unsigned char Z_TRANS_POL :1;
            unsigned char ZTRANSE :1;
            unsigned char EA :1;
            unsigned char :1;
        };
        unsigned char value;
    };

    /**
     * Transient Threshold Register
     *
     * 0x1F TRANSIENT_THS Register (Read/Write)
     */
    union TRANSIENT_THSbits {

        struct {
            unsigned char THS :7;
            unsigned char DBCNTM :1;
        };
        unsigned char value;
    };

    /**
     * Pulse Configuration Register
     *
     * 0x21 PULSE_CFG Register (Read/Write)
     */
    union PULSE_CFGbits {

        struct {
            unsigned char XSPEFE :1;
            unsigned char XDPEFE :1;
            unsigned char YSPEFE :1;
            unsigned char YDPEFE :1;
            unsigned char ZSPEFE :1;
            unsigned char ZDPEFE :1;
            unsigned char ELE :1;
            unsigned char DPA :1;
        };
        struct {
            unsigned char PEFE :6;
            unsigned char :2;
        };
        unsigned char value;
    };

    /**
     * Pulse Source Register
     *
     * 0x22 PULSE_SRC Register (Read Only)
     */
    union PULSE_SRCbits {

        struct {
            unsigned char POLX :1;
            unsigned char POLY :1;
            unsigned char POLZ :1;
            unsigned char DPE :1;
            unsigned char AXX :1;
            unsigned char AXY :1;
            unsigned char AXZ :1;
            unsigned char EA :1;
        };
        unsigned char value;
    };

    /**
     * System Control 1 Register
     *
     * 0x2A CTRL_REG1 Register (Read/Write)
     */
    union CTRL_REG1bits {

        struct {
            unsigned char ACTIVE :1;
            unsigned char F_READ :1;
            unsigned char LNOISE :1;
            unsigned char DR :3;
            unsigned char ASLP_RATE :2;
        };
        unsigned char value;
    };

    /**
     * System Control 2 Register
     *
     * 0x2B CTRL_REG2 Register (Read/Write)
     */
    union CTRL_REG2bits {

        struct {
            unsigned char MODS :2;
            unsigned char SLPE :1;
            unsigned char SMODS :2;
            unsigned char :1;
            unsigned char RST :1;
            unsigned char ST :1;
        };
        unsigned char value;
    };

    /**
     * Interrupt Control Register
     *
     * 0x2C CTRL_REG3 Register (Read/Write)
     */
    union CTRL_REG3bits {

        struct {
            unsigned char PP_OD :1;
            unsigned char IPOL :1;
            unsigned char :1;
            unsigned char WAKE_FF_MT :1;
            unsigned char WAKE_PULSE :1;
            unsigned char WAKE_LNDPRT :1;
            unsigned char WAKE_TRANS :1;
            unsigned char FIFO_GATE :1;
        };
        unsigned char value;
    };

    /**
     * Interrupt Enable Register
     *
     * 0x2D CTRL_REG4 Register (Read/Write)
     */
    union CTRL_REG4bits {

        struct {
            unsigned char INT_EN_DRDY :1;
            unsigned char :1;
            unsigned char INT_EN_FF_MT :1;
            unsigned char INT_EN_PULSE :1;
            unsigned char INT_EN_LNDPR :1;
            unsigned char INT_EN_TRANS :1;
            unsigned char INT_EN_FIFO :1;
            unsigned char INT_EN_ASLP :1;
        };
        unsigned char value;
    };

    /**
     * Interrupt Configuration Register
     *
     * 0x2E: CTRL_REG5 Interrupt Configuration Register
     */
    union CTRL_REG5bits {

        struct {
            unsigned char INT_CFG_DRDY :1;
            unsigned char :1;
            unsigned char INT_CFG_FF_MT :1;
            unsigned char INT_CFG_PULSE :1;
            unsigned char INT_CFG_LNDPRT :1;
            unsigned char INT_CFG_TRANS :1;
            unsigned char INT_CFG_FIFO :1;
            unsigned char INT_CFG_ASLP :1;
        };
        unsigned char value;
    };

    /**
     * Internal registers.
     */
    enum Location {
        STATUS = 0x00,
        F_STATUS = 0x00,
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
     * Some useful masks.
     */
    enum Mask {
        STATUS_XDR = 0x01,
        STATUS_YDR = 0x02,
        STATUS_ZDR = 0x04,
        STATUS_ZYXDR = 0x08,
        STATUS_XOW = 0x10,
        STATUS_YOW = 0x20,
        STATUS_ZOW = 0x40,
        STATUS_ZYXOW = 0x80,

        F_STATUS_F_CNT = 0x3f,
        F_STATUS_F_WMRK_FLAG = 0x40,
        F_STATUS_F_OVF = 0x80,

        F_SETUP_F_WMRK = 0x3f,
        F_SETUP_F_MODE = 0xc0,

        TRIG_CFG_TRIG_FF_MT = 0x04,
        TRIG_CFG_TRIG_PULSE = 0x08,
        TRIG_CFG_TRIG_LNDPRT = 0x10,
        TRIG_CFG_TRIG_TRANS = 0x20,

        SYSMOD_SYSMOD = 0x03,
        SYSMOD_FGT = 0x7c,
        SYSMOD_FGERR = 0x80,

        INT_SOURCE_SRC_DRDY = 0x01,
        INT_SOURCE_SRC_FF_MT = 0x04,
        INT_SOURCE_SRC_PULSE = 0x08,
        INT_SOURCE_SRC_LNDPRT = 0x10,
        INT_SOURCE_SRC_TRANS = 0x20,
        INT_SOURCE_SRC_FIFO = 0x40,
        INT_SOURCE_SRC_ASLP = 0x80,

        XYZ_DATA_CFG_FS = 0x03,
        XYZ_DATA_CFG_HPF_OUT = 0x10,

        HP_FILTER_CUTOFF_SEL = 0x03,
        HP_FILTER_PULSE_LPF_EN = 0x10,
        HP_FILTER_PULSE_HPF_BYP = 0x20,

        PL_STATUS_BAFRO = 0x01,
        PL_STATUS_LAPO = 0x06,
        PL_STATUS_LO = 0x40,
        PL_STATUS_NEWLP = 0x80,

        PL_CFG_PL_EN = 0x40,
        PL_CFG_DBCNTM = 0x80,

        PL_BF_ZCOMP_ZLOCK = 0x07,
        PL_BF_ZCOMP_BKFR = 0xc0,

        P_L_THS_REG_HYS = 0x07,
        P_L_THS_REG_P_L_THS = 0xf8,

        FF_MT_CFG_XEFE = 0x08,
        FF_MT_CFG_YEFE = 0x10,
        FF_MT_CFG_ZEFE = 0x20,
        FF_MT_CFG_EFE = 0x07,
        FF_MT_CFG_OAE = 0x40,
        FF_MT_CFG_ELE = 0x80,

        FF_MT_SRC_XHP = 0x01,
        FF_MT_SRC_XHE = 0x02,
        FF_MT_SRC_YHP = 0x04,
        FF_MT_SRC_YHE = 0x08,
        FF_MT_SRC_ZHP = 0x10,
        FF_MT_SRC_ZHE = 0x20,
        FF_MT_SRC_EA = 0x80,

        FF_MT_THS_THS = 0x7f,
        FF_MT_THS_DBCNTM = 0x80,

        TRANSIENT_CFG_HPF_BYP = 0x01,
        TRANSIENT_CFG_XTEFE = 0x02,
        TRANSIENT_CFG_YTEFE = 0x04,
        TRANSIENT_CFG_ZTEFE = 0x08,
        TRANSIENT_CFG_TEFE = 0x07,
        TRANSIENT_CFG_ELE = 0x10,

        TRANSIENT_SRC_X_TRANS_POL = 0x01,
        TRANSIENT_SRC_XTRANSE = 0x02,
        TRANSIENT_SRC_Y_TRANS_POL = 0x04,
        TRANSIENT_SRC_YTRANSE = 0x08,
        TRANSIENT_SRC_Z_TRANS_POL = 0x10,
        TRANSIENT_SRC_ZTRANSE = 0x20,
        TRANSIENT_SRC_EA = 0x40,

        TRANSIENT_THS_THS = 0x7f,
        TRANSIENT_THS_DBCNTM = 0x80,

        /**
         * Pulse detection, axes single/double tap
         */
        PULSE_CFG_XSPEFE = 0x01,
        PULSE_CFG_XDPEFE = 0x02,
        PULSE_CFG_YSPEFE = 0x04,
        PULSE_CFG_YDPEFE = 0x08,
        PULSE_CFG_ZSPEFE = 0x10,
        PULSE_CFG_ZDPEFE = 0x20,
        PULSE_CFG_ELE = 0x40,
        PULSE_CFG_DPA = 0x80,
        PULSE_CFG_PEFE = 0x3f,

        PULSE_SRC_POLX = 0x01,
        PULSE_SRC_POLY = 0x02,
        PULSE_SRC_POLZ = 0x04,
        PULSE_SRC_DPE = 0x08,
        PULSE_SRC_AXX = 0x10,
        PULSE_SRC_AXY = 0x20,
        PULSE_SRC_AXZ = 0x40,
        PULSE_SRC_EA = 0x80,

        PULSE_THS_THS = 0x7f,

        CTRL_REG1_ACTIVE = 0x01,
        CTRL_REG1_F_READ = 0x02,
        CTRL_REG1_LNOISE = 0x04,
        CTRL_REG1_DR = 0x38,
        CTRL_REG1_ASLP_RATE = 0xc0,

        CTRL_REG2_MODS = 0x03,
        CTRL_REG2_SLPE = 0x04,
        CTRL_REG2_SMODS = 0x18,
        CTRL_REG2_RST = 0x40,
        CTRL_REG2_ST = 0x80,

        CTRL_REG3_PP_OD = 0x01,
        CTRL_REG3_IPOL = 0x02,
        CTRL_REG3_WAKE_FF_MT = 0x08,
        CTRL_REG3_WAKE_PULSE = 0x10,
        CTRL_REG3_WAKE_LNDPRT = 0x20,
        CTRL_REG3_WAKE_TRANS = 0x40,
        CTRL_REG3_FIFO_GATE = 0x80,

        CTRL_REG4_INT_EN_DRDY = 0x01,
        CTRL_REG4_INT_EN_FF_MT = 0x04,
        CTRL_REG4_INT_EN_PULSE = 0x08,
        CTRL_REG4_INT_EN_LNDPR = 0x10,
        CTRL_REG4_INT_EN_TRANS = 0x20,
        CTRL_REG4_INT_EN_FIFO = 0x40,
        CTRL_REG4_INT_EN_ASLP = 0x80,

        CTRL_REG5_INT_CFG_DRDY = 0x01,
        CTRL_REG5_INT_CFG_FF_MT = 0x04,
        CTRL_REG5_INT_CFG_PULSE = 0x08,
        CTRL_REG5_INT_CFG_LNDPRT = 0x10,
        CTRL_REG5_INT_CFG_TRANS = 0x20,
        CTRL_REG5_INT_CFG_FIFO = 0x40,
        CTRL_REG5_INT_CFG_ASLP = 0x80
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
     * Table 3.Full Scale Selection
     *
     * <pre>
     * FS1 FS0  g Range
     * 0    0   +/-2g
     * 0    1   +/-4g
     * 1    0   +/-8g
     * 1    1   -
     * </pre>
     */
    enum DynamicRange {
        DR_2G = 0x00,
        DR_4G = 0x01,
        DR_8G = 0x02
    };

    /**
     * Table 5. Output Data Rates
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
     * It is important to note that when the device is Auto-SLEEP mode,
     * the system ODR and the data rate for all the system functional
     * blocks are overridden by the data rate set by the ASLP_RATE
     * field.
     *
     * <pre>
     * ASLP_RATE1   ASLP_RATE0  Frequency (Hz)
     * 0            0           50
     * 0            1           12.5
     * 1            0           6.25
     * 1            1           1.56
     */
    enum AslpOutputDataRate {
        ASLP_50HZ = 0x00,
        ASLP_12_5HZ = 0x01,
        ASLP_6_25HZ = 0x02,
        ASLP_1_56HZ = 0x03
    };

    /**
     * Oversampling Mode.
     *
     * <pre>
     * MODS1    MODS0
     * 0        0       Normal
     * 0        1       Low noise, low power
     * 1        0       High resolution
     * 1        1       Low power
     * </pre>
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
        HP_FILTER_CUTOFF_0 = 0x00,
        HP_FILTER_CUTOFF_1 = 0x01,
        HP_FILTER_CUTOFF_2 = 0x02,
        HP_FILTER_CUTOFF_3 = 0x03
    };

    /**
     * FIFO buffer overflow mode
     *
     * FIFO buffer overflow mode. Default value: 0.
     * 00: FIFO is disabled.
     * 01: FIFO contains the most recent samples when overflowed
     *      (circular buffer). Oldest sample is discarded to be replaced
     *      by new sample.
     * 10: FIFO stops accepting new samples when overflowed.
     * 11: Trigger mode. The FIFO will be in a circular mode up to the
     *      number of samples in the watermark. The FIFO will be in a
     *      circular mode until the trigger event occurs after that the
     *      FIFO will continue to accept samples for 32-WMRK samples and
     *      then stop receiving further samples. This allows data to be
     *      collected both before and after the trigger event and it is
     *      definable by the watermark setting. The FIFO is flushed
     *      whenever the FIFO is disabled, during an automatic ODR
     *      change (Auto-WAKE/SLEEP), or transitioning from STANDBY mode
     *      to ACTIVE mode. Disabling the FIFO (F_MODE = 00) resets the
     *      F_OVF, F_WMRK_FLAG, F_CNT to zero. A FIFO overflow event
     *      (i.e., F_CNT = 32) will assert the F_OVF flag and a FIFO
     *      sample count equal to the sample count watermark (i.e.,
     *      F_WMRK) asserts the F_WMRK_FLAG event flag.
     */
    enum FifoBufferOverflowMode {
        FIFO_DISABLED = 0x00,
        FIFO_CIRCULAR_BUFFER = 0x01,
        FIFO_STOP_WHEN_OVERFLOWED = 0x02,
        FIFO_TRIGGER = 0x03
    };

    /**
     * Fast Read mode: Data format limited to single Byte Default
     * value: 0.
     * (0: Normal mode 1: Fast Read Mode)
     */
    enum ReadMode {
        FAST_READ = 0x01,
        NORMAL_READ = 0x00
    };

    /**
     * Trigger bits.
     *
     * <pre>
     * INT_SOURCE   Description
     *
     * Trig_TRANS   Transient interrupt trigger bit. Default value: 0
     * Trig_LNDPRT  Landscape/Portrait Orientation interrupt trigger
     *              bit. Default value: 0
     * Trig_PULSE   Pulse interrupt trigger bit. Default value: 0
     * Trig_FF_MT   Freefall/Motion trigger bit. Default value: 0
     * <pre>
     */
    enum InterruptTriggerBits {
        TRIG_TRANS = 0x20,
        TTRIG_LNDPRT = 0x10,
        TRIG_PULSE = 0x08,
        TTRIG_FF_MT = 0x04
    };

    /**
     * Back/Front Trip Angle Threshold. Default: 01 >= +/-75º.
     * Step size is 5º.
     * Range: +/-(65º to 80º).
     */
    enum BackFrontTrip {
        BKFR_80 = 0x00,
        BKFR_75 = 0x01,
        BKFR_70 = 0x02,
        BKFR_65 = 0x03
    };

    /**
     * Z-Lock Angle Threshold. Range is from 14º to 43º.
     * Step size is 4º.
     * Default value: 100 >= 29º.
     * Maximum value: 111 >= 43º.
     */
    enum ZLockThresholdAngle {
        ZLOCK_14 = 0x00,
        ZLOCK_18 = 0x01,
        ZLOCK_21 = 0x02,
        ZLOCK_25 = 0x03,
        ZLOCK_29 = 0x04,
        ZLOCK_33 = 0x05,
        ZLOCK_37 = 0x06,
        ZLOCK_42 = 0x07
    };

    enum PortraitLandscapeThresholdAngle {
        P_L_THS_15 = 0x07,
        P_L_THS_20 = 0x09,
        P_L_THS_30 = 0x0c,
        P_L_THS_35 = 0x0d,
        P_L_THS_40 = 0x0f,
        P_L_THS_45 = 0x10,
        P_L_THS_55 = 0x13,
        P_L_THS_60 = 0x14,
        P_L_THS_70 = 0x17,
        P_L_THS_75 = 0x19
    };

    /**
     * Trip Angles with Hysteresis for 45º Angle
     *
     * <pre>
     * Hysteresis       Hysteresis      Landscape to Portrait   Portrait to Landscape
     * Register Value   +/- Angle Range   Trip Angle              Trip Angle
     * 0                +/-0              45º                     45º
     * 1                +/-4              49º                     41º
     * 2                +/-7              52º                     38º
     * 3                +/-11             56º                     34º
     * 4                +/-14             59º                     31º
     * 5                +/-17             62º                     28º
     * 6                +/-21             66º                     24º
     * 7                +/-24             69º                     21º
     * </pre>
     */
    enum HysteresisAngle {
        HYS_0 = 0x00,
        HYS_4 = 0x01,
        HYS_7 = 0x02,
        HYS_11 = 0x03,
        HYS_14 = 0x04,
        HYS_17 = 0x05,
        HYS_21 = 0x06,
        HYS_24 = 0x07
    };

    /**
     * INT_EN_ASLP
     * 0: Auto-SLEEP/WAKE interrupt disabled;
     * 1: Auto-SLEEP/WAKE interrupt enabled.
     *
     * INT_EN_FIFO
     * 0: FIFO interrupt disabled;
     * 1: FIFO interrupt enabled.
     *
     * INT_EN_TRANS
     * 0: Transient interrupt disabled;
     * 1: Transient interrupt enabled.
     *
     * INT_EN_LNDPRT
     * 0: Orientation (Landscape/Portrait) interrupt disabled.
     * 1: Orientation (Landscape/Portrait) interrupt enabled.
     *
     * INT_EN_PULSE
     * 0: Pulse Detection interrupt disabled;
     * 1: Pulse Detection interrupt enabled
     *
     * INT_EN_FF_MT
     * 0: Freefall/Motion interrupt disabled;
     * 1: Freefall/Motion interrupt enabled
     *
     * INT_EN_DRDY
     * 0: Data Ready interrupt disabled;
     * 1: Data Ready interrupt enabled
     *
     * INT_CFG_ASLP
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     *
     * INT_CFG_FIFO
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     *
     * INT_CFG_TRANS
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     *
     * INT_CFG_LNDPRT
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     *
     * INT_CFG_PULSE
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     *
     * INT_CFG_FF_MT
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     *
     * INT_CFG_DRDY
     * 0: Interrupt is routed to INT2 pin;
     * 1: Interrupt is routed to INT1 pin
     */
    enum Interrupt {
        INT_DRDY = 0x01,
        INT_FF_MT = 0x04,
        INT_PULSE = 0x08,
        INT_LNDPRT = 0x10,
        INT_TRANS = 0x20,
        INT_FIFO = 0x40,
        INT_ASLP = 0x80,
        INT_ALL = 0xff
    };

    /**
     * Interrupt polarity
     */
    enum InterruptPolarity {
        ACTIVE_LOW = 0x00,
        ACTIVE_HIGH = 0x01
    };

    /**
     * Push-Pull/Open Drain selection on interrupt pad.
     * Default value: 0.
     * 0: Push-Pull; 1: Open Drain
     */
    enum PushPullOpenDrain {
        PUSH_PULL = 0x00,
        OPEN_DRAIN = 0x01
    };

    /**
     * Errors
     *
     * 1: data too long to fit in transmit buffer
     * 2: received NACK on transmit of address
     * 3: received NACK on transmit of data
     * 4: other error
     * 5: Timeout
     */
    enum CommunicationError {
        NO_ERROR = 0x00,
        DATA_TOO_LONG = 0x01,
        NACK_ON_ADDRESS = 0x02,
        NACK_ON_DATA = 0x03,
        OTHER = 0x04,
        TIMEOUT_ON_READING = 0x05
    };

    /**
     * Axis
     */
    enum DetectionAxis {
        AXIS_NONE = 0x00,
        AXIS_X = 0x01,
        AXIS_Y = 0x02,
        AXIS_Z = 0x04,
        AXIS_XY = AXIS_X | AXIS_Y,
        AXIS_XZ = AXIS_X | AXIS_Z,
        AXIS_YZ = AXIS_Y | AXIS_Z,
        AXIS_XYZ = AXIS_X | AXIS_Y | AXIS_Z
    };

    /**
     * Motion detect or Freefall detect
     */
    enum DetectionFunction {
        FREEFALL_DETECT = 0x00,
        MOTION_DETECT = 0x01
    };

    /**
     * Public constructor.
     *
     * @param sa0       The LSBit of the address.
     */
    AccelerometerMMA8451(bool sa0);

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
    bool isDataReady();

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
    void standby() {
        deviceActivation(DEACTIVATE);
    }

    /**
     * Put sensor into Active Mode
     *
     * Read current value of System Control 1 Register.
     * Put sensor into Active Mode by setting the Active bit
     * Return with previous value of System Control 1 Register.
     */
    void activate() {
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
     * <pre>
     * FS1  FS0 g Range
     * 0    0   +/-2g
     * 0    1   +/-4g
     * 1    0   +/-8g
     * 1    1   -
     * </pre>
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
     * @param rate              The Output Data Rate.
     */
    void setOutputDataRate(OutputDataRate rate);

    /**
     * Portrait/Landscape Detection Enable
     *
     * Default value: 0
     * 0: Portrait/Landscape Detection is Disabled.
     * 1: Portrait/Landscape Detection is Enabled.
     *
     * @param enable            The enable flag.
     */
    void setPortraitLandscapeDetection(bool enable);

    /**
     * Transient Detection Enable
     *
     * Transient event flags are latched into the TRANSIENT_SRC register.
     * Reading of the TRANSIENT_SRC register clears the event flag.
     *
     * Default value: 0.
     * 0: Event flag latch disabled;
     * 1: Event flag latch enabled
     *
     * ELE:
     * Transient event flags are latched into the TRANSIENT_SRC register.
     * Reading of the TRANSIENT_SRC register clears the event flag.
     *
     * Default value: 0.
     * 0: Event flag latch disabled;
     * 1: Event flag latch enabled
     *
     * ZTEFE:
     *
     * Event flag enable on Z transient acceleration greater than transient threshold event.
     *
     * Default value: 0.
     * 0: Event detection disabled;
     * 1: Raise event flag on measured acceleration delta value greater than transient threshold.
     *
     * YTEFE:
     * Event flag enable on Y transient acceleration greater than transient threshold event.
     *
     * Default value: 0.
     * 0: Event detection disabled;
     * 1: Raise event flag on measured acceleration delta value greater than transient threshold.
     *
     * XTEFE:
     * Event flag enable on X transient acceleration greater than transient threshold event.
     *
     * Default value: 0.
     * 0: Event detection disabled;
     * 1: Raise event flag on measured acceleration delta value greater than transient threshold.
     *
     * HPF_BYP:
     * Bypass High-Pass filter.
     *
     * Default value: 0.
     * 0: Data to transient acceleration detection block is through HPF
     * 1: Data to transient acceleration detection block is NOT through HPF (similar to motion detection function)
     *
     * @param enable            The enable flag.
     * <pre>
     * @param axis              0x00000111
     *                                 |||_ enable on X
     *                                 ||__ enable on Y
     *                                 |___ enable on Z
     * </pre>
     * @param bypass            Bypass High-Pass filter.
     */
    void setTransientDetection(bool enable, unsigned char axis, bool bypass);

    /**
     * Transient Threshold.
     *
     * The Transient Threshold register sets the threshold limit for the detection of the transient acceleration.
     * The value in the TRANSIENT_THS register corresponds to a g value which is compared against the values of High-Pass Filtered Data.
     * If the HighPass Filtered acceleration value exceeds the threshold limit, an event flag is raised and the interrupt is generated if enabled.
     *
     * @param debounceCounterMode       Debounce counter mode selection. Default value: 0.
     *                                  0: increments or decrements debounce;
     *                                  1: increments or clears counter.
     * @param threshold                 Transient Threshold: Default value (0x7f mask will be applied)
     */
    void setTransientThreshold(bool debounceCounterMode, unsigned char threshold);

    /**
     * The TRANSIENT_COUNT sets the minimum number of debounce counts continuously matching the condition where the
     * unsigned value of high-pass filtered data is greater than the user specified value of TRANSIENT_THS.
     *
     * The time step for the transient detection debounce counter is set by the value of the system ODR and the Oversampling mode.
     *
     * @param count                     The number of debounce sample counts.
     */
    void setTransientCount(unsigned char count);

    /**
     * configure the event flag for the tap detection interrupt function for enabling/disabling single and double pulse
     *  on each of the axes
     *
     * @param enable Whether the status bits will be latched or not in the source register
     * @param axis See PULSE_CFG_ flags above.
     * @param dpa Double Pulse Abort, see Application Notes 4072
     */
    void setPulseDetection(bool enable, unsigned char axis, bool dpa);

    /**
     * The pulse threshold can be set separately for the X, Y, and Z axes. This allows for the same change in acceleration
     * for all axes to be set regardless of the static orientation. The threshold values range from 0 to 127 (7 bits
     * expressed as an absolute value) with steps of 0.063g/LSB at a fixed 8g acceleration range.
     */
    void setPulseThreshold(unsigned char thresholdX, unsigned char thresholdY, unsigned char thresholdZ);

    /**
     * The byte PULSE_TMLT (bit fields defined as Tmlt7 through Tmlt0) define the maximum time interval that can elapse
     * between the start of the acceleration on the selected axis exceeding the specified threshold and the end when
     * the axes of acceleration must go back below the specified threshold.
     */
    void setPulseFirstTimer(unsigned char timer);

    /**
     * The Pulse Latency Timer Register is the time duration that the tap event can be read from the source register
     * to detect the X, Y, and Z for single pulse or double pulse events without the latch enabled. The duration
     * of any specified latency time is valid each time a single or double pulse occurs
     */
    void setPulseLatency(unsigned char latency);

    /**
     * The PULSE_WIND register with (bit fields Wind7 through Wind0) defines the maximum interval of time that can
     * elapse after the end of the latency interval within which the start of the second pulse event must be detected
     * (provided the device has been configured with double pulse detection enabled). The detected second pulse width
     * must be shorter than the time limit constraints specified by the PULSE_TMLT register, but the double pulse need
     * not cross the threshold within the time specified by the PULSE_WIND register. The timing is the same as that of
     * the latency timer. The maximum time is the time step at the ODR and oversampling mode multiplied by 255.
     */
    void setPulseSecondWindow(unsigned char window);

    /**
     * The MMA8451Q Orientation Detection algorithm confirms the reliability of the function with a configurable Z-lockout angle.
     * Based on known functionality of linear accelerometers, it is not possible to rotate the device about the Z-axis to detect change in
     * acceleration at slow angular speeds. The angle at which the device no longer detects the orientation change is referred to as the
     * "Z-Lockout angle". The device operates down to 14° from the flat position.
     *
     * 0x11: Portrait/Landscape Configuration Register
     *
     * This register enables the Portrait/Landscape function and sets the behavior of the debounce counter.
     *
     * DBCNTM:
     * Debounce counter mode selection. Default value: 1
     *  0: Decrements debounce whenever condition of interest is no longer valid.
     *  1: Clears counter whenever condition of interest is no longer valid.
     *
     * PL_EN:
     * Portrait/Landscape Detection Enable. Default value: 0
     *  0: Portrait/Landscape Detection is Disabled.
     *  1: Portrait/Landscape Detection is Enabled.
     *
     * @param enable                Portrait/Landscape Detection Enable. Default value: 0
     *                                  0: Portrait/Landscape Detection is Disabled.
     *                                  1: Portrait/Landscape Detection is Enabled.
     * @param debounceCounterMode   Debounce counter mode selection. Default value: 1
     *                                  0: Decrements debounce whenever condition of interest is no longer valid.
     *                                  1: Clears counter whenever condition of interest is no longer valid.
     */
    void setOrientationDetection(bool enable, bool debounceCounterMode);

    /**
     * Portrait/Landscape Debounce Counter
     *
     * This register sets the debounce count for the orientation state transition. The minimum debounce latency is determined by the
     * data rate set by the product of the selected system ODR and PL_COUNT registers. Any transition from WAKE to SLEEP or vice
     * versa resets the internal Landscape/Portrait debounce counter. Note: The debounce counter weighting (time step) changes
     * based on the ODR and the Oversampling mode. Table 27 explains the time step value for all sample rates and all Oversampling
     * modes.
     *
     * @param counter Portrait/Landscape Debounce Counter value.
     */
    void setOrientationDebounceCounter(unsigned char debounceCounter);

    /**
     * PL_BF_ZCOMP Back/Front and Z Compensation Register
     *
     * The Z-Lock angle compensation bits allow the user to adjust the Z-lockout region from 14° up to 43°. The default Z-lockout angle
     * is set to the default value of 29° upon power up. The Back to Front trip angle is set by default to ±75° but this angle also can be
     * adjusted from a range of 65° to 80° with 5° step increments.
     *
     * @param tripAngleThreshold        Back/Front Trip Angle Threshold. Default: 01 ≥ ±75°.
     *                                  Step size is 5°. Range: ±(65° to 80°).
     * @param zLockAngle                Z-Lock Angle Threshold. Range is from 14° to 43°.
     *                                  Step size is 4°. Default value: 100 ≥ 29°. Maximum value: 111 ≥ 43°.
     */
    void setOrientationBackFrontCompensation(unsigned char tripAngleThreshold, unsigned char zLockAngle);

    /**
     * PL_THS_REG Portrait/Landscape Threshold and Hysteresis Register
     *
     * PL_THS[4] PL_THS[3] PL_THS[2] PL_THS[1] PL_THS[0] HYS[2] HYS[1] HYS[0]
     *
     * This register represents the Portrait to Landscape trip threshold register used to set the trip angle for transitioning from Portrait
     * to Landscape and Landscape to Portrait. This register includes a value for the hysteresis.
     *
     * @param threshold             Portrait/Landscape trip threshold angle from 15° to 75°.
     *                              See Table 32 for the values with the corresponding approximate threshold angle.
     *                              Default value: 1_0000 (45°).
     * @param hysteresis            This angle is added to the threshold angle for a smoother transition from Portrait
     *                              to Landscape and Landscape to Portrait. This angle ranges from 0° to ±24°. The default is 100 (±14°).
     */
    void setOrientationThresholdAndHysteresis(unsigned char threshold, unsigned char hysteresis);

    /**
     * This is the Freefall/Motion configuration register for setting up the conditions of the freefall or motion function.
     *
     * ELE:
     * Event Latch Enable: Event flags are latched into FF_MT_SRC register. Reading of the FF_MT_SRC register clears the event
     * flag EA and all FF_MT_SRC bits. Default value: 0.
     * 0: Event flag latch disabled;
     * 1: event flag latch enabled
     *
     * OAE:
     * Motion detect / Freefall detect flag selection. Default value: 0. (Freefall Flag)
     * 0: Freefall Flag (Logical AND combination)
     * 1: Motion Flag (Logical OR combination)
     *
     * ZEFE:
     * Event flag enable on Z Default value: 0.
     * 0: event detection disabled;
     * 1: raise event flag on measured acceleration value beyond preset threshold
     *
     * YEFE:
     * Event flag enable on Y event. Default value: 0.
     * 0: Event detection disabled;
     * 1: raise event flag on measured acceleration value beyond preset threshold
     *
     * XEFE:
     * Event flag enable on X event. Default value: 0.
     * 0: event detection disabled;
     * 1: raise event flag on measured acceleration value beyond preset threshold
     *
     * OAE bit allows the selection between Motion (logical OR combination) and Freefall (logical AND combination) detection.
     * ELE denotes whether the enabled event flag will to be latched in the FF_MT_SRC register or the event flag status
     * in the FF_MT_SRC will indicate the real-time status of the event. If ELE bit is set to a logic ‘1’, then the event
     * flags are frozen when the EA bit gets set, and are cleared by reading the FF_MT_SRC source register. ZHFE, YEFE, XEFE enable
     * the detection of a motion or freefall event when the measured acceleration data on X, Y, Z channel is beyond the threshold
     * set in FF_MT_THS register. If the ELE bit is set to logic ‘1’ in the FF_MT_CFG register new event flags are blocked from
     * updating the FF_MT_SRC register. FF_MT_THS is the threshold register used to detect freefall motion events.
     * The unsigned 7-bit FF_MT_THS threshold register holds the threshold for the freefall detection where the magnitude of the
     * X and Y and Z acceleration values is lower or equal than the threshold value. Conversely, the FF_MT_THS also holds the
     * threshold for the motion detection where the magnitude of the X or Y or Z acceleration value is higher than the threshold value.
     *
     * @param ele                       Event Latch Enable
     * @param oae                       Motion detect / Freefall detect flag selection
     * <pre>
     * @param axis              0x00000111
     *                                 |||_ enable on X
     *                                 ||__ enable on Y
     *                                 |___ enable on Z
     * </pre>
     */
    void setMotionDetection(bool ele, bool oae, unsigned char axis);

    /**
     * The threshold resolution is 0.063g/LSB and the threshold register has a range of 0 to 127 counts.
     * The maximum range is to 8g. Note that even when the full scale value is set to 2g or 4g the motion
     * detects up to 8g. If the Low Noise bit is set in Register 0x2A then the maximum threshold will be
     * limited to 4g regardless of the full scale range.
     *
     * When DBCNTM bit is a logic ‘1’, the debounce counter is cleared to 0 whenever the inertial event of
     * interest is no longer true as shown in Figure 13, (b). While the DBCNTM bit is set to logic ‘0’ the
     * debounce counter is decremented by 1 whenever the inertial event of interest is no longer true
     * (Figure 13, (c)) until the debounce counter reaches 0 or the inertial event of interest becomes active.
     *
     * @param debounceCounterMode       Debounce counter mode selection. Default value: 0.
     *                                  0: increments or decrements debounce;
     *                                  1: increments or clears counter.
     * @param threshold                 FF MD Threshold: Default value (0x7f mask will be applied)
     */
    void setMotionDetectionThreshold(bool debounceCounterMode, unsigned char threshold);

    /**
     * This register sets the number of debounce sample counts for the event trigger.
     *
     * This register sets the minimum number of debounce sample counts of continuously matching the detection
     * condition user selected for the freefall, motion event. When the internal debounce counter reaches
     * the FF_MT_COUNT value a Freefall/Motion event flag is set. The debounce counter will never increase beyond
     * the FF_MT_COUNT value.
     *
     * NOTE: Time step used for the debounce sample count depends on the ODR chosen and the Oversampling mode as
     * shown in Table 38.
     *
     * @param count                     The number of debounce sample counts.
     */
    void setMotionDetectionCount(unsigned char count);

    /**
     * Sets the Back/Front Trip Angle Threshold.
     *
     * Default: 01 >= +/-75º.
     * Step size is 5º.
     * Range: +/-(65º to 80º).
     *
     * @param trip              The trip.
     */
    void setBackFrontTrip(BackFrontTrip trip);

    /**
     * Sets the Z-Lock Angle Threshold.
     *
     * Range is from 14º to 43º.
     * Step size is 4º.
     * Default value: 100 >= 29º.
     * Maximum value: 111 >= 43º.
     *
     * @param angle             The angle.
     */
    void setZLockThresholdAngle(ZLockThresholdAngle angle);

    /**
     * Sets Portrait/Landscape trip threshold angle from 15º to 75º.
     *
     * See Table 31 for the values with the corresponding approximate
     * threshold angle. Default value: 1_0000 (45º).
     *
     * @param angle             The angle.
     */
    void setPortraitLandscapeThresholdAngle(PortraitLandscapeThresholdAngle angle);

    /**
     * Sets the Hysteresis Angle.
     *
     * This angle is added to the threshold angle for a smoother
     * transition from Portrait to Landscape and Landscape to Portrait.
     * This angle ranges from 0º to +/-24º. The default is 100 (+/-14º).
     *
     * @param angle             The angle.
     */
    void setHysteresisAngle(HysteresisAngle angle);

    /**
     * Enables some interrupt.
     *
     * NOTE: The interrupt will be routed to the default pin.
     *
     * @param interrupt         The interrupt flag.
     */
    void enableInterrupt(Interrupt interrupt);

    /**
     * Enables some interrupt, and route to the given pin.
     *
     * @param interrupt         The interrupt flag.
     * @param routePin          The pin where the interrupt will be routed.
     *                          should be 1 or 2
     */
    void enableInterrupt(Interrupt interrupt, unsigned char routePin);

    /**
     * Disable some interrupt.
     *
     * @param interrupt         The interrupt flag.
     */
    void disableInterrupt(Interrupt interrupt);

    /**
     * Interrupt is routed to INT1 pin;
     *
     * @deprecated
     * @param interrupt         The interrupt flag.
     */
    void routeInterruptToInt1(Interrupt interrupt);

    /**
     * Interrupt is routed to INT2 pin;
     *
     * @deprecated
     * @param interrupt         The interrupt flag.
     */
    void routeInterruptToInt2(Interrupt interrupt);

    /**
     * Interrupt is routed to the given pin;
     *
     * @param interrupt         The interrupt flag.
     * @param routePin          The pin where the interrupt will be routed.
     *                          should be 1 or 2
     */
    void routeInterrupt(Interrupt interrupt, unsigned char routePin);

    /**
     * Sets the interrupt polarity
     *
     * @param polarity          The polarity of the interrupt.
     */
    void setInterruptPolarity(InterruptPolarity polarity);

    /**
     * Sets the sllep output data rate.
     *
     * It is important to note that when the device is Auto-SLEEP mode,
     * the system ODR and the data rate for all the system functional
     * blocks are overridden by the data rate set by the ASLP_RATE
     * field.
     *
     * @param rate              The Output Data Rate.
     */
    void setAslpOutputDataRate(AslpOutputDataRate rate);

    /**
     * Set the read mode
     *
     * F_READ bit selects between normal and Fast Read mode. When
     * selected, the auto increment counter will skip over the LSB data
     * bytes. Data read from the FIFO will skip over the LSB data,
     * reducing the acquisition time. Note F_READ can only be changed
     * when FMODE = 00. The F_READ bit applies for both the output
     * registers and the FIFO.
     *
     * @param mode              The read mode.
     */
    void setReadMode(ReadMode mode);

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
    void highPassFilteredData(bool filtered);

    /**
     * Sets the fifo overflow mode.
     *
     * NOTE: The FIFO mode can be changed while in the active state.
     * The mode must first be disabled F_MODE = 00 then the mode can be
     * switched between Fill mode, Circular mode and Trigger mode.
     *
     * @param mode              The overflow mode.
     */
    void setFifoBufferOverflowMode(FifoBufferOverflowMode mode);

    /**
     * Sets the fifo watermark.
     *
     * FIFO Event Sample Count Watermark. Default value: 00_0000.
     * These bits set the number of FIFO samples required to trigger a
     * watermark interrupt. A FIFO watermark event flag is raised when
     * FIFO sample count F_CNT[5:0] >= F_WMRK[5:0] watermark.
     *
     * Setting the F_WMRK[5:0] to 00_0000 will disable the FIFO
     * watermark event flag generation. Also used to set the number of
     * pre-trigger samples in Trigger mode.
     *
     * @param watermark         The fifo watermark count.
     */
    void setFifoWatermark(unsigned char watermark);

    /**
     * Gets the FIFO Gate Error
     */
    bool getFifoGateError();

    /**
     * Gets the FIFO Fgt
     */
    unsigned char getFifoFgt();

    /**
     * Gets the FIFO Sysmode
     */
    unsigned char getSysmod();

    /**
     * Sets the selection on interrupt pad.
     *
     * @param ppod            PushPullOpenDrain
     */
    void setPushPullOpenDrain(PushPullOpenDrain ppod);

    /**
     * Converts an array of chars into a float type. Using current FAST_READ info.
     *
     * @param buf               1 (8-bit) or 2 (14-bit) array of bytes to be converted.
     *                          value.
     */
    float convertToG(unsigned char* buf);

    /**
     * Converts an array of chars into a float type.
     *
     * @param buf               1 (8-bit) or 2 (14-bit) array of bytes to be converted.
     * @param fastRead          boolean indication if the buffer has a fast read
     *                          value.
     */
    float convertToG(unsigned char* buf, bool fastRead);

    /**
     * Gets the last error happened.
     *
     * @return                  The last error, 0 means no error.
     */
    CommunicationError gerLastError();

protected:

    /**
     * Last error
     */
    CommunicationError lastError;

    /**
     * Holds the current Dynamic Range Settings
     *
     * It is important to hold this on the object to avoid
     * unnecessary read operations on the device.
     */
    XYZ_DATA_CFGbits xyzDataCfg;

    /**
     * Holds the system Control 1 Register
     *
     * It is important to hold this on the object to avoid
     * unnecessary read operations on the device.
     */
    CTRL_REG1bits ctrlReg1;

    /**
     * Generic method to convert an axis to g.
     *
     * @param axisRegister              One of 3 possible register which contains the axis data (OUT_X_MSB, OUT_Y_MSB or OUT_Z_MSB)
     */
    float readAxisGravity(unsigned char axisRegister);
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_H__ */
