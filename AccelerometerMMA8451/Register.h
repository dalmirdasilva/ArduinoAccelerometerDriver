/**
 * Arduino - Accelerometer driver
 * 
 * Register.h
 * 
 * The implementation of the MMA8451 accelerometer.
 * 
 * This file provides a useful list of registers and locations.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_REGISTER_H__
#define __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_REGISTER_H__ 1

/**
 * Main register has its own structs.
 */
class Register {

public:
    
    /**
     * 0x00 STATUS: Data Status Register (Read Only)
     * When F_MODE == 0
     */
    union STATUSbits {
        struct {
            unsigned char XDR : 1;
            unsigned char YDR : 1;
            unsigned char ZDR : 1;
            unsigned char ZYXDR : 1;
            unsigned char XOW : 1;
            unsigned char YOW : 1;
            unsigned char ZOW : 1;
            unsigned char ZYXOW : 1;
        };
    };
    
    /**
     * 0x00 F_STATUS: FIFO Status Register
     * When F_MODE > 0
     */
    union F_STATUSbits {
        struct {
            unsigned char F_CNT : 6;
            unsigned char F_WMRK_FLAG : 1;
            unsigned char F_OVF : 1;
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
            unsigned char F_WMRK : 6;
            unsigned char F_MODE : 2;
        };
        unsigned char value;
    };
     
    /**
     * 0x0A: TRIG_CFG Trigger Configuration Register (Read/Write)
     */
    union TRIG_CFGbits {
        struct {
            unsigned char : 2;
            unsigned char TRIG_FF_MT : 1;
            unsigned char TRIG_PULSE : 1;
            unsigned char TRIG_LNDPRT : 1;
            unsigned char TRIG_TRANS : 1;
            unsigned char : 2;
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
            unsigned char SYSMOD : 2;
            unsigned char FGT : 5;
            unsigned char FGERR : 1;
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
            unsigned char SRC_DRDY : 1;
            unsigned char : 1;
            unsigned char SRC_FF_MT : 1;
            unsigned char SRC_PULSE : 1;
            unsigned char SRC_LNDPRT : 1;
            unsigned char SRC_TRANS : 1;
            unsigned char SRC_FIFO : 1;
            unsigned char SRC_ASLP : 1;
        };
        unsigned char value;
    };
     
    /**
     * 0x0E: XYZ_DATA_CFG (Read/Write)
     */
    union XYZ_DATA_CFGbits {
        struct {
            unsigned char FS : 2;
            unsigned char : 2;
            unsigned char HPF_OUT : 1;
            unsigned char : 3;
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
            unsigned char SEL : 2;
            unsigned char : 2;
            unsigned char PULSE_LPF_EN : 1;
            unsigned char PULSE_HPF_BYP : 1;
            unsigned char : 2;
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
            unsigned char BAFRO : 1;
            unsigned char LAPO : 2;
            unsigned char : 3;
            unsigned char LO : 1;
            unsigned char NEWLP : 1;
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
            unsigned char : 6;
            unsigned char PL_EN : 1;
            unsigned char DBCNTM : 1;
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
            unsigned char ZLOCK : 3;
            unsigned char : 3;
            unsigned char BKFR : 2;
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
            unsigned char HYS : 3;
            unsigned char P_L_THS : 5;
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
            unsigned char : 3;
            unsigned char XEFE : 1;
            unsigned char YEFE : 1;
            unsigned char ZEFE : 1;
            unsigned char OAE : 1;
            unsigned char ELE : 1;
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
            unsigned char XHP : 1;
            unsigned char XHE : 1;
            unsigned char YHP : 1;
            unsigned char YHE : 1;
            unsigned char ZHP : 1;
            unsigned char ZHE : 1;
            unsigned char : 1;
            unsigned char EA : 1;
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
            unsigned char THS: 7;
            unsigned char DBCNTM : 1;
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
            unsigned char HPF_BYP : 1;
            unsigned char XTEFE : 1;
            unsigned char YTEFE : 1;
            unsigned char ZTEFE : 1;
            unsigned char ELE : 1;
            unsigned char : 3;
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
            unsigned char X_TRANS_POL : 1;
            unsigned char XTRANSE : 1;
            unsigned char Y_TRANS_POL : 1;
            unsigned char YTRANSE : 1;
            unsigned char Z_TRANS_POL : 1;
            unsigned char ZTRANSE : 1;
            unsigned char EA : 1;
            unsigned char : 1;
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
            unsigned char THS  : 7;
            unsigned char DBCNTM : 1;
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
            unsigned char XSPEFE : 1;
            unsigned char XDPEFE : 1;
            unsigned char YSPEFE : 1;
            unsigned char YDPEFE : 1;
            unsigned char ZSPEFE : 1;
            unsigned char ZDPEFE : 1;
            unsigned char ELE : 1;
            unsigned char DPA : 1;
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
            unsigned char POLX : 1;
            unsigned char POLY : 1;
            unsigned char POLZ : 1;
            unsigned char DPE : 1;
            unsigned char AXX : 1;
            unsigned char AXY : 1;
            unsigned char AXZ : 1;
            unsigned char EA : 1;
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
            unsigned char ACTIVE : 1;
            unsigned char F_READ : 1;
            unsigned char LNOISE : 1;
            unsigned char DR : 3;
            unsigned char ASLP_RATE : 2;
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
            unsigned char MODS : 2;
            unsigned char SLPE : 1;
            unsigned char SMODS : 2;
            unsigned char : 1;
            unsigned char RST : 1;
            unsigned char ST : 1;
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
            unsigned char PP_OD : 1;
            unsigned char IPOL : 1;
            unsigned char : 1;
            unsigned char WAKE_FF_MT : 1;
            unsigned char WAKE_PULSE : 1;
            unsigned char WAKE_LNDPRT : 1;
            unsigned char WAKE_TRANS : 1;
            unsigned char FIFO_GATE : 1;
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
            unsigned char INT_EN_DRDY : 1;
            unsigned char : 1;
            unsigned char INT_EN_FF_MT : 1;
            unsigned char INT_EN_PULSE : 1;
            unsigned char INT_EN_LNDPR : 1;
            unsigned char INT_EN_TRANS : 1;
            unsigned char INT_EN_FIFO : 1;
            unsigned char INT_EN_ASLP : 1;
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
            unsigned char INT_CFG_DRDY : 1;
            unsigned char : 1;
            unsigned char INT_CFG_FF_MT : 1;
            unsigned char INT_CFG_PULSE : 1;
            unsigned char INT_CFG_LNDPRT : 1;
            unsigned char INT_CFG_TRANS : 1;
            unsigned char INT_CFG_FIFO : 1;
            unsigned char INT_CFG_ASLP : 1;
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
        INT_SOURCE_SRC_FIFO = 0x80,
        
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
        FF_MT_CFG_OAE = 0x40,
        FF_MT_CFG_ELE = 0x80,
        
        FF_MT_SRC_XHP = 0x01,
        FF_MT_SRC_XHE = 0x02,
        FF_MT_SRC_YHP = 0x04,
        FF_MT_SRC_YHE = 0x08,
        FF_MT_SRC_ZHP = 0x10,
        FF_MT_SRC_ZHE = 0x20,
        FF_MT_SRC_EA = 0x80,
        
        FF_MT_THS = 0x7f,
        FF_MT_DBCNTM = 0x80,
        
        TRANSIENT_CFG_HPF_BYP = 0x01,
        TRANSIENT_CFG_XTEFE = 0x02,
        TRANSIENT_CFG_YTEFE = 0x04,
        TRANSIENT_CFG_ZTEFE = 0x08,
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
        
        PULSE_CFG_XSPEFE = 0x01,
        PULSE_CFG_XDPEFE = 0x02,
        PULSE_CFG_YSPEFE = 0x04,
        PULSE_CFG_YDPEFE = 0x08,
        PULSE_CFG_ZSPEFE = 0x10,
        PULSE_CFG_ZDPEFE = 0x20,
        PULSE_CFG_ELE = 0x40,
        PULSE_CFG_DPA = 0x80,
        
        PULSE_SRC_POLX = 0x01,
        PULSE_SRC_POLY = 0x02,
        PULSE_SRC_POLZ = 0x04,
        PULSE_SRC_DPE = 0x08,
        PULSE_SRC_AXX = 0x10,
        PULSE_SRC_AXY = 0x20,
        PULSE_SRC_AXZ = 0x40,
        PULSE_SRC_EA = 0x80,
        
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
};

#endif /* __ARDUINO_DRIVER_ACCELEROMETER_MMA8451_REGISTER_H__ */
