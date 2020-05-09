//! DPS422 register addresses
//!
//! See datasheet https://www.infineon.com/dgdl/Infineon-DPS422-DS-v01_03-EN.pdf?fileId=5546d46264fee02f01650249502c1ddf

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[repr(u8)]
pub enum Register {
    PSR_B2 = 0x00,
    PSR_B1 = 0x01,
    PSR_B0 = 0x02,
    TMP_B2 = 0x03,
    TMP_B1 = 0x04,
    TMP_B0 = 0x05,
    PSR_CFG = 0x06,
    TEMP_CFG = 0x07,
    MEAS_CFG = 0x08,
    CFG_REG = 0x09,
    INT_STS = 0x0A,
    WM_CFG = 0x0B,
    FIFO_STS = 0x0C,
    RESET = 0x0D,
    PROD_ID = 0x1D,

    T_GAIN_COEFF = 0x20,
    T_dVBE_COEFF = 0x21,
    T_VBE_COEFF = 0x22,

    COEFF_REG_1 = 0x26,
    COEFF_REG_2 = 0x27,
    COEFF_REG_3 = 0x28,
    COEFF_REG_4 = 0x29,
    COEFF_REG_5 = 0x2A,
    COEFF_REG_6 = 0x2B,
    COEFF_REG_7 = 0x2C,
    COEFF_REG_8 = 0x2D,
    COEFF_REG_9 = 0x2E,
    COEFF_REG_10 = 0x2F,
    COEFF_REG_11 = 0x30,
    COEFF_REG_12 = 0x31,
    COEFF_REG_13 = 0x32,
    COEFF_REG_14 = 0x33,
    COEFF_REG_15 = 0x34,
    COEFF_REG_16 = 0x35,
    COEFF_REG_17 = 0x36,
    COEFF_REG_18 = 0x37,
    COEFF_REG_19 = 0x38,
    COEFF_REG_20 = 0x39,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

}