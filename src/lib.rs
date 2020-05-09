#![no_std]

use embedded_hal as hal;
use hal::blocking::i2c;

// DPS422 register addresses

// See datasheet https://www.infineon.com/dgdl/Infineon-DPS422-DS-v01_03-EN.pdf?fileId=5546d46264fee02f01650249502c1ddf

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

const PRODUCT_ID: u8 = 0x1A;

#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub struct CalbrationCoeffs {
    pub C00: i32,
    pub C01: i32,
    pub C02: i32,
    pub C10: i32,
    pub C11: i32,
    pub C12: i32,
    pub C20: i32,
    pub C21: i32,
    pub C30: i32,
    pub T_gain: i32,
    pub T_dVbe: i32,
    pub T_Vbe: i32,
    pub T_A: f32,
    pub T_B: f32
}

impl CalbrationCoeffs {

    fn default() -> Self {
        Self {
            C00: 0,
            C01: 0,
            C02: 0,
            C10: 0,
            C11: 0,
            C12: 0,
            C20: 0,
            C21: 0,
            C30: 0,
            T_gain: 0,
            T_dVbe: 0,
            T_Vbe: 0,
            T_A: 0f32,
            T_B: 0f32
        }
    }
}

pub struct DPS422<I2C> {
    i2c: I2C,
    address: u8,
    pub coeffs: CalbrationCoeffs
}

impl<I2C, E> DPS422<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {

        let mut dps422 = Self {
            i2c,
            address,
            coeffs: CalbrationCoeffs::default()
        };


        let id: u8 = dps422.get_product_id().ok().unwrap();
        if id == PRODUCT_ID {
            dps422.read_calbration_coefficients().ok();
            // dps422.read_calbration_coefficients().ok();
            // IMPORTANT: the MSB of the TEMP_CFG register MUST be set for temperature readings to work
            // without the temperature values make no sense.
            dps422.write_reg(Register::TEMP_CFG, 0x80);

            dps422.standby().ok();
            // dps422.reset().ok();
        }

        dps422
    }

    fn standby(&mut self) -> Result<(), E> {
        // measurement mode = idle
        self.write_reg(Register::MEAS_CFG, 0)
    }

    pub fn get_product_id(&mut self) -> Result<u8, E> {
        let id = self.read_reg(Register::PROD_ID)?;
        Ok(id)
    }

    pub fn read_status(&mut self) -> Result<u8, E> {
        let meas_cfg = self.read_reg(Register::MEAS_CFG)?;
        Ok(meas_cfg & 0xF0)
    }

    pub fn trigger_measurement(&mut self) -> Result<(), E> {
        self.write_reg(Register::MEAS_CFG, 0b011)
    }

    pub fn init_complete(&mut self) -> Result<bool, E> {
        let status = self.read_status()?;
        Ok((status & 0x80) != 0)
    }
    pub fn data_ready(&mut self) -> Result<bool, E> {
        let status = self.read_status()?;
        // todo only return pressure ready for now
        Ok((status & 0x10) != 0)
    }

    fn read_temp_raw(&mut self) -> Result<i32, E> {
        let mut bytes: [u8; 3] = [0, 0, 0];
        self.i2c.write_read(self.address, &[Register::TMP_B2.addr()], &mut bytes)?;
        let temp: i32 = ((((bytes[0] as i32) << 24) | ((bytes[1] as i32) << 16) | (bytes[2] as i32) << 8)) >> 8;

        Ok(temp)
    }

    // 5.1.1
    pub fn read_temp_scaled(&mut self) -> Result<f32, E> {
        let temp_x: f32 = self.read_temp_raw()? as f32 / 1048576.0;
        Ok((8.5 * temp_x) / (1.0 + 8.8 * temp_x))
    }

    // 6.2
    pub fn read_temp_calibrated(&mut self) -> Result<f32, E> {

        const ALPHA: f32 = 9.45;

        let t_raw = self.read_temp_raw()?;
        let t_cal: f32 =  t_raw as f32 / 1048576.0;
        let mu: f32 = t_cal / (1.0 + ALPHA * t_cal);

        Ok((self.coeffs.T_A * mu) + self.coeffs.T_B)
    }

    pub fn read_pressure_raw(&mut self) -> Result<i32, E> {

        let mut bytes: [u8; 3] = [0, 0, 0];
        self.i2c.write_read(self.address, &[Register::PSR_B2.addr()], &mut bytes)?;

        let pressure: i32 = ((((bytes[0] as i32) << 24) | ((bytes[1] as i32) << 16) | (bytes[2] as i32) << 8)) >> 8;
        Ok(pressure)
    }

    pub fn read_pressure_scaled(&mut self) -> Result<f32, E> {

        let pres_raw = self.read_pressure_raw()?;
        let k_p = 524288.0; // todo depends on oversampling config
        let pres_scaled = pres_raw as f32 / k_p;

        Ok(pres_scaled)
    }

    pub fn read_pressure_calibrated(&mut self) -> Result<f32, E> {

        let pres_scaled = self.read_pressure_scaled()?;

        let temp_scaled = self.read_temp_scaled()?;

        let pres_cal =
            self.coeffs.C00 as f32 +
            (self.coeffs.C10 as f32 * pres_scaled) +
            (self.coeffs.C01 as f32 * temp_scaled) +
            (self.coeffs.C20 as f32 * pres_scaled * pres_scaled) +
            (self.coeffs.C02 as f32 * temp_scaled * temp_scaled) +
            (self.coeffs.C30 as f32 * pres_scaled * pres_scaled * pres_scaled) +
            (self.coeffs.C11 as f32 * pres_scaled * temp_scaled) +
            (self.coeffs.C12 as f32 * pres_scaled * temp_scaled * temp_scaled) +
            (self.coeffs.C21 as f32 * pres_scaled * pres_scaled * temp_scaled);

        Ok(pres_cal)
    }

    fn reset(&mut self) -> Result<(), E> {
        self.write_reg(Register::RESET, 0b1001)
    }

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), E> {
        let bytes = [reg.addr(), value];
        self.i2c.write(self.address, &bytes)
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, E> {
        let mut buffer: [u8; 1] = [0];
        self.i2c.write_read(self.address, &[reg.addr()], &mut buffer)?;
        Ok(buffer[0])
    }

    fn write_byte(&mut self, reg: u8, value: u8) -> Result<(), E> {
        let bytes = [reg, value];
        self.i2c.write(self.address, &bytes)
    }

    fn correct_temp(&mut self) -> Result<(), E> {
        self.write_byte(0x0E, 0xA5)?;
        self.write_byte(0x0F, 0x96)?;
        self.write_byte(0x62, 0x02)?;
        self.write_byte(0x0E, 0x00)?;
        self.write_byte(0x0F, 0x00)?;
        Ok(())
    }

    fn read_calbration_coefficients(&mut self) -> Result<(), E> {

        let mut bytes: [u8; 20] = [0; 20];
        self.i2c.write_read(self.address, &[Register::COEFF_REG_1.addr()], &mut bytes)?;

        // C00 20bits 2's complement
        self.coeffs.C00 = ((((bytes[0] as i32) << 24) | ((bytes[1] as i32) << 16) | ((bytes[2] & 0xF0) as i32) << 8)) >> (32-20);

        // C10 20bits 2's complement
        self.coeffs.C10 = (((((bytes[2] & 0x0F) as i32) << 28) | ((bytes[3] as i32) << 20) | (bytes[4] as i32) << 12)) >> (32-20);

        // C01 20bits 2's complement
        self.coeffs.C01 = ((((bytes[5] as i32) << 24) | ((bytes[6] as i32) << 16) | ((bytes[7] & 0xF0) as i32) << 8)) >> (32-20);

        // C02 20bits 2's complement
        self.coeffs.C02 = (((((bytes[7] & 0x0F) as i32) << 28) | ((bytes[8] as i32) << 20) | (bytes[9] as i32) << 12)) >> (32-20);

        // C20 15 bits 2's complement
        self.coeffs.C20 = (((((bytes[10] & 0x7F) as i32) << 9) | ((bytes[11] as i32) << 1)) << 16) >> (32-15);

        //C30 12 bits 2's complement
        self.coeffs.C30 = ( ( ((bytes[12] & 0x0F) as i32) << 8 ) | ( (bytes[13] as i32) << 0) ) << (32-12) >> (32-12) ;

        //C11 17 bits 2's complement
        self.coeffs.C11 = (((bytes[14] as i32) << 16) | ((bytes[15] as i32) << 8) | ((bytes[16] & 0x80) as i32)) << 8 >> (32-17);

        //C12 17 bits 2's complement
        self.coeffs.C12 = ((((bytes[16] & 0x7F) as i32) << (16+1)) | ((bytes[17] as i32) << (8+1)) | (((bytes[18] & 0xC0) as i32) << 1)) << 8 >> (32-17);

        //C21 14 bits 2's complement
        self.coeffs.C21 = ((((bytes[18] & 0x3F) as i32) << 10) | ((bytes[19] as i32) << 2)) << 16 >> (32-14);


        // Calculate temperature coefficients

        const T_REF: f32 = 27.0;
        const V_BE_TARGET_TREF: f32 = 0.687027;
        const ALPHA: f32 = 9.45;
        const T_C_VBE: f32 = -1.735E-3;
        const K_PTAT_CORNER: f32 = -0.8;
        const K_PTAT_CURVATURE: f32 = 0.039;
        const A0: f32 = 5030.0;

        // datasheet 6.1
        // step 1 Read T_Vbe, T_dVbe and T_gain:

        let mut bytes: [u8; 3] = [0; 3];
        self.i2c.write_read(self.address, &[Register::T_GAIN_COEFF.addr()], &mut bytes)?;

        // 8 bits 2's complement
        self.coeffs.T_gain = (bytes[0] as i32) << 24 >> (32-8);

        // 7 bits 2's complement
        self.coeffs.T_dVbe = ((bytes[1] & 0xF7) as i32) << 24 >> (32-7);

        // 9 bit 2's complement
        // todo verify: maybe bytes[1] & 0x01 is bit 8
        self.coeffs.T_Vbe = (((bytes[2] as i32) << 1) | ((bytes[1] & 0x01) as i32)) << (32-9) >> (32-9);
        // self.coeffs.T_Vbe = (((bytes[2] as i32) << 0) | ((bytes[1] & 0x01) as i32) << 8) << (32-9) >> (32-9);

        // step 2 Calculate V_BE, ΔV_BE and A_ADC
        let v_be: f32 = self.coeffs.T_Vbe as f32 * 1.05031E-4 + 0.463232422;
        let delta_vbe = self.coeffs.T_dVbe as f32 * 1.25885E-5 + 0.04027621;
        let a_adc = self.coeffs.T_gain as f32 * 8.4375E-5 + 0.675;

        // step 3
        let vbe_cal = v_be / a_adc;
        let delta_vbe_cal = delta_vbe / a_adc;

        // step 4
        let t_calib = A0 * delta_vbe_cal - 273.15;

        // step 5
        let vbe_cal_tref = vbe_cal - (t_calib - T_REF) * T_C_VBE;

        // step 6
        let k_ptat = (V_BE_TARGET_TREF - vbe_cal_tref) * K_PTAT_CORNER + K_PTAT_CURVATURE;

        // step 7
        self.coeffs.T_A = A0 * (vbe_cal + ALPHA * delta_vbe_cal) * (1.0 + k_ptat);
        self.coeffs.T_B = -273.15 * (1.0 + k_ptat) - k_ptat * t_calib;

        Ok(())
    }
}
