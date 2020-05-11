//!
//! DPS422 embedded-hal I2C driver crate
//!
//! A platform agnostic driver to interface with the DPS422 barometric pressure & temp sensor.
//! This driver uses I2C via [embedded-hal]. Note that the DPS422 also supports SPI, however that
//! is not (yet) implemented in this driver.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal


#![no_std]

mod register;
mod config;

use embedded_hal as hal;
use hal::blocking::i2c;

pub use register::Register;
pub use config::*;

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
    coeffs: CalbrationCoeffs,
    oversampling_rate: PressureResolution
}

impl<I2C, E> DPS422<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    pub fn new(i2c: I2C, address: u8, config: &Config) -> Result<Self, E>  {

        let mut dps422 = Self {
            i2c,
            address,
            coeffs: CalbrationCoeffs::default(),
            oversampling_rate: config.pres_res.unwrap_or_default()
        };


        let id: u8 = dps422.get_product_id()?;
        if id != PRODUCT_ID {

        }

        dps422.read_calbration_coefficients().ok();
        dps422.apply_config(config)?;
        dps422.standby().ok();

        Ok(dps422)
    }

    fn apply_config(&mut self, config: &Config) -> Result<(), E> {
        let psr_cfg = ((config.pres_rate.unwrap_or_default() as u8) << 4) | (config.pres_res.unwrap_or_default() as u8);
        self.write_reg(Register::PSR_CFG, psr_cfg)?;

        // IMPORTANT: the MSB of the TEMP_CFG register MUST be set for temperature readings to work
        // without the temperature values make no sense.
        let temp_cfg = 0x80 | (config.temp_rate.unwrap_or_default() as u8) << 4 | (config.temp_res.unwrap_or_default() as u8);
        self.write_reg(Register::TEMP_CFG, temp_cfg)?;

        let cfg = ((config.int_source.unwrap_or_default() as u8) << 4) |
            ((config.int_polarity as u8) << 3) |
            ((config.fifo_stop_on_full as u8) << 2) |
            ((config.fifo_enable as u8) << 1) |
            ((config.spi_mode as u8) << 0);

        self.write_reg(Register::CFG_REG, cfg)?;

        self.write_reg(Register::WM_CFG, config.watermark_level)?;

        Ok(())
    }

    /// Set measurement mode to `idle`
    fn standby(&mut self) -> Result<(), E> {
        self.write_reg(Register::MEAS_CFG, 0)
    }

    /// Returns the product ID from PROD_ID register.
    /// This value is expected to be 0x1A
    pub fn get_product_id(&mut self) -> Result<u8, E> {
        let id = self.read_reg(Register::PROD_ID)?;
        Ok(id)
    }

    /// Read status bits from MEAS_CFG reg.
    /// MEAS_CFG register is maskedd with 0xF0
    pub fn read_status(&mut self) -> Result<u8, E> {
        let meas_cfg = self.read_reg(Register::MEAS_CFG)?;
        Ok(meas_cfg & 0xF0)
    }

    /// Start a single or continuous measurement for `pres`sure or `temp`erature
    pub fn trigger_measurement(&mut self, temp: bool, pres: bool, continuous: bool) -> Result<(), E> {
        self.write_reg(Register::MEAS_CFG, (continuous as u8) << 2 | (temp as u8) << 1 | pres as u8)
    }

    /// returns the init_complete bit from the status register
    pub fn init_complete(&mut self) -> Result<bool, E> {
        let status = self.read_status()?;
        Ok((status & 0x80) != 0)
    }

    /// returns the data_ready bit from the status register
    pub fn data_ready(&mut self) -> Result<bool, E> {
        let status = self.read_status()?;
        // todo only return pressure ready for now
        Ok((status & 0x10) != 0)
    }

    /// Read raw temperature contents
    fn read_temp_raw(&mut self) -> Result<i32, E> {
        let mut bytes: [u8; 3] = [0, 0, 0];
        self.i2c.write_read(self.address, &[Register::TMP_B2.addr()], &mut bytes)?;
        let temp: i32 = ((((bytes[0] as i32) << 24) | ((bytes[1] as i32) << 16) | (bytes[2] as i32) << 8)) >> 8;

        Ok(temp)
    }

    /// See section 5.1.1: only used for pressure correction
    fn read_temp_scaled(&mut self) -> Result<f32, E> {
        let temp_x: f32 = self.read_temp_raw()? as f32 / 1048576.0;
        Ok((8.5 * temp_x) / (1.0 + 8.8 * temp_x))
    }

    /// Read calibrated temperature data in degrees Celsius
    /// This method uses the pre calculated constants based on the calibration coefficients
    /// See section 6.2 in the datasheet
    pub fn read_temp_calibrated(&mut self) -> Result<f32, E> {

        const ALPHA: f32 = 9.45;

        let t_raw = self.read_temp_raw()?;
        let t_cal: f32 =  t_raw as f32 / 1048576.0;
        let mu: f32 = t_cal / (1.0 + ALPHA * t_cal);

        Ok((self.coeffs.T_A * mu) + self.coeffs.T_B)
    }

    /// Read raw pressure contents
    pub fn read_pressure_raw(&mut self) -> Result<i32, E> {

        let mut bytes: [u8; 3] = [0, 0, 0];
        self.i2c.write_read(self.address, &[Register::PSR_B2.addr()], &mut bytes)?;

        let pressure: i32 = ((((bytes[0] as i32) << 24) | ((bytes[1] as i32) << 16) | (bytes[2] as i32) << 8)) >> 8;
        Ok(pressure)
    }

    fn read_pressure_scaled(&mut self) -> Result<f32, E> {

        let pres_raw = self.read_pressure_raw()?;
        let k_p = self.oversampling_rate.get_kP_value();
        let pres_scaled = pres_raw as f32 / k_p;

        Ok(pres_scaled)
    }

    /// Read calibrated pressure data in kPa
    /// This method uses the calibration coefficients
    /// See section 5.1 in the datasheet
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

    /// Issue a full reset and fifo flush
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_reg(Register::RESET, 0b10001001)
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

    fn _write_byte(&mut self, reg: u8, value: u8) -> Result<(), E> {
        let bytes = [reg, value];
        self.i2c.write(self.address, &bytes)
    }

    /// Writes to undocumented registers. Taken from official DSP422 Arduino driver
    ///
    /// See https://github.com/Infineon/DPS422-Library-Arduino/blob/63a4abc442a00cfe166988dcc3b844c281380cea/src/DpsClass.cpp#L442
    fn _correct_temp(&mut self) -> Result<(), E> {
        self._write_byte(0x0E, 0xA5)?;
        self._write_byte(0x0F, 0x96)?;
        self._write_byte(0x62, 0x02)?;
        self._write_byte(0x0E, 0x00)?;
        self._write_byte(0x0F, 0x00)?;
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
