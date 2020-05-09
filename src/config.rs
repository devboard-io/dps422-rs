#![allow(non_camel_case_types)]

#[derive(Copy, Clone, Debug)]
pub enum PressureRange {
    _1_SPS = 0b000,
    _2_SPS = 0b001,
    _4_SPS = 0b010,
    _8_SPS = 0b011,
    _16_SPS = 0b100,
    _32_SPS = 0b101,
    _64_SPS = 0b110,
    _128_SPS = 0b111,
}

impl PressureRange {
    pub fn val(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug)]
pub enum PressureResolution {
    _256_SAMPLES_1X_DECI = 0b000,
    _512_SAMPLES_2X_DECI = 0b001,
    _1024_SAMPLES_4X_DEC = 0b010,
    _2048_SAMPLES_8X_DEC = 0b011,
    _4096_SAMPLES_16X_DEC = 0b100,
    _8192_SAMPLES_32X_DEC = 0b101,
    _16384_SAMPLES_64X_DEC = 0b110,
    _32768_SAMPLES_128X_DEC = 0b111,
}

impl PressureResolution {
    pub fn val(self) -> u8 {
        self as u8
    }

    #[allow(non_snake_case)]
    pub fn get_kP_value(self) -> f32 {
        match self {
            PressureResolution::_256_SAMPLES_1X_DECI => 524_288_f32,
            PressureResolution::_512_SAMPLES_2X_DECI => 1_572_864_f32,
            PressureResolution::_1024_SAMPLES_4X_DEC => 3_670_016_f32,
            PressureResolution::_2048_SAMPLES_8X_DEC => 7_864_320_f32,
            PressureResolution::_4096_SAMPLES_16X_DEC => 253_952_f32,
            PressureResolution::_8192_SAMPLES_32X_DEC => 516_096_f32,
            PressureResolution::_16384_SAMPLES_64X_DEC => 1_040_384_f32,
            PressureResolution::_32768_SAMPLES_128X_DEC => 2_088_960_f32,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum TemperatureRange {
    _1_SPS = 0b000,
    _2_SPS = 0b001,
    _4_SPS = 0b010,
    _8_SPS = 0b011,
    _16_SPS = 0b100,
    _32_SPS = 0b101,
    _64_SPS = 0b110,
}

impl TemperatureRange {
    pub fn val(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug)]
pub enum TemperatureResolution {
    _256_SAMPLES_1X_DECI = 0b000,
    _512_SAMPLES_2X_DECI = 0b001,
    _1024_SAMPLES_4X_DEC = 0b010,
    _2048_SAMPLES_8X_DEC = 0b011,
    _4096_SAMPLES_16X_DEC = 0b100,
    _8192_SAMPLES_32X_DEC = 0b101,
    _16384_SAMPLES_64X_DEC = 0b110,
    _32768_SAMPLES_128X_DEC = 0b111,
}

impl TemperatureResolution {
    pub fn val(self) -> u8 {
        self as u8
    }
}
#[derive(Copy, Clone, Debug)]
pub enum InterruptSource {
    NO_INTERRRUPT   = 0b0000,
    PRESSURE_INT    = 0b0001,
    TEMPERATURE_INT = 0b0010,
    PRES_TEMP_INT   = 0b0011,
    FIFO_WATERMARK  = 0b0100,
    FIFO_FULL       = 0b1000,
}

impl InterruptSource {
    pub fn val(self) -> u8 {
        self as u8
    }
}


/// DPS422 configuration struct
pub struct Config {
    pub(crate) pres_range: Option<PressureRange>,
    pub(crate) pres_res: Option<PressureResolution>,
    pub(crate) temp_range: Option<TemperatureRange>,
    pub(crate) temp_res: Option<TemperatureResolution>,
    pub(crate) int_source: Option<InterruptSource>,
    pub(crate) int_polarity: Option<bool>,
    pub(crate) fifo_stop_on_full: Option<bool>,
    pub(crate) fifo_enable: Option<bool>,
    pub(crate) spi_mode: Option<bool>,
    pub(crate) watermark_level: Option<u8>
}

impl Config {

     // Creates a new configuration object with default values
     pub fn new() -> Self {
        Config {
            pres_range: None,
            pres_res: None,
            temp_range: None,
            temp_res: None,
            int_source: None,
            int_polarity: None,
            fifo_stop_on_full: None,
            fifo_enable: None,
            spi_mode: None,
            watermark_level: None
        }
    }

    pub fn pres_range(&mut self, range: PressureRange) -> &mut Self {
        self.pres_range = Some(range);
        self
    }

    pub fn pres_res(&mut self, res: PressureResolution) -> &mut Self {
        self.pres_res = Some(res);
        self
    }

    pub fn temp_range(&mut self, range: TemperatureRange) -> &mut Self {
        self.temp_range = Some(range);
        self
    }

    pub fn temp_res(&mut self, res: TemperatureResolution) -> &mut Self {
        self.temp_res = Some(res);
        self
    }

    pub fn int_source(&mut self, int_source: InterruptSource, polarity: bool) -> &mut Self {
        self.int_source = Some(int_source);
        self.int_polarity = Some(polarity);
        self
    }

    pub fn fifo(&mut self, stop_on_full: bool, enable: bool) -> &mut Self {
        self.fifo_stop_on_full = Some(stop_on_full);
        self.fifo_enable = Some(enable);
        self
    }

    pub fn spi_mode(&mut self, three_wire: bool) -> &mut Self {
        self.spi_mode = Some(three_wire);
        self
    }

    pub fn watermark_level(&mut self, level: u8) -> &mut Self {
        if level > 0x1F {
            self.watermark_level = Some(0x1F);
        } else {
            self.watermark_level = Some(level);
        }
        self
    }


}
