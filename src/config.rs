#![allow(non_camel_case_types)]

#[derive(Copy, Clone, Debug)]
pub enum PressureRate {
    _1_SPS = 0b000,
    _2_SPS = 0b001,
    _4_SPS = 0b010,
    _8_SPS = 0b011,
    _16_SPS = 0b100,
    _32_SPS = 0b101,
    _64_SPS = 0b110,
    _128_SPS = 0b111,
}

impl PressureRate {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl Default for PressureRate {
    fn default() -> Self {
        PressureRate::_1_SPS
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

impl Default for PressureResolution {
    fn default() -> Self {
        PressureResolution::_256_SAMPLES_1X_DECI
    }
}

#[derive(Copy, Clone, Debug)]
pub enum TemperatureRate {
    _1_SPS = 0b000,
    _2_SPS = 0b001,
    _4_SPS = 0b010,
    _8_SPS = 0b011,
    _16_SPS = 0b100,
    _32_SPS = 0b101,
    _64_SPS = 0b110,
}

impl TemperatureRate {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl Default for TemperatureRate {
    fn default() -> Self {
        TemperatureRate::_1_SPS
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

impl Default for TemperatureResolution {
    fn default() -> Self {
        TemperatureResolution::_256_SAMPLES_1X_DECI
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

impl Default for InterruptSource {
    fn default() -> Self {
        InterruptSource::NO_INTERRRUPT
    }
}

/// DPS422 configuration struct
#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub(crate) pres_rate: Option<PressureRate>,
    pub(crate) pres_res: Option<PressureResolution>,
    pub(crate) temp_rate: Option<TemperatureRate>,
    pub(crate) temp_res: Option<TemperatureResolution>,
    pub(crate) int_source: Option<InterruptSource>,
    pub(crate) int_polarity: bool,
    pub(crate) fifo_stop_on_full: bool,
    pub(crate) fifo_enable: bool,
    pub(crate) spi_mode: bool,
    pub(crate) watermark_level: u8
}

impl Config {

     // Creates a new configuration object with default values
     pub fn new() -> Self {
        Config {
            pres_rate: None,
            pres_res: None,
            temp_rate: None,
            temp_res: None,
            int_source: None,
            int_polarity: false,
            fifo_stop_on_full: false,
            fifo_enable: false,
            spi_mode: false,
            watermark_level: 0x1F
        }
    }

    pub fn pres_rate(&mut self, rate: PressureRate) -> &mut Self {
        self.pres_rate = Some(rate);
        self
    }

    pub fn pres_res(&mut self, res: PressureResolution) -> &mut Self {
        self.pres_res = Some(res);
        self
    }

    pub fn temp_rate(&mut self, rate: TemperatureRate) -> &mut Self {
        self.temp_rate = Some(rate);
        self
    }

    pub fn temp_res(&mut self, res: TemperatureResolution) -> &mut Self {
        self.temp_res = Some(res);
        self
    }

    pub fn int_source(&mut self, int_source: InterruptSource, polarity: bool) -> &mut Self {
        self.int_source = Some(int_source);
        self.int_polarity = polarity;
        self
    }

    pub fn fifo(&mut self, stop_on_full: bool, enable: bool) -> &mut Self {
        self.fifo_stop_on_full = stop_on_full;
        self.fifo_enable = enable;
        self
    }

    pub fn spi_mode(&mut self, three_wire: bool) -> &mut Self {
        self.spi_mode = three_wire;
        self
    }

    pub fn watermark_level(&mut self, level: u8) -> &mut Self {
        if level > 0x1F {
            self.watermark_level = 0x1F;
        } else {
            self.watermark_level = level;
        }
        self
    }


}
