#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
extern crate panic_halt;

use rt::entry;


// use embedded_hal::digital::v2::OutputPin;
use core::fmt::Write;

use stm32g0xx_hal::{
    prelude::*,
    stm32,
    serial::Config,
    i2c,
};

use dps422::DPS422;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led1 = gpiob.pb5.into_push_pull_output();
    led1.set_high().unwrap();

    let mut led2 = gpiob.pb9.into_push_pull_output();
    led2.set_low().unwrap();


    let gpioa = dp.GPIOA.split(&mut rcc);


    let tx = gpioa.pa9;
    let rx = gpioa.pa10;
    let mut usart = dp
        .USART1
        .usart(tx, rx, Config::default().baudrate(115200.bps()), &mut rcc)
        .unwrap();

    writeln!(usart, "Hallo, brievenbus!\n").unwrap();

    // I2C pins
    // PB6 = SCL, yellow, links boven TP5(rood)
    // PB7 = SDA, green, midden boven TP6(zwart)
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();

    let i2c = dp
        .I2C1
        .i2c(sda, scl, i2c::Config::with_timing(0x2020151b), &mut rcc);

    writeln!(usart, "i2c initialized!\n").unwrap();

    let address = 0x76;

    // let reg = [0x1D];
    // let mut data = [0];

    // i2c.write_read(address,  &reg, &mut data).unwrap();
    // writeln!(usart, "pressure sensor id: Got {} from reg {}!", data[0], reg[0]).unwrap();

    let mut dps = DPS422::new(i2c, address);
    let mut init_done = false;
    writeln!(usart, "Wait for init done..").unwrap();
    while !init_done {
        let compl = dps.init_complete();
        init_done = match compl {
            Ok(c) => c,
            Err(_e) => false
        };

    }
    writeln!(usart, "pressure sensor init done: {:?}", dps.coeffs).unwrap();
    loop {


        dps.trigger_measurement().unwrap();

        for _ in 0..10_000 {
            led1.set_low().unwrap();
        }
        for _ in 0..10_000 {
            led1.set_high().unwrap();
        }
        led2.toggle().unwrap();

        // let status = dps.read_status().unwrap();
        // writeln!(usart, "status: {}", status).unwrap();


        // while dps.data_ready().unwrap() == false {}

        let pressure = dps.read_pressure_calibrated().unwrap();

        let temp = dps.read_temp_calibrated().unwrap();

        writeln!(usart, "pressure: {}, {}", pressure, temp).unwrap();

        // writeln!(usart, "{}, {}, {}", accel.x, accel.y, accel.z).unwrap();

    }
}
