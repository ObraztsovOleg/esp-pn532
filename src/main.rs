#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::{I2C, self},
    peripherals::{Peripherals, I2C0, TIMG0},
    prelude::*,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Uart,
    timer::{TimerGroup, Timer0}, Timer, Delay,
};
use pn532::{i2c::I2CInterface, Pn532, Request, requests::SAMMode};
use format_no_std;
use fugit::MicrosDurationU64;
use esp_backtrace as _;

const SUCCESS: u8 = 0x00;
const FAILED: u8 = 0x01;
const ACK: [u8; 3] = [0x00, 0x90, 0x00];


#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let config = Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio8.into_push_pull_output(),
        io.pins.gpio9.into_floating_input(),
    );

    let mut serial = Uart::new_with_config(peripherals.UART1, config, Some(pins), &clocks);

    let i2c_driver = I2C::new(
        peripherals.I2C0,
        io.pins.gpio6,
        io.pins.gpio5,
        100u32.kHz(),
        &clocks,
    );

    let mut led = io.pins.gpio15.into_push_pull_output();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer_driver = timer_group0.timer0;
    
    let interface = I2CInterface { i2c: i2c_driver };
    let mut pn532: Pn532<I2CInterface<I2C<'_, I2C0>>, esp32s2_hal::Timer<esp32s2_hal::timer::Timer0<TIMG0>>, 32> = Pn532::new(interface, timer_driver);


    serial.write_bytes(b"HELLO\n\r");
    let mut delay = Delay::new(&clocks);
    let mut delay_value: u32 = 500;
    let mut buf = [0u8; 2048];
    let timeout = MicrosDurationU64::micros(10000000);

    match pn532.process(
        &Request::GET_FIRMWARE_VERSION, 4, timeout
    ) {
        Ok(v) => {
            let s: &str = format_no_std::show(
                &mut buf,
                format_args!("Success VER {:?}\n\r", v),
            ).unwrap();

            serial.write_bytes(s.as_bytes());
            // delay_value = 200u32;
        },
        Err(e) => {
            let s: &str = format_no_std::show(
                &mut buf,
                format_args!("Failed VER {:?}\n\r", e),
            ).unwrap();

            serial.write_bytes(s.as_bytes());
        }
    }

    let select_apdu = [
        0x00, /* CLA */
        0xA4, /* INS */
        0x04, /* P1  */
        0x00, /* P2  */
        0x07, /* Length of AID  */
        0xA0, 0x00, 0x00, 0x02, 0x47, 0x10, 0x01, /* AID defined on Android App */
        0x00  /* Le  */
    ];

    match pn532.process(
        &Request::sam_configuration(SAMMode::Normal, 0x01), 0, timeout
    ) {
        Ok(v) => {
            let s: &str = format_no_std::show(
                &mut buf,
                format_args!("Success SAM {:?}\n\r", v),
            ).unwrap();

            serial.write_bytes(s.as_bytes());
        },
        Err(e) => {
            let s: &str = format_no_std::show(
                &mut buf,
                format_args!("Failed SAM {:?}\n\r", e),
            ).unwrap();

            serial.write_bytes(s.as_bytes());
        }
    }

    loop {
        match pn532.process(&Request::INLIST_ONE_ISO_A_TARGET, 10, timeout) {
            Ok(v) => {
                let s: &str = format_no_std::show(
                    &mut buf,
                    format_args!("Success ISO {:?}\n\r", v),
                ).unwrap();
    
                serial.write_bytes(s.as_bytes());
            },
            Err(e) => {
                let s: &str = format_no_std::show(
                    &mut buf,
                    format_args!("Failed ISO {:?}\n\r", e),
                ).unwrap();
    
                serial.write_bytes(s.as_bytes());
            }
        }

        let connected = pn532.in_data_exchange(
            &Request::exchange_data(0x01), 3, &select_apdu, timeout
        ).unwrap();

        if connected == ACK {
            let mut received = false;
            let mut client_cert: [u8; 377] = [0; 377];
            while !received {
                match pn532.in_data_exchange(
                    &Request::exchange_data(0x01), 21, &[SUCCESS], timeout
                ) {
                    Ok(v) => {
                        if v == ACK { received = true }

                        let s: &str = format_no_std::show(
                            &mut buf,
                            format_args!("Success answer {:?}\n\r", v),
                        ).unwrap();
            
                        serial.write_bytes(s.as_bytes());
                    },
                    Err(e) => {
                        let s: &str = format_no_std::show(
                            &mut buf,
                            format_args!("Failed answer {:?}\n\r", e),
                        ).unwrap();
            
                        serial.write_bytes(s.as_bytes());
                    }   
                }
            }
        }

        led.toggle().unwrap();
        delay.delay_ms(delay_value)
    }
}
