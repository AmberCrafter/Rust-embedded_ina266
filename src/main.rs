#![no_std]
#![no_main]

use nb::block;
use panic_semihosting as _;
use cortex_m;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{delay::Delay, gpio, i2c::{ BlockingI2c, Mode}, pac, prelude::*, serial:: { Serial, StopBits, Config }};

type TypeUART2 = Serial<pac::USART2,
    (
        gpio::gpioa::PA2<gpio::Alternate<gpio::PushPull>>,
        gpio::gpioa::PA3<gpio::Input<gpio::Floating>>
    )>;

type TypeI2C1 = BlockingI2c<pac::I2C1, 
    (
        gpio::gpiob::PB8<gpio::Alternate<gpio::OpenDrain>>,
        gpio::gpiob::PB9<gpio::Alternate<gpio::OpenDrain>>
    )>;

type TypeLED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

type TypeData = [u8; 12];

#[entry]
fn main() -> ! {
    // step up peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // get rcc (reset control clock) and flash from hal
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    // impl gpio including afio (used to map pin function) and clocks
    // global setting
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // led
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // usart2 use pa2, pa3
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let (tx, rx) = (
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3
    );

    // i2c use pb8, pb9
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let (scl, sda) = (
        gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh),
        gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh)
    );

    // impl delay function
    let mut delay = Delay::new(cp.SYST, clocks);

    // impl UART2
    let mut uart2: TypeUART2 = Serial::usart2(
        dp.USART2,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(57600.bps()).parity_none().stopbits(StopBits::STOP1),
        clocks,
        &mut rcc.apb1
    );

    // impl I2c
    let mut iic1: TypeI2C1 = BlockingI2c::i2c1(
        dp.I2C1,
        (scl,sda),
        &mut afio.mapr,
        Mode::standard(57600.hz()),
        clocks,
        &mut rcc.apb1,
        1000,
        3,
        1000,
        1000
    );


    let mut is_cmd: bool = false;
    led.set_high().unwrap();
    loop {
        is_cmd = get_command(&mut led, &mut uart2);
        if is_cmd {
            // let buffer = read_i2c(&mut iic1, &mut delay).unwrap();
            let buffer = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C];
            export_uart(&mut led, &mut uart2, buffer);
        }
        led.set_high().unwrap();
    }
}

fn read_i2c(iic: &mut TypeI2C1, delay: &mut Delay) -> Result<TypeData,()>{
    const IICADDR: u8 = 0x45;
    // const IICOPC1: u8 = 0x00;
    // const IICOPC2: u8 = 0x00;

    let mut buffer: TypeData = [0; 12];

    // read bus voltage from ina226 with measurement 1.1ms
    let ina226_vbus_addr = 0..6_u8;
    for addr in ina226_vbus_addr{
        let buffer_offset = (addr as usize)*2;
        if iic.write(IICADDR, &[addr]).is_ok() {
            delay.delay_ms(2_u16);
            iic.read(IICADDR, &mut buffer[buffer_offset..]).unwrap();
        };
    }
    Ok(buffer)
}

fn get_command(led: &mut TypeLED, uart: &mut TypeUART2) -> bool{
    let valid_cmd: [u8; 12] = [0x55, 0xB0, 0x15, 0x0B, 0x01, 0xDA, 0x13, 0x05, 0x01, 0xC0, 0x86, 0xED];

    clear_uart(uart);
    let mut count = 0;
    while let Ok(word) = block!(uart.read()) {
        if word==valid_cmd[count] {
            count+=1;
        } else {
            // block!(uart.write(0xFF)).unwrap();
            // block!(uart.write(word)).unwrap();
            break;
        }
        if count==12 {
            // for word in valid_cmd {
            //     block!(uart.write(word)).unwrap();
            // }
            // break;
            led.set_low().unwrap();
            return true
        }
    }
    false
}

fn clear_uart(uart: &mut TypeUART2) {
    while let Ok(_) = uart.read() {
        ();
    }
}


fn export_uart(led: &mut TypeLED, uart: &mut TypeUART2, data:TypeData) {
    led.set_high().unwrap();
    block!(uart.write(0x55)).unwrap();
    block!(uart.write(0xDB)).unwrap();
    for val in data {
        block!(uart.write(val)).unwrap();
    };
    block!(uart.write(0xED)).unwrap();
    led.set_low().unwrap();
}