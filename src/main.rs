#![no_main]
#![no_std]

// use panic_halt as _;
use stm32f1xx_hal as hal;
use hal::{
    delay::Delay, 
    gpio, 
    i2c::{ BlockingI2c, Mode}, 
    pac::{self, interrupt, Interrupt, TIM2, USART2},
    timer::{Timer, Event, CountDownTimer},
    prelude::*, 
    serial:: { self, Serial, StopBits, Config },
};
use nb::block;
use panic_semihosting as _;
use cortex_m::{self, asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use core::{cell::RefCell, ops::DerefMut};

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

type TypeData = [u8; 10];

// setup globally storage
static G_LED: Mutex<RefCell<Option<TypeLED>>> = Mutex::new(RefCell::new(None));
static G_RX: Mutex<RefCell<Option<serial::Rx<USART2>>>> = Mutex::new(RefCell::new(None));
static G_TX: Mutex<RefCell<Option<serial::Tx<USART2>>>> = Mutex::new(RefCell::new(None));
static G_I2C1: Mutex<RefCell<Option<TypeI2C1>>> = Mutex::new(RefCell::new(None));
static G_DATA: Mutex<RefCell<Option<TypeData>>> = Mutex::new(RefCell::new(None));
static G_DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static G_TIM: Mutex<RefCell<Option<CountDownTimer<TIM2>>>> = Mutex::new(RefCell::new(None));

static G_COUNTER: Mutex<RefCell<Option<usize>>> = Mutex::new(RefCell::new(None));
static G_BUFFER: Mutex<RefCell<Option<[u8;16]>>> = Mutex::new(RefCell::new(None));

static G_VALID_CMD: [u8; 12] = [0x55, 0xB0, 0x15, 0x0B, 0x01, 0xDA, 0x13, 0x05, 0x01, 0xC0, 0x86, 0xED];

// interrupt processs
#[interrupt]
fn USART2() {
    // static mut DATA: Option<TypeData> = None;
    // static mut LED: Option<TypeLED> = None;
    static mut BUF: Option<[u8; 16]> = None;
    static mut COUNT: Option<usize> = None;

    // let led = LED.get_or_insert_with(|| {
    //     cortex_m::interrupt::free(|cs| {
    //         // Move LED pin here, leaving a None in its place
    //         G_LED.borrow(cs).replace(None).unwrap()
    //     })
    // });

    // if led.is_set_high().unwrap() {
    //     led.set_low().unwrap();
    // } else {
    //     led.set_high().unwrap();
    // }
    // let data = DATA.get_or_insert_with(|| {
    //     cortex_m::interrupt::free(|cs| {
    //         // Move LED pin here, leaving a None in its place
    //         G_DATA.borrow(cs).replace(None).unwrap()
    //     })
    // });

    let buffer =  BUF.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                G_BUFFER.borrow(cs).replace(None).unwrap()
        })
    });
    let counter = unsafe{
        COUNT.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                G_COUNTER.borrow(cs).replace(None).unwrap()
            })
        })
    };

    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut tx), Some(ref mut rx), Some(ref mut data)) = (
            G_TX.borrow(cs).borrow_mut().deref_mut(),
            G_RX.borrow(cs).borrow_mut().deref_mut(),
            G_DATA.borrow(cs).borrow_mut().deref_mut(),
        ) {
            match block!(rx.read()) {
                Ok(byte) => {
                    // match block!( tx.write(byte) ) {
                    //     Ok(_) => {
                    //     }
                    //     Err(error) => {
                    //     }
                    // }
                    // match block!( tx.write(*counter as u8) ) {
                    //     Ok(_) => {
                    //     }
                    //     Err(error) => {
                    //     }
                    // }
                    if byte==G_VALID_CMD[*counter]{
                        buffer[*counter] = byte;
                        // block!( tx.write(buffer[*counter]) ).unwrap();
                        if *counter>=11 {
                            (*counter) = 0;
                            // for word in buffer {
                            //     block!(tx.write(*word)).unwrap();
                            // }
                            // export_tx(led, tx, *data);
                            export_tx(tx, *data);
                        } else {
                            (*counter)+=1;
                        }
                    } else {
                        (*counter) = 0;
                    }
                }
                Err(error) => {
                }
            }
        }
    });
}

#[interrupt]
fn TIM2() {
    static mut TIM: Option<CountDownTimer<TIM2>> = None;

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });
    
    tim.clear_update_interrupt_flag();

    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut data), Some(ref mut iic), Some(ref mut delay), Some(ref mut led)) = (
            G_DATA.borrow(cs).borrow_mut().deref_mut(),
            G_I2C1.borrow(cs).borrow_mut().deref_mut(),
            G_DELAY.borrow(cs).borrow_mut().deref_mut(),
            G_LED.borrow(cs).borrow_mut().deref_mut(),
        ) {
            led.set_low().unwrap();
            let dummy = read_i2c(iic, delay).unwrap();
            for (i, &val) in dummy.iter().enumerate() {
                data[i] = val;
            }
            delay.delay_ms(10_u16);
            led.set_high().unwrap();
        }
    });

    // cortex_m::interrupt::free(|cs| {
    //     if let (Some(ref mut data), Some(ref mut tx)) = (
    //         G_DATA.borrow(cs).borrow_mut().deref_mut(),
    //         G_TX.borrow(cs).borrow_mut().deref_mut(),
    //     ) {
    //         for word in data {
    //             block!(tx.write(*word)).unwrap();
    //         }
    //     }
    // });

    let _ = tim.wait();
}


#[entry]
fn main() -> ! {
    // step up peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // get rcc (reset control clock) and flash from hal
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    // impl gpio including afio (used to map pin function) and clocks
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
        Config::default().baudrate(115200.bps()).parity_none().stopbits(StopBits::STOP1),
        clocks,
        &mut rcc.apb1
    );

    // impl I2c
    let mut iic1: TypeI2C1 = BlockingI2c::i2c1(
        dp.I2C1,
        (scl,sda),
        &mut afio.mapr,
        Mode::standard(115200.hz()),
        clocks,
        &mut rcc.apb1,
        1000,
        3,
        1000,
        1000
    );

    let data: TypeData = [0xff_u8; 10];

    led.set_high().unwrap();

    // setup global variables
    cortex_m::interrupt::free(|cs| *G_COUNTER.borrow(cs).borrow_mut() = Some(0));
    cortex_m::interrupt::free(|cs| *G_BUFFER.borrow(cs).borrow_mut() = Some([0;16]));

    cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(data));
    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));
    cortex_m::interrupt::free(|cs| *G_I2C1.borrow(cs).borrow_mut() = Some(iic1));
    cortex_m::interrupt::free(|cs| *G_DELAY.borrow(cs).borrow_mut() = Some(delay));
    
    // Set up a timer expiring after 1s
    let mut timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
    // Generate an interrupt when the timer expires
    uart2.listen(serial::Event::Rxne);
    timer.listen(Event::Update);

    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));

    // setup dma for rx
    let (tx, rx) = uart2.split();
    cortex_m::interrupt::free(|cs| *G_TX.borrow(cs).borrow_mut() = Some(tx));
    cortex_m::interrupt::free(|cs| *G_RX.borrow(cs).borrow_mut() = Some(rx));
    

    cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
    cortex_m::peripheral::NVIC::unpend(Interrupt::USART2);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
    }

    loop {
        wfi();
    }
}

fn read_i2c(iic: &mut TypeI2C1, delay: &mut Delay) -> Result<TypeData,()>{
    const IICADDR: u8 = 0x45;
    let mut buffer: TypeData = [0; 10];

    // read bus voltage from ina226 with measurement 1.1ms
    let ina226_vbus_addr = 0..5_u8;
    for addr in ina226_vbus_addr{
        let buffer_offset = (addr)*2;
        if iic.write(IICADDR, &[addr]).is_ok() {
            delay.delay_ms(2_u16);
            iic.read(IICADDR, &mut buffer[buffer_offset.into()..]).unwrap();
        };
    }
    Ok(buffer)
}

// fn export_tx(led: &mut TypeLED, tx: &mut serial::Tx<USART2>, data:TypeData) {
//     let mut buffer: [u8; 32] = [
//         0x55, 0xDA, 0x13, 0x05, 0x01,
//         0xB0, 0x15, 0x0B, 0x01,
//         0xC0, 0x00, 0x14,
//         0x01, 0x02, 0x03, 0x04,
//         0x05, 0x06, 0x07, 0x08,
//         0x09, 0x0A, 0xFF, 0xED,
//         0x00, 0x00, 0x00, 0x00,
//         0x00, 0x00, 0x00, 0x00,
//     ];
//     for (i, &val) in data.iter().enumerate() {
//         buffer[12+i] = val;
//     }
//     buffer[22] = checksum(&buffer[1..22]);

//     led.set_low().unwrap();
//     for val in buffer {
//         block!(tx.write(val)).unwrap();
//     };
//     led.set_high().unwrap();
// }

fn export_tx(tx: &mut serial::Tx<USART2>, data:TypeData) {
    let mut buffer: [u8; 32] = [
        0x55, 0xDA, 0x13, 0x05, 0x01,
        0xB0, 0x15, 0x0B, 0x01,
        0xC0, 0x00, 0x14,
        0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0xFF, 0xED,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    ];
    for (i, &val) in data.iter().enumerate() {
        buffer[12+i] = val;
    }
    buffer[22] = checksum(&buffer[1..22]);

    for val in buffer {
        block!(tx.write(val)).unwrap();
    };
}

fn checksum(data: &[u8]) -> u8 {
    let mut temp:i32 = 0;
    for &val in data {
        temp += val as i32;
    }
    (temp%255) as u8
}