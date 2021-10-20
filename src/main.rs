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
use core::{cell::{Ref, RefCell}, ops::{Deref, DerefMut}};
use stm32f1;


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

// setup globally storage
static G_LED: Mutex<RefCell<Option<TypeLED>>> = Mutex::new(RefCell::new(None));
// static G_UART2: Mutex<RefCell<Option<TypeUART2>>> = Mutex::new(RefCell::new(None));
static G_RX: Mutex<RefCell<Option<serial::Rx<USART2>>>> = Mutex::new(RefCell::new(None));
static G_TX: Mutex<RefCell<Option<serial::Tx<USART2>>>> = Mutex::new(RefCell::new(None));
static G_I2C1: Mutex<RefCell<Option<TypeI2C1>>> = Mutex::new(RefCell::new(None));
static G_DATA: Mutex<RefCell<Option<TypeData>>> = Mutex::new(RefCell::new(None));
static G_DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static G_TIM: Mutex<RefCell<Option<CountDownTimer<TIM2>>>> = Mutex::new(RefCell::new(None));

// interrupt process
#[interrupt]
fn USART2() {
    static mut LED: Option<TypeLED> = None;
    // static mut UART: Option<TypeUART2> = None;
    static mut TX: Option<serial::Tx<USART2>> = None;
    static mut RX: Option<serial::Rx<USART2>> = None;
    static mut DATA: Option<TypeData> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_LED.borrow(cs).replace(None).unwrap()
        })
    });
    // let uart = UART.get_or_insert_with(|| {
    //     cortex_m::interrupt::free(|cs| {
    //         // Move LED pin here, leaving a None in its place
    //         G_UART2.borrow(cs).replace(None).unwrap()
    //     })
    // });
    let tx = TX.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TX.borrow(cs).borrow_mut().replace(None).unwrap()
        })
    });
    let rx = RX.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_RX.borrow(cs).replace(None).unwrap()
        })
    });

    let data = DATA.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_DATA.borrow(cs).replace(None).unwrap()
        })
    });

    // uart.is_rx_not_empty();
    // unsafe{ (*USART2::ptr()).sr.read().rxne().bit() };
    // unsafe{ (*USART2::ptr()).sr.modify(|_,w| w.rxne().set_bit()) };

    // if uart.is_rxne(){

    led.set_low().unwrap();
    block!( tx.write(0x55) ).unwrap();
    block!( tx.write(0x33) ).unwrap();
    let word = block!(rx.read()).unwrap();
    block!( tx.write(word) ).unwrap();

    // } 

    // if led.is_set_high().unwrap() {
    //     // block!(uart2.write(0x00)).unwrap();
    //     led.set_low().unwrap();
    // } else {
    //     // block!(uart2.write(0xff)).unwrap();
    //     led.set_high().unwrap();
    // }

    // let trigger = get_command(led, uart);
    

    // if trigger {
    //     export_uart(led, uart, *data);
    // };
    // uart.flush().unwrap();

    // cortex_m::interrupt::free(|cs| {
    //     if let (Some(ref mut tx), Some(ref mut rx)) = (
    //         G_TX.borrow(cs).borrow_mut().deref_mut(),
    //         G_RX.borrow(cs).borrow_mut().deref_mut(),
    //     ) {
    //         match block!(rx.read()) {
    //             Ok(byte) => {
    //                 match block!(tx.write(byte)) {
    //                     Ok(_) => {
    //                     }
    //                     Err(error) => {
    //                     }
    //                 }
    //             }
    //             Err(error) => {
    //             }
    //         }
    //     }
    // });
}

#[interrupt]
fn TIM2() {
    static mut LED: Option<TypeLED> = None;
    static mut IIC: Option<TypeI2C1> = None;
    static mut DATA: Option<TypeData> = None;
    static mut DELAY: Option<Delay> = None;
    static mut TIM: Option<CountDownTimer<TIM2>> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_LED.borrow(cs).replace(None).unwrap()
        })
    });

    let iic = IIC.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs|{
            G_I2C1.borrow(cs).replace(None).unwrap()
        })
    });

    let data = DATA.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_DATA.borrow(cs).replace(None).unwrap()
        })
    });

    let delay = DELAY.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            G_DELAY.borrow(cs).replace(None).unwrap()
        })
    });

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });

    // *data = read_i2c(iic, delay).unwrap();
    tim.clear_update_interrupt_flag();

    // if led.is_set_high().unwrap() {
    //     // block!(uart2.write(0x00)).unwrap();
    //     led.set_low().unwrap();
    // } else {
    //     // block!(uart2.write(0xff)).unwrap();
    //     led.set_high().unwrap();
    // }

    // block!(uart2.write(0xAA)).unwrap();
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

    let data: TypeData = [0xff_u8; 12];

    led.set_high().unwrap();

    // setup global variables
    cortex_m::interrupt::free(|cs| *G_DATA.borrow(cs).borrow_mut() = Some(data));
    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));
    cortex_m::interrupt::free(|cs| *G_I2C1.borrow(cs).borrow_mut() = Some(iic1));
    cortex_m::interrupt::free(|cs| *G_DELAY.borrow(cs).borrow_mut() = Some(delay));
    
    // Set up a timer expiring after 1s
    let mut timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
    // Generate an interrupt when the timer expires
    uart2.listen(serial::Event::Rxne);
    timer.listen(Event::Update);

    // cortex_m::interrupt::free(|cs| *G_UART2.borrow(cs).borrow_mut() = Some(uart2));
    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));
    let (mut tx, mut rx) = uart2.split();
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
    // let mut count = 0;
    // while let Ok(word) = block!(uart.read()) {
    //     if word==valid_cmd[count] {
    //         count+=1;
    //     } else {
    //         // block!(uart.write(0xFF)).unwrap();
    //         // block!(uart.write(word)).unwrap();
    //         break;
    //     }
    //     if count==12 {
    //         // for word in valid_cmd {
    //         //     block!(uart.write(word)).unwrap();
    //         // }
    //         // break;
    //         led.set_low().unwrap();
    //         return true
    //     }
    // }
    false
}

fn clear_uart(uart: &mut TypeUART2) {
    while let Ok(_) = uart.read() {
        ();
    }
    // while let Ok(word) = uart.read() {
    //     block!(uart.write(word)).unwrap();
    // }
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