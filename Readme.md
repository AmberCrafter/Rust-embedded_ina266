# Usage
---
0. asking mode

ask: 0x55, 0xB0, 0x15, 0x0B, 0x01, 0xDA, 0x13, 0x05, 0x01, 0xC0, 0x86, 0xED
res: 
0x55, 0xDA, 0x13, 0x05, 0x01,   --
0xB0, 0x15, 0x0B, 0x01,          |  custom setting header
0xC0, 0x00, 0x14,               --
0x01, 0x02, 0x03, 0x04,         --  0x01 - 0x0A is the inn266 data stream
0x05, 0x06, 0x07, 0x08,          |  0xFF is set for custom checksum
0x09, 0x0A, 0xFF, 0xED,         --  0xED is stream end flag
0x00, 0x00, 0x00, 0x00,         --  remain
0x00, 0x00, 0x00, 0x00,         --  remain

1. memory setting 

memory.x -> stm32f103c6
memory(stm32f103c8).x -> stm32f103c8

when use memory(stm32f103c8).x, need to rename it in to memroy.x.

2. When flash chip occur memory error, can use the "reset_stm32f103.sh" to reset chip setting.

3. protocol

0x55, 0xDA, 0x13, 0x05, 0x01,   --
0xB0, 0x15, 0x0B, 0x01,          |  custom setting header
0xC0, 0x00, 0x14,               --
0x01, 0x02, 0x03, 0x04,         --  0x01 - 0x0A is the inn266 data stream
0x05, 0x06, 0x07, 0x08,          |  0xFF is set for custom checksum
0x09, 0x0A, 0xFF, 0xED,         --  0xED is stream end flag
0x00, 0x00, 0x00, 0x00,         --  remain
0x00, 0x00, 0x00, 0x00,         --  remain

> checksum: 
> ```
> fn checksum(data: &[u8]) -> u8 {
>     let mut temp:i32 = 0;
>     for &val in data {
>         temp += val as i32;
>     }
>     (temp%255) as u8
> }
> ```

---
## STM32硬體配置簡單手冊

1. 記憶體配置
    ### memory.x
    ```
    # Linker script for the STM#2F103C8T6
    # 參考官方記憶體文件進行配置

    範例:
    MEMORY
    {
        /*Flash memory begins at 0x08000000 and has a size of 64kB*/
        FLASH : ORIGIN = 0x08000000, LENGTH = 64K
        /*RAM begins at 0x20000000 and has a size of 20kB*/
        RAM : ORIGIN = 0x20000000, LENGTH = 20K
    }
    ```

2. cargo編譯目標
   ###.cargo/config
   ```
   [build]
   # Always compile for the instruction set of the STM32F1
   target = "thumbv7m-none-eabi"

   # use the Tlink.x script from the cortex-m-et create
   rustflags = [ "-C", "link-arg=-Tlink.x" ]
   ```
3. cargo函數庫引入
   ### Cargo.toml
   ```
   [dependencies]
    nb = "^0.1.2"
    cortex-m = "^0.6.3"      # Access to the generic ARM peripherals
    cortex-m-rt = "^0.6.12"  # Startup code for the ARM Core
    embedded-hal = "^0.2.4"  # Access to generic embedded functions (`set_high`)
    panic-halt = "^0.2.0"    # Panic handler
    panic-semihosting = "0.5.6"

    # Access to the stm32f103 HAL.
    [dependencies.stm32f1xx-hal]
    # Bluepill contains a 64kB flash variant which is called "medium density"
    features = ["stm32f103", "rt", "medium"]
    version = "^0.6.1"
   ```
4. main code
   ### src/main.rs
   ```
   #![no_std]
   #![no_main]
   use panic_semihosting as _;

   use nb; // used for usart io
   use cortex_m;
   use cortex_m_rt::entry;
   use stm32f1xx_hal::{
      prelude::*,
      pac
   };

   #[entry]
   fn main() -> ! {
      let mut dp = pac::Peripherals::take().unwrap();
      loop {}
   }
   ```

5. build code 
   > cargo build --release
6. flash to chip
   > cargo flash --chip stm32f103C8 --release