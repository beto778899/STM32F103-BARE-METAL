# STM32F103-BARE-METAL
Blue Pill STM32F103 Bare Metal I2C SPI UART PWM Programming

Yet another Bare Metal Implementation For The Blue Pill STM32F103 

Based on the Ideas Taken From This Site https://github.com/aleksey-makarov/stm32.
By Aleksei Makarov who wrote Some Code For His Son Peter.

The Application codes are for the Diferent Pheripherals like UART, SPI, I2C, GPIO, PWM, A0

All the Examples Make Use Of The SysTick Timer.

Tested on the Aliexpress Blue Pill STM32F103 Board

-- Projects --
* GPIO  	  -   Toggles The Pins PC13, PC14 using the SysTick Timer.
* USART 	  -   Reads the PC15 at 300ms Interval, Then Sends The Value To USART.
* ADC_PWM 	- 	Reads The ADC1 periph at 100ms Interval, Then Sends The Value To PWM, LED (Proportional To ADC1)
* SPI_I2C_PWM_BLINK - Counter Loop From 0-255 Sends The Value To SPI, I2C, PWM, LED
* SPI_MAX7219 - Inits and Blinks Some LEDs on The MAX7219 8x8 LED Matrix Module. Using the SPI Protocol
* I2C_PCF8574 - Inits and Blinks Some LEDs on The PCF8574 Module. Using the I2C Protocol
* PID_CONTROL - Implements a Simple PID CONTROL Using ADC1 And PWM
* TEMPLATE  - 	Use this folder as a Skeleton for other Projects
* MAPLE 	  -	   Generated bins inside this Folder are for the maple dfu bootloader.

- Use the TEMPLATE folder as a Skeleton for other Projects

- In the MAPLE folder I copied the GPIO example but the generated bins are for the maple dfu bootloader.
  Previously You have to upload the maple_dfu_bootloader firmware into the Blue Pill Chip.
  It is a lot easier to upload the binary code using the maple_dfu_bootloader.

- To Build the Binaries just Browse into any of the project folder and type make to compile

-- Install --
- For Linux
  apt update
  apt install binutils-arm-none-eabi gcc-arm-none-eabi
  apt install cmake

- For Windows:
  Install ARM Cross-Compiler and Linker from the ARM Developer Website:
  https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

  When Installing Select the "Add path to environment variable" option at the last install step

  Install Msys	When Installing Select the "Add path to environment variable" option
				set PATH=%PATH%;C:\msys\1.0\bin
 

