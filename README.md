# STM32F103xx Drivers

This repository contains a low-level driver library written in C for the STM32F103xx microcontroller series. The aim is to gain a deeper understanding of how STM32 peripherals work at the register level, without using HAL or SPL libraries.

## ✅ Completed
- GPIO (General Purpose Input/Output)
- SPI (Serial Peripheral Interface)
- I2C (Inter-Integrated Circuit)
  
## 🔄 In Progress
- UART / USART

## 📖 About This Project

This project was created as part of my personal learning journey.  
I am following the **"Mastering Microcontroller and Embedded Driver Development"** course on Udemy, and I’m adapting what I’ve learned there to the **STM32F103xx** series.

The goals of this project are to:
- Improve my knowledge of bare-metal embedded programming  
- Practice register-level driver development  
- Build a reusable and modular driver library from scratch  

## 🛠️ Getting Started

1. Place the source files (.c) into your project's src folder and the header files (.h) into the inc folder.
2. Include the necessary headers in your source code:

```c
#include "stm32f103xx.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_spi.h"
#include "stm32f103xx_i2c.h"
#include "stm32f103xx_rcc.h"
```

3. Use the provided API functions to configure and control GPIO, SPI, and I2C peripherals.
4. Check out the example projects for usage:
   - GPIO: Examples/001_led_toggle
   - SPI: Examples/004_spi_tx
   - I2C: Examples/005_i2c_master_tx
   
## 📌 Notes
- Drivers are written from scratch using only CMSIS headers and the STM32 reference manual.  
- Example projects will be added for each module.

## 📬 Contributions
Suggestions, improvements, and feedback are always welcome!
