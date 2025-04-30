# STM32F103xx Drivers

This repository contains a low-level driver library written in C for the STM32F103xx microcontroller series. The aim is to gain a deeper understanding of how STM32 peripherals work at the register level, without using HAL or SPL libraries.

## ✅ Completed
- GPIO (General Purpose Input/Output)

## 🔄 In Progress
- SPI
- I2C
- UART / USART

## 📖 About This Project

This project was created as part of my personal learning journey.  
I am following the **"Mastering Microcontroller and Embedded Driver Development"** course on Udemy, and I’m adapting what I’ve learned there to the **STM32F103xx** series.

The goals of this project are to:
- Improve my knowledge of bare-metal embedded programming  
- Practice register-level driver development  
- Build a reusable and modular driver library from scratch  

## 🛠️ Getting Started

To use the GPIO driver:

1. Add `stm32f103xx_gpio.c` to your project source files.  
2. Include `stm32f103xx.h` and `stm32f103xx_gpio.h` in your main code.  
3. Use the provided API functions to configure and control GPIO pins.  
4. Refer to the example project (`Examples/001_led_toggle`) for usage.

## 📌 Notes
- Drivers are written from scratch using only CMSIS headers and the STM32 reference manual.  
- Example projects will be added for each module.

## 📬 Contributions
Suggestions, improvements, and feedback are always welcome!
