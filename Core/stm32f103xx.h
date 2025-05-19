/*
 * stm32f103xx.h
 *
 *  Created on: Apr 17, 2025
 *      Author: ASA
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stddef.h>
#include <stdint.h>

/****************************************Processor Specified Deteails**************************************************/

/*
 * ARM Cortex M3 Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0							( (volatile uint32_t*)0xE000E100 )	//
#define NVIC_ISER1							( (volatile uint32_t*)0xE000E104 )	//
#define NVIC_ISER2							( (volatile uint32_t*)0xE000E108 )	//

/*
 * ARM Cortex M3 Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0							( (volatile uint32_t*)0XE000E180 )	//
#define NVIC_ICER1							( (volatile uint32_t*)0XE000E184 )	//
#define NVIC_ICER2							( (volatile uint32_t*)0XE000E188 )	//

/*
 * ARM Cortex M3 Processor Priority Register Addresses
 */
#define NVIC_PR_BASE_ADDR					( (volatile uint32_t*)0xE000E400 ) 	// Base address for NVIC priority registers


/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR							0x08000000U			// Flash memory starting address
#define SRAM_BASEADDR							0x20000000U 		// SRAM memory starting address
#define SRAM									SRAM_BASEADDR		// Alias for SRAM

/*
 * AHB and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR        					0x40000000U			// General peripheral starting address
#define APB1PERIPH_BASEADDR    					PERIPH_BASEADDR		// APB1 bus starting address
#define APB2PERIPH_BASEADDR    					0x40010000U			// APB2 bus starting address
#define AHBPERIPH_BASEADDR     					0x40018000U			// AHB bus starting address

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define SPI2_BASEADDR							(APB1PERIPH_BASEADDR + 0x3800) // SPI2 peripheral base address on APB1 bus
#define SPI3_BASEADDR							(APB1PERIPH_BASEADDR + 0x3C00) // SPI3 peripheral base address on APB1 bus

#define USART2_BASEADDR							(APB1PERIPH_BASEADDR + 0x4400) // USART2 peripheral base address on APB1 bus
#define USART3_BASEADDR							(APB1PERIPH_BASEADDR + 0x4800) // USART3 peripheral base address on APB1 bus
#define UART4_BASEADDR							(APB1PERIPH_BASEADDR + 0x4C00) // UART4 peripheral base address on APB1 bus
#define UART5_BASEADDR							(APB1PERIPH_BASEADDR + 0x5000) // UART5 peripheral base address on APB1 bus

#define I2C1_BASEADDR							(APB1PERIPH_BASEADDR + 0x5400) // I2C1 peripheral base address on APB1 bus
#define I2C2_BASEADDR							(APB1PERIPH_BASEADDR + 0x5800) // I2C2 peripheral base address on APB1 bus

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR							(APB2PERIPH_BASEADDR + 0x0400) // EXTI peripheral base address on APB2 bus

#define AFIO_BASEADDR							(APB2PERIPH_BASEADDR)		   // AFIO peripheral base address on APB2 bus

#define GPIOA_BASEADDR							(APB2PERIPH_BASEADDR + 0x0800) // GPIOA peripheral base address on APB2 bus
#define GPIOB_BASEADDR							(APB2PERIPH_BASEADDR + 0x0C00) // GPIOB peripheral base address on APB2 bus
#define GPIOC_BASEADDR							(APB2PERIPH_BASEADDR + 0x1000) // GPIOC peripheral base address on APB2 bus
#define GPIOD_BASEADDR							(APB2PERIPH_BASEADDR + 0x1400) // GPIOD peripheral base address on APB2 bus
#define GPIOE_BASEADDR							(APB2PERIPH_BASEADDR + 0x1800) // GPIOE peripheral base address on APB2 bus
#define GPIOF_BASEADDR							(APB2PERIPH_BASEADDR + 0x1C00) // GPIOF peripheral base address on APB2 bus
#define GPIOG_BASEADDR							(APB2PERIPH_BASEADDR + 0x2000) // GPIOG peripheral base address on APB2 bus

#define SPI1_BASEADDR							(APB2PERIPH_BASEADDR + 0x3000) // SPI1 peripheral base address on APB2 bus

#define USART1_BASEADDR							(APB2PERIPH_BASEADDR + 0x3800) // USART1 peripheral base address on APB2 bus

/*
 * Base addresses of peripherals which are hanging on AHB bus
 */
#define RCC_BASEADDR							(AHBPERIPH_BASEADDR + 0x9000) // RCC peripheral base address on AHB bus


/************************Peripheral Register Definition Structures***************************************/

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	volatile uint32_t CR; 				// Clock control register							Address offset:0x00
	volatile uint32_t CFGR;				// Clock configuration register						Address offset:0x04
	volatile uint32_t CIR;				// Clock interrupt register							Address offset:0x08
	volatile uint32_t APB2RSTR;			// APB2 peripheral reset register					Address offset:0x0C
	volatile uint32_t APB1RSTR;			// APB1 peripheral reset register					Address offset:0x10
	volatile uint32_t AHBENR;			// AHB peripheral Clock enable register				Address offset:0x14
	volatile uint32_t APB2ENR;			// APB2 peripheral clock enable register			Address offset:0x18
	volatile uint32_t APB1ENR;			// APB1 peripheral clock enable register			Address offset:0x1C
	volatile uint32_t BDCR;				// Backup domain control register					Address offset:0x20
	volatile uint32_t CSR;				// Control/status register							Address offset:0x24
	volatile uint32_t AHBRSTR;			// AHB peripheral clock reset register				Address offset:0x28
	volatile uint32_t CFGR2;			// Clock configuration register2					Address offset:0x2C

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR; 				// Interrupt mask register							Address offset:0x00
	volatile uint32_t EMR;				// Event mask register								Address offset:0x04
	volatile uint32_t RTSR;				// Rising trigger selection register				Address offset:0x08
	volatile uint32_t FTSR;				// Falling trigger selection register				Address offset:0x0c
	volatile uint32_t SWIER;			// Software interrupt event register				Address offset:0x10
	volatile uint32_t PR;				// Pending register									Address offset:0x14
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile uint32_t CRL;				// Port configuration register low					Address offset:0x00
	volatile uint32_t CRH;				// Port configuration register high					Address offset:0x04
	volatile uint32_t IDR;				// Port input data register							Address offset:0x08
	volatile uint32_t ODR;				// Port output data register						Address offset:0x0c
	volatile uint32_t BSRR;				// Port bit set/reset register						Address offset:0x10
	volatile uint32_t BRR;				// Port bit reset register							Address offset:0x14
	volatile uint32_t LCKR;				// Port configuration lock register					Address offset:0x18
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for AFIO
 */
typedef struct
{
	volatile uint32_t EVCR;				// Event control register 							Address offset:0x00
	volatile uint32_t MAPR;				// AF remap and debug I/O configuration register	Address offset:0x04
	volatile uint32_t EXTICR[4];		// External interrupt configuration registers 		Address offset:0x08-0x14
	volatile uint32_t MAPR2;			// AF remap and debug I/O configuration register2	Address offset:0x18
}AFIO_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;				// SPI control register 1							Address offset:0x00
	volatile uint32_t CR2;				// SPI control register 2							Address offset:0x04
	volatile uint32_t SR;				// SPI status register								Address offset:0x08
	volatile uint32_t DR;				// SPI data register								Address offset:0x0c
	volatile uint32_t CRCPR;			// SPI CRC polynomial register						Address offset:0x10
	volatile uint32_t RXCRCR;			// SPI RX CRC register								Address offset:0x14
	volatile uint32_t TXCRCR;			// SPI TX CRC register								Address offset:0x18
	volatile uint32_t I2SCFGR;			// SPI_I2S configuration register					Address offset:0x1c
	volatile uint32_t I2SPR;			// SPI_I2S prescaler register						Address offset:0x20
}SPI_RegDef_t;


/*
 * Peripheral Definitions
 */
#define GPIOA  ((GPIO_RegDef_t*) GPIOA_BASEADDR)					// GPIOA register structure pointer
#define GPIOB  ((GPIO_RegDef_t*) GPIOB_BASEADDR)					// GPIOB register structure pointer
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASEADDR)					// GPIOC register structure pointer
#define GPIOD  ((GPIO_RegDef_t*) GPIOD_BASEADDR)					// GPIOD register structure pointer
#define GPIOE  ((GPIO_RegDef_t*) GPIOE_BASEADDR)					// GPIOE register structure pointer
#define GPIOF  ((GPIO_RegDef_t*) GPIOF_BASEADDR)					// GPIOF register structure pointer
#define GPIOG  ((GPIO_RegDef_t*) GPIOG_BASEADDR)					// GPIOG register structure pointer

#define RCC  ((RCC_RegDef_t*) RCC_BASEADDR)  						// RCC register structure pointer

#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)  						// EXTI register structure pointer

#define AFIO ((AFIO_RegDef_t*) AFIO_BASEADDR)  						// AFIO register structure pointer

#define SPI1 ((SPI_RegDef_t*) SPI1_BASEADDR)						// SPI1 register structure pointer
#define SPI2 ((SPI_RegDef_t*) SPI2_BASEADDR)						// SPI2 register structure pointer
#define SPI3 ((SPI_RegDef_t*) SPI3_BASEADDR)						// SPI3 register structure pointer


/*
 * Clock Enable Macro for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() 		(RCC->APB2ENR |= (1 << 2) )			// Enable clock for GPIOA
#define GPIOB_PCLK_EN()			(RCC->APB2ENR |= (1 << 3) )			// Enable clock for GPIOB
#define GPIOC_PCLK_EN()			(RCC->APB2ENR |= (1 << 4) )			// Enable clock for GPIOC
#define GPIOD_PCLK_EN()			(RCC->APB2ENR |= (1 << 5) )			// Enable clock for GPIOD
#define GPIOE_PCLK_EN()			(RCC->APB2ENR |= (1 << 6) )			// Enable clock for GPIOE

/*
 * Clock Enable Macro for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21) )		// Enable clock for I2C1
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22) )		// Enable clock for I2C2

/*
 * Clock Enable Macro for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12) )		// Enable clock for SPI1
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14) )		// Enable clock for SPI2
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15) )		// Enable clock for SPI3

/*
 * Clock Enable Macro for UARTx & USARTx peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 14) )		// Enable clock for USART1
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17) )		// Enable clock for USART2
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18) )		// Enable clock for USART3
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19) )		// Enable clock for UART4
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20) )		// Enable clock for UART5

/*
 * Clock Enable Macro for GPIOx peripherals
 */
#define AFIO_PCLK_EN() 		(RCC->APB2ENR |= (1 << 0) )				// Enable clock for AFIO

/*
 * Clock Disable Macro for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 2) )		// Disable clock for GPIOA
#define GPIOB_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 3) )		// Disable clock for GPIOB
#define GPIOC_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4) )		// Disable clock for GPIOC
#define GPIOD_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5) )		// Disable clock for GPIOD
#define GPIOE_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 6) )		// Disable clock for GPIOE

/*
 * Clock Disable Macro for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21) )		// Disable clock for I2C1
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22) )		// Disable clock for I2C2

/*
 * Clock Disable Macro for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12) )		// Disable clock for SPI1
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14) )		// Disable clock for SPI2
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15) )		// Disable clock for SPI3

/*
 * Clock Disable Macro for UARTx & USARTx peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14) )		// Disable clock for USART1
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17) )		// Disable clock for USART2
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18) )		// Disable clock for USART3
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19) )		// Disable clock for UART4
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20) )		// Disable clock for UART5

/*
 * Clock Enable Macro for GPIOx peripherals
 */
#define AFIO_PCLK_DI() 		(RCC->APB2ENR &= ~ (1 << 0) )				// Disable clock for AFIO

/*
 * Macros to reset the GPIOx peripheral
 */
#define GPIOA_REG_RESET();	do{ (RCC->APB2RSTR |= (1 << 0) ); (RCC->APB2RSTR &= ~(1 << 0) );} while(0)
#define GPIOB_REG_RESET();	do{ (RCC->APB2RSTR |= (1 << 1) ); (RCC->APB2RSTR &= ~(1 << 1) );} while(0)
#define GPIOC_REG_RESET();	do{ (RCC->APB2RSTR |= (1 << 2) ); (RCC->APB2RSTR &= ~(1 << 2) );} while(0)
#define GPIOD_REG_RESET();	do{ (RCC->APB2RSTR |= (1 << 3) ); (RCC->APB2RSTR &= ~(1 << 3) );} while(0)
#define GPIOE_REG_RESET();	do{ (RCC->APB2RSTR |= (1 << 4) ); (RCC->APB2RSTR &= ~(1 << 4) );} while(0)

/*
 * Macros to reset the SPIx peripheral
 */
#define SPI1_REG_RESET();	do{ (RCC->APB2RSTR |= (1 << 12) ); (RCC->APB2RSTR &= ~(1 << 12) );} while(0)
#define SPI2_REG_RESET();	do{ (RCC->APB1RSTR |= (1 << 14) ); (RCC->APB1RSTR &= ~(1 << 14) );} while(0)
#define SPI3_REG_RESET();	do{ (RCC->APB1RSTR |= (1 << 15) ); (RCC->APB1RSTR &= ~(1 << 15) );} while(0)



/*
 * IRQ(Interrupts Request) Numbers of STM32F103x
 * NOTE: Update these macros with valid values according to you MCU
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51


/*
 * Generic Macros
 */
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET 				SET
#define GPIO_PIN_RESET 				RESET
#define FLAG_RESET 					RESET
#define FLAG_SET 					SET

/**************************************************
 * Bit Position Definitions for SPI Peripheral
 **************************************************/

/*
 * Bit position definitions for SPI_CR1
 */
#define SPI_CR1_CPHA       0   // Clock Phase
#define SPI_CR1_CPOL       1   // Clock Polarity
#define SPI_CR1_MSTR       2   // Master Selection
#define SPI_CR1_BR         3   // Baud Rate Control (3 bits: 3-5)
#define SPI_CR1_SPE        6   // SPI Enable
#define SPI_CR1_LSBFIRST   7   // Frame Format (LSB/MSB first)
#define SPI_CR1_SSI        8   // Internal Slave Select
#define SPI_CR1_SSM        9   // Software Slave Management
#define SPI_CR1_RXONLY     10  // Receive Only
#define SPI_CR1_DFF        11  // Data Frame Format (8-bit or 16-bit)
#define SPI_CR1_CRCNEXT    12  // CRC Transfer Next
#define SPI_CR1_CRCEN      13  // Hardware CRC Calculation Enable
#define SPI_CR1_BIDIOE     14  // Output Enable in Bidirectional Mode
#define SPI_CR1_BIDIMODE   15  // Bidirectional Data Mode Enable

/*
 * Bit position definitions for SPI_CR2
 */
#define SPI_CR2_RXDMAEN    0   // Rx Buffer DMA Enable
#define SPI_CR2_TXDMAEN    1   // Tx Buffer DMA Enable
#define SPI_CR2_SSOE       2   // SS Output Enable
#define SPI_CR2_ERRIE      5   // Error Interrupt Enable
#define SPI_CR2_RXNEIE     6   // RX buffer Not Empty Interrupt Enable
#define SPI_CR2_TXEIE      7   // Tx buffer Empty Interrupt Enable

/*
 * Bit position definitions for SPI_SR (Status Register)
 */
#define SPI_SR_RXNE        0   // Receive Buffer Not Empty
#define SPI_SR_TXE         1   // Transmit Buffer Empty
#define SPI_SR_CHSIDE      2   // Channel Side
#define SPI_SR_UDR         3   // Underrun Flag
#define SPI_SR_CRCERR      4   // CRC Error Flag
#define SPI_SR_MODF        5   // Mode Fault
#define SPI_SR_OVR         6   // Overrun Flag
#define SPI_SR_BSY         7   // Busy Flag







#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_spi_driver.h"

#endif /* INC_STM32F103XX_H_ */
