/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Apr 19, 2025
 *      Author: ASA
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_


#include "stm32f103xx.h"

/*
 * GPIO_PortToCode
 * Returns the port code for the given GPIOx base address.
 * Example: GPIOA -> 0, GPIOB -> 1, ..., GPIOG -> 6.
 */
uint8_t GPIO_PortToCode(GPIO_RegDef_t *pGPIOx);

/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t PinNumber;     // Pin number (GPIO_PIN_0, GPIO_PIN_1, etc.)    					@GPIO_PIN_NUMBERS
    uint8_t PinSpeed;      // Pin speed (10 MHz, 2 MHz, 50MHz)			   					@GPIO_PIN_SPEED
    uint8_t PinMode;	  // Pin mode (Analog, Push-pull, Open-drain ....)					@GPIO_PIN_MODE
} GPIO_PinConfig_t;


/*
 * Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; 					// Holds the base address of the GPIO port to which the ping belongs
	GPIO_PinConfig_t GPIO_PinConfig;		// Holds the GPIO pin configuration settings

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_1 						1U
#define GPIO_PIN_NO_2 						2U
#define GPIO_PIN_NO_3 						3U
#define GPIO_PIN_NO_4 						4U
#define GPIO_PIN_NO_5 						5U
#define GPIO_PIN_NO_6  						6U
#define GPIO_PIN_NO_7 						7U
#define GPIO_PIN_NO_8 						8U
#define GPIO_PIN_NO_9 						9U
#define GPIO_PIN_NO_10 						10U
#define GPIO_PIN_NO_11 						11U
#define GPIO_PIN_NO_12 						12U
#define GPIO_PIN_NO_13 						13U
#define GPIO_PIN_NO_14 						14U
#define GPIO_PIN_NO_15 						15U

/*
 * @GPIO_PIN_SPEED
 * GPIO speed configuration macros
 */
#define GPIO_SPEED_INPUT              		00U  // Input mode (reset state)
#define GPIO_SPEED_OUTPUT_10MHZ      	  	01U  // Output mode, max speed 10 MHz
#define GPIO_SPEED_OUTPUT_2MHZ       	    02U  // Output mode, max speed 2 MHz
#define GPIO_SPEED_OUTPUT_50MHZ          	03U  // Output mode, max speed 50 MHz

/*
 * @GPIO_PIN_MODE
 * GPIO MODE Configuration macros
 */

// Input configuration
#define GPIO_MODE_ANALOG              	  	00U  // Analog mode
#define GPIO_MODE_FLOATING_INPUT      	  	01U  // Floating input
#define GPIO_MODE_INPUT_PU_PD         		02U  // Input with pull-up / pull-down

// Output configuration
#define GPIO_MODE_GP_PUSH_PULL        		00U  // General purpose output push-pull
#define GPIO_MODE_GP_OPEN_DRAIN       		01U  // General purpose output open-drain
#define GPIO_MODE_AF_PUSH_PULL        		02U  // Alternate function output push-pull
#define GPIO_MODE_AF_OPEN_DRAIN       		03U  // Alternate function output open-drain

// EXTI configuration
#define GPIO_MODE_IT_RISING					04U  // Interrupt on rising edge
#define GPIO_MODE_IT_FALLING				05U  // Interrupt on falling edge
#define GPIO_MODE_IT_RISING_FALLING   		06U  // Interrupt on rising and falling edge

#define GPIO_MODE_EVT_RISING				07U  // Event on rising and rising edge
#define GPIO_MODE_EVT_FALLING				08U  // Event on rising and falling edge
#define GPIO_MODE_EVT_RISING_FALLING		09U  //Event on rising and rising and falling edge


/*************************************************************************************
 *  						APIs supported by this driver
 *  		For more information about the APIs check the function definitions
 *************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint8_t enorDi);


/*
 *  Init and De-Init
 */
void GPIO_Init (GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx);


/*
 *  Read and Write
 */
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 *  IRQ Configuration and IRQ Handling
 */
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling (uint8_t PinNumber);










#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
