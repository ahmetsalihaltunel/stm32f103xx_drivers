/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Apr 19, 2025
 *      Author: ASA
 */


#include "stm32f103xx_gpio_driver.h"


/**
  * @brief  Returns the port code for the given GPIO base address.
  * @param  pGPIOx: base address of the GPIO peripheral
  * @retval uint8_t: Port code
  * @note   This function is typically used in configurations involving
  *         external interrupts (EXTI), where a numerical code corresponding
  *         to a specific GPIO port is required.
  */
uint8_t GPIO_PortToCode(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) return 0;
    else if (pGPIOx == GPIOB) return 1;
    else if (pGPIOx == GPIOC) return 2;
    else if (pGPIOx == GPIOD) return 3;
    else if (pGPIOx == GPIOE) return 4;
    else if (pGPIOx == GPIOF) return 5;
    else if (pGPIOx == GPIOG) return 6;
    else return 0; // Default value
}

/**
  * @brief  Enables or disables peripheral clock for the given GPIO port
  * @param  pGPIOx: base address of the GPIO peripheral
  * @param  EnorDi: ENABLE or DISABLE macros
  * @retval None
  */
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
	}

}


/**
* @brief  Initialize the given GPIO pin and its port based on handle configuration
* @param  pGPIOHandle: Pointer to GPIO handle with port base address and pin settings
* @retval None
*/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	if(pGPIOHandle->GPIO_PinConfig.PinMode <= GPIO_MODE_AF_OPEN_DRAIN)
	{
		uint32_t ConfigValue=0;          	// Variable to hold the configuration value
		uint32_t position=0;				// Variable to determine the position of the pin in CRL/CRH

		// Combine Pin configuration (CNF) and speed
		ConfigValue = ((pGPIOHandle->GPIO_PinConfig.PinMode << 2) | (pGPIOHandle->GPIO_PinConfig.PinSpeed));

		// Calculate the position
		position = (pGPIOHandle->GPIO_PinConfig.PinNumber % 8 ) * 4;

		// Check if the pin number is less than 8 (CRL register)
		if(pGPIOHandle->GPIO_PinConfig.PinNumber < 8)
		{
			pGPIOHandle->pGPIOx->CRL &= ~(0xF << position);				// Clear the corresponding position in CRL register
			pGPIOHandle->pGPIOx->CRL |= ( ConfigValue << position ) ;	// Set the corresponding position in CRL register
		}
		else
		{
			// If the pin number is 8 or more (CRH register)

			pGPIOHandle->pGPIOx->CRH &= ~(0xF << position);				// Clear the corresponding position in CRH register
			pGPIOHandle->pGPIOx->CRH |= ( ConfigValue << position );	// Set the corresponding position in CRH register
		}
		ConfigValue=0;													// Reset ConfigValue
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FALLING)
		{
			// Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RISING)
		{
			// Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RISING_FALLING)
		{
			// Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}

		// Configure the GPIO port selection in AFIO_EXTICR

		// Calculate the index number of the EXTICR	register based on the pin number
		uint8_t exticr_index = pGPIOHandle->GPIO_PinConfig.PinNumber / 4;

		// Calculate the position within the EXTICR register for the specific pin
		uint8_t exticr_position = (pGPIOHandle->GPIO_PinConfig.PinNumber % 4) * 4;

		// Get the port code corresponding to the GPIO port
		uint8_t portcode = GPIO_PortToCode(pGPIOHandle->pGPIOx);

		AFIO_PCLK_EN(); // Enable the clock
		AFIO->EXTICR[exticr_index] &= ~(0xF << exticr_position);        // Clear the bits in EXTI register
		AFIO->EXTICR[exticr_index] |= portcode << (exticr_position);	// Set the bits in EXTI register

		// Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
	}

}

/**
* @brief  Reset the specified GPIO peripheral registers to their default reset state.
* @param  pGPIOx: Base address of the GPIO peripheral to reset.
* @retval None
*/

void GPIO_DeInit (GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
}


/**
  * @brief  Reads the input value from the given GPIO pin.
  * @param  pGPIOx: Base address of the GPIO peripheral.
  * @param  PinNumber: Pin number (0 to 15).
  * @retval 0 or 1 based on pin state.
  */
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x1);			// Return the state of the specified pin
}

/**
  * @brief  Reads the entire 16-bit input port.
  * @param  pGPIOx: Base address of the GPIO peripheral.
  * @retval 16-bit value representing the state of all pins in the port.
  */
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t) (pGPIOx->IDR);								// Return the entire Input Data Register
}

/**
  * @brief  Writes a value (0 or 1) to the given GPIO output pin.
  * @param  pGPIOx: Base address of the GPIO peripheral.
  * @param  PinNumber: Pin number (0 to 15).
  * @param  Value: Value to write (GPIO_PIN_RESET= 0, GPIO_PIN_SET = 1).
  * @retval None
  */
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber );						// Set the bit to 1
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);						// Set the bit to 0
	}
}

/**
  * @brief  Writes a 16-bit value to the entire GPIO output port.
  * @param  pGPIOx: Base address of the GPIO peripheral.
  * @param  Value: 16-bit value to write to the port (each bit controls one pin).
  * @retval None
  */
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value; 										// Set the port to the given Value
}

/**
  * @brief  Toggles the state of a given GPIO pin (changes it from high to low or low to high)
  * @param  pGPIOx: Base address of the GPIO peripheral.
  * @param  PinNumber: Pin number (0 to 15).
  * @retval None
  */
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber); 				// Toggle the pin (flip the current state)
}


/**
  * @brief  Configures (enables or disables) the interrupt for a given IRQ number in the NVIC.
  * @param  IRQNumber: The IRQ (Interrupt Request) number to configure.
  * 		Valid values range from 0 to 95 depending on the microcontroller.
  * @param 	EnorDi: ENABLE or DISABLE macros
  * @retval None
  */
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			// Program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64 ) // Between 32 and 63
		{
			// Program ISER1 Register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}else if (IRQNumber >= 64 && IRQNumber < 96 ) // Between 64 and 95
		{
			// Program ISER2 Register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <=31)
		{
			// Program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64 ) // Between 32 and 63
		{
			// Program ICER1 Register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if (IRQNumber >= 64 && IRQNumber < 96 ) // Between 64 and 95
		{
			// Program ICER2 Register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}

	}

}

/**
  * @brief  Configures the priority of a given IRQ number in the NVIC (Nested Vectored Interrupt Controller).
  * @param  IRQNumber: The IRQ (Interrupt Request) number whose priority is to be configured.
  * @param  IRQPriority: Priority level to be assigned to the IRQ.
  *         Lower numerical values represent higher priority.
  *         Only the upper 4 bits of the 8-bit field are implemented in most Cortex-M cores.
  * @retval None
  */
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Calculate the index of the IRQ register for the given IRQ number
	uint8_t ipr_index = (IRQNumber / 4) ;

	// Calculate the section within the IPR register for the given IRQ number
	uint8_t ipr_section = (IRQNumber % 4) * 8;

	*(NVIC_PR_BASE_ADDR + ipr_index) &= ~(0xFF << (ipr_section)); 					// Clear the existing priority bit
	*(NVIC_PR_BASE_ADDR + ipr_index) |= (IRQPriority << (ipr_section + 4));			// Set the new priority value

}

/**
  * @brief  Handles the interrupt by clearing the pending bit in EXTI PR (Pending Register)
  *         for the specified GPIO pin number.
  * @param  PinNumber: The GPIO pin number (0 to 15) associated with the EXTI line to be cleared.
  * @retval None
  *
  * This function should be called inside the EXTI interrupt handler.
  * It checks if the interrupt pending bit is set for the given pin and clears it
  * by writing a '1' to the appropriate bit in the EXTI->PR register.
  */
void GPIO_IRQHandling (uint8_t PinNumber)
{
	// Clear the EXTI  PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}

}

