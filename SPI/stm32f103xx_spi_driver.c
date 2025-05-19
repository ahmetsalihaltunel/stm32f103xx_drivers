/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: May 6, 2025
 *      Author: ASA
 */


#include "stm32f103xx_SPI_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/**
  * @brief	Enables or disables the peripheral clock for the given SPI port.
  * @param	pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void SPI_PeriClockControl (SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}

	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}

}

/**
  * @brief	Initializes the SPI peripheral with the specified configuration.
  * @param	pSPIHandle: Pointer to the SPI handle structure.
  * @retval None
  */
void SPI_Init (SPI_Handle_t *pSPIHandle)
{
	// Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t RegValue = 0;

	// 1. Configure the device mode (Master/Slave)
	RegValue |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2. Configure the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Clear BIDIMODE for full-duplex
		RegValue &= ~(1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Set BIDIMODE for half-duplex
		RegValue |= (1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		// Clear BIDIMODE and set RXONLY
		RegValue &= ~(1 << SPI_CR1_BIDIMODE);
		RegValue |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI Serial Clock Speed (Baud rate)
	RegValue |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. Configure the DFF (8-bit or 16-bit)
	RegValue |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 5.Configure the CPOL
	RegValue |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 6.Configure the CPHA
	RegValue |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 8. Configure SSM (Software slave management)
	RegValue |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	// 9. Write to CR1
	pSPIHandle->pSPIx->CR1 = RegValue;

}

/**
  * @brief	Resets the specified SPI peripheral registers.
  * @param	pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @retval None
  */
void SPI_DeInit (SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/**
  * @brief  Checks the status of a specific flag in the SPI status register.
  * @param  pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @param  FlagName: Name of the flag to check. Possible values are:
  *         - SPI_SR_TXE  : Transmit buffer empty flag
  *         - SPI_SR_RXNE : Receive buffer not empty flag
  *         - SPI_SR_BUSY : Busy flag
  *         - SPI_SR_OVR  : Overrun flag
  * @retval uint8_t: FLAG_SET if the flag is set, FLAG_RESET otherwise.
  * @note   This function helps in polling-based SPI communication.
  */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**
  * @brief	Transmits data using the SPI peripheral in blocking mode.
  * @param	pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @param	pTxBuffer: Pointer to the data buffer to be transmitted.
  * @param	Len: Length of data to transmit in bytes.
  * @retval None
  */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		// Check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

/**
  * @brief  Receives data over SPI in blo	cking mode (polling-based).
  * @param  pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @param  pRxBuffer: Pointer to the buffer where received data will be stored.
  * @param  Len: Length of data to receive in bytes.
  * @retval None
  * @note   This function blocks the CPU until all data is received. Use with caution in time-critical applications.
  */
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// Check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// Load the data from DR to RxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}

}

/**
  * @brief	Enables or disables the SPI peripheral.
  * @param	pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// Set the SPE (SPI Enable) bit in CR1 to enable the SPI peripheral
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		// Clear the SPE bit to disable the SPI peripheral
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/**
  * @brief  Enables or disables IRQ for a given interrupt number.
  * @param  IRQNumber: IRQ (Interrupt Request) number to configure.
  * @param  EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
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
  * @brief  Configures the priority of a given IRQ number in the NVIC.
  * @param  IRQNumber: IRQ (Interrupt Request) number to configure.
  * @param  IRQPriority: Priority level to assign to the IRQ (0 = highest, 15 = lowest for 4-bit priority fields).
  * @retval None
  */
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Calculate the index of the IRQ register for the given IRQ number
	uint8_t ipr_index = (IRQNumber / 4) ;

	// Calculate the section within the IPR register for the given IRQ number
	uint8_t ipr_section = (IRQNumber % 4) * 8;

	*(NVIC_PR_BASE_ADDR + ipr_index) &= ~(0xFF << (ipr_section)); 					// Clear the existing priority bit
	*(NVIC_PR_BASE_ADDR + ipr_index) |= (IRQPriority << (ipr_section + 4));			// Set the new priority value
}


/**
  * @brief  Sends data over SPI using interrupt mode (non-blocking).
  * @note   This function only sets up the transmission. The actual data transfer
  *         is handled in the interrupt service routine.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @param  pTxBuffer: Pointer to the data buffer to be transmitted.
  * @param  Len: Length of data to transmit in bytes.
  * @retval uint8_t: SPI transmission state before starting:
  *         - SPI_READY (0): Transmission started successfully.
  *         - SPI_BUSY_IN_TX (1): Transmission is already ongoing.
  */
uint8_t	 SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	 // Proceed only if SPI is not already transmitting
	if(state != SPI_BUSY_IN_TX)
	{
        // Save the Tx buffer pointer and length information in the SPI handle
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

        // Mark the SPI state as busy in transmission
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable the TXEIE (TX buffer empty interrupt) to trigger an interrupt when TXE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

    // Return the previous state to inform the caller whether the function started a new transmission
	return state;
}

/**
  * @brief  Receives data over SPI using interrupt mode (non-blocking).
  * @note   This function only sets up the reception. The actual data transfer
  *         is handled in the interrupt service routine.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @param  pRxBuffer: Pointer to the data buffer where received data will be stored.
  * @param  Len: Length of data to receive in bytes.
  * @retval uint8_t: SPI reception state before starting:
  *         - SPI_READY (0): Reception started successfully.
  *         - SPI_BUSY_IN_RX (1): Reception is already ongoing.
  */
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

    // Proceed only if SPI is not already receiving
	if(state != SPI_BUSY_IN_RX)
	{
		// Save the Rx buffer pointer and length information in the SPI handle
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// Mark the SPI state as busy in reception
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable the RXNEIE (Receive buffer not empty interrupt) to trigger an interrupt when RXNE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	// Return the previous state to inform the caller whether the function started a new reception
	return state;
}


/**
  * @brief  Handles SPI interrupt events (TXE, RXNE, and OVR).
  * @note   This function should be called from the SPI interrupt handler (e.g., SPI1_IRQHandler).
  *         It checks which interrupt source triggered the SPI interrupt and calls the appropriate
  *         handler function for:
  *         - Transmit buffer empty (TXE)
  *         - Receive buffer not empty (RXNE)
  *         - Overrun error (OVR)
  * @param  pHandle: Pointer to the SPI handle structure containing SPI configuration and buffers.
  * @retval None
  */
void SPI_IRQHandling (SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;

	// Check and handle TXE interrupt
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);					// Is TXE flag set?
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);				// Is TXEIE interrupt enabled?

	if(temp1 && temp2)
	{
		// Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

    // Check and handle RXNE interrupt
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);				// Is RXNE flag set?
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);			// Is RXNEIE interrupt enabled?

	if(temp1 && temp2)
	{
		// Handle RXNE (Receive buffer not empty)
		spi_rxne_interrupt_handle(pHandle);
	}

	// Check and handle OVR (Overrun error) interrupt
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);					// Is OVR flag set?
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);				// Is error interrupt enabled?

	if(temp1 && temp2)
	{
		// Handle Overrun Error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


/************************* Some helper function implementations ************************************************/


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/**
  * @brief  Handles the TXE (Transmit buffer empty) interrupt for SPI.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @retval None
  * @note   This function is called when TXE flag is set and TXEIE interrupt is enabled.
  *         It sends one unit (byte or word) of data and updates the Tx buffer.
  */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		// 16 bit DFF
		// Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	// Transmission complete
	if(pSPIHandle->TxLen == 0)
	{
		// TxLen is zero, so close the spi transmission and inform the application that TX is over

		// This prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

/**
  * @brief  Handles the RXNE (Receive buffer not empty) interrupt for SPI.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @retval None
  * @note   This function is called when RXNE flag is set and RXNEIE interrupt is enabled.
  *         It reads one unit (byte or word) of data from the SPI peripheral and stores it in Rx buffer.
  */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		// 16 bit DFF
		// Load the data from DR to RxBuffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;

	}else
	{
		// 8 bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	// Reception complete
	if(pSPIHandle->RxLen == 0)
	{
		// Reception is complete

		// Turn off the rxneie interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

/**
  * @brief  Handles the OVR (Overrun Error) interrupt for SPI.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @retval None
  * @note   This function clears the OVR flag and notifies the application.
  *         OVR should be cleared only when not transmitting.
  */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// Clear OVR flag only if not transmitting
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;												// Prevent unused variable warning

	// Inform the application
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);
}

/**
  * @brief  Closes the SPI transmission by disabling TXEIE and resetting state.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @retval None
  */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);		// Disable TXE interrupt
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/**
  * @brief  Closes the SPI reception by disabling RXNEIE and resetting state.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @retval None
  */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);		// Disable RXNE interrupt
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

/**
  * @brief  Clears the OVR (Overrun Error) flag in the SPI status register.
  * @param  pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3).
  * @retval None
  * @note   OVR flag is cleared by reading DR and then SR.
  */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;												// Prevent unused variable warning
}


/**
  * @brief  Weak callback function to notify the application about SPI events.
  * @param  pSPIHandle: Pointer to the SPI handle structure.
  * @param  AppEv: Event type. Possible values:
  *         - SPI_EVENT_TX_CMPLT
  *         - SPI_EVENT_RX_CMPLT
  *         - SPI_EVENT_OVR_ERR
  * @retval None
  * @note   The application can override this function to handle events.
  */
__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	// This is a weak implementation. The application may override this function
}
