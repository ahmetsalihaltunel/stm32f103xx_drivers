/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Jun 22, 2025
 *      Author: ASA
 */


#include "stm32f103xx_usart_driver.h"

/**
  * @brief	Configures the baud rate for the specified USART peripheral.
  * @param	pUSARTx: Base address of the USART peripheral (USART1, USART2...).
  * @param	BaudRate: Desired baud rate (e.g., 9600, 115200).
  * @retval None
  */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLK, usartdiv, Mantissa, Fraction, RegValue;

	if(pUSARTx == USART1)
	{
		// Usart1 hanging on APB2 bus
		PCLK = RCC_GetPCLK2Value();
	}
	else
	{
		PCLK = RCC_GetPCLK1Value();
	}

    // USARTDIV = PCLK / (16 * BaudRate)
	usartdiv = (25 * PCLK) / (4 * BaudRate);

	// Extract mantissa (integer part)
	Mantissa = usartdiv / 100;

	RegValue = 0;

	// Place the Mantissa part in appropriate bit position
	RegValue |= Mantissa << 4;

	// Extract fractional part and scale it to 4 bits (0–15) using rounding
	Fraction = usartdiv - (100 * Mantissa);
	Fraction = ((( Fraction * 16)+ 50) / 100) & (0x0F);

	// Place the fractional part in appropriate bit position
	RegValue |= Fraction;

	// Program the BRR register
	pUSARTx->BRR = RegValue;
}


/**
  * @brief	Enables or disables the peripheral clock for the given USART port.
  * @param	pUSARTx: Base address of the USART peripheral (USART1, USART2...).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}

	}else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
	}

}

/**
  * @brief	Enables or disables the USART peripheral.
  * @param	pUSARTx: Base address of the USART peripheral (USART1, USART2...).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// Set the UE (USART Enable) bit in CR1 to enable the USART peripheral
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		// Clear the UE bit to disable the USART peripheral
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/**
  * @brief	Initializes the USART peripheral with the specified configuration.
  * @param	pUSARTHandle: Pointer to the USART handle structure.
  * @retval None
  */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t RegValue = 0;

	// Enable peripheral clock for USART
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// 2. Disable USART before configuration
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_UE);

	/*********************************** Configuration CR1 ******************************************/

	// Configure the mode
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		RegValue |= (1 << USART_CR1_TE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		RegValue |= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
		{
		RegValue |= ( (1 << USART_CR1_TE) | (1 << USART_CR1_RE) );
		}

	// Configure the word length
	RegValue |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	// Configure the parity control

	if(pUSARTHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE)
	{
		RegValue |= (1 << USART_CR1_PCE);			// Enable parity control

		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EVEN)
		{
			RegValue &= ~(1 << USART_CR1_PS);		// Even parity
		}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_ODD)
		{
			RegValue |= (1 << USART_CR1_PS);		// Odd parity
		}
	}


	// Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = RegValue;


	/*********************************** Configuration CR2 ******************************************/

	RegValue = 0;

	// Configure the number of stops
	RegValue |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = RegValue;

	/*********************************** Configuration CR3 ******************************************/

	RegValue = 0;

	// Configure the hardware flow control

	if(pUSARTHandle->USART_Config.USART_HWFlowControl != USART_HW_FLOW_CTRL_NONE)
	{
		if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			RegValue |= (1 << USART_CR3_CTSE);
		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			RegValue |= (1 << USART_CR3_RTSE);
		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTSRTS)
		{
			RegValue |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) );
		}

	}

	//Program the CR3 register
	pUSARTHandle->pUSARTx->CR3 = RegValue;

	/*********************************** Configuration of Baudrate (BRR) ******************************************/

	// Configure the baudrate
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_UE);

}

/**
  * @brief	Resets the specified USART peripheral registers.
  * @param	pUSARTx: Base address of the USART peripheral (USART1, USART2...).
  * @retval	None
  */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}

}

/**
  * @brief	Checks the status of a specified USART flag.
  * @param	pUSARTx: Pointer to the USART peripheral base address (e.g., USART1, USART2).
  * @param  FlagName: The flag to check in the USART status register (SR).
  *         This parameter can be one of the USART_FLAG_xxx macros (e.g., USART_FLAG_TXE, USART_FLAG_RXNE).
  * @retval uint8_t: Returns FLAG_SET (1) if the specified flag is set, otherwise returns FLAG_RESET (0).
  */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/**
  * @brief	Transmits data over USART in blocking mode (polling-based).
  * @Note	This function sends data byte-by-byte or word-by-word depending on the
  *         word length and parity configuration. It waits for TXE before each byte
  *         and optionally waits for TC (transmission complete).
  * @param	pUSARTHandle: Pointer to the USART handle structure.
  * @param  pTxBuffer: Pointer to the data buffer to be transmitted.
  * @param  Len: Length of data to transmit.
  * @retval None
  *
  * @retval None
  */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;	// Pointer for 16-bit access when sending 9-bit data

	while(Len > 0)
	{
		// Wait for TXE is set
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) );

        // Check if configured word length is 9 bits
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
            // Cast the byte pointer to 16-bit pointer to send 9 bits
			pdata = (uint16_t*) pTxBuffer;

			// Load the 9-bit data
			pUSARTHandle->pUSARTx->DR = (*pdata & 0x01FF);

			// Check for parity configuration
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used in this transfer , so 9bits of user data will be sent
				// Increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
				Len--;
			}
			else
			{
				// Parity bit is used in this transfer so 8bits of user data will be sent
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
            // 8-bit word length: mask the buffer to 8 bits and load to DR
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & 0xFF);
			pTxBuffer++;
		}

		// Wait for TC (Transmission Complete) to ensure full transfer
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC) );

        // Decrease frame count after each character sent
		Len--;
	}
}

/**
  * @brief  Receives data over USART in blocking mode.
  * @param  pUSARTHandle: Pointer to the USART handle structure.
  * @param  pRxBuffer: Pointer to the data buffer to be received.
  * @param  Len: Number of bytes to receive.
  * @retval None
  */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			// Wait until RXNE is set
			while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) );

	        // Check if configured word length is 9 bits
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					// Read only first 9 bits so mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & 0x01FF);

					// Increment the pRxBuffer two times
					pRxBuffer++;
					pRxBuffer++;
				}
				else
				{
					//Parity is used, so 8bits will be of user data and 1 bit is parity
					*pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
					pRxBuffer++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					// No parity is used , so all 8bits will be of user data

					// Read 8 bits from DR
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & 0xFF);
				}
				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & 0x7F);
				}

				// Increment the pRxBuffer
				pRxBuffer++;
			}
		}
}

/**
  * @brief  Initiates transmission of data over USART using interrupt mode.
  * @param  pUSARTHandle: Pointer to the USART handle structure.
  * @param  pTxBuffer: Pointer to the data buffer to be transmitted.
  * @param  Len: Number of bytes to receive.
  * @retval retval uint8_t: Returns previous transmission state.
  *         USART_BUSY_IN_TX if a transmission is already ongoing,
  *         otherwise USART_READY (or equivalent) indicating transmission started.
  */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}

	return txstate;
}

/**
  * @brief  Initiates reception of data over USART using interrupt mode.
  * @param  pUSARTHandle: Pointer to the USART handle structure.
  * @param  pRxBuffer: Pointer to the data buffer to be received.
  * @param  Len: Number of bytes to receive.
  * @retval uint8_t: Returns previous reception state.
  *         USART_BUSY_IN_RX if a reception is already ongoing,
  *         otherwise USART_READY (or equivalent) indicating reception started.
  */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		// Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

/**
  * @brief  Enables or disables IRQ for a given interrupt number.
  * @param  IRQNumber: IRQ (Interrupt Request) number to configure.
  * @param  EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void USART_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Calculate the index of the IRQ register for the given IRQ number
	uint8_t ipr_index = (IRQNumber / 4) ;

	// Calculate the section within the IPR register for the given IRQ number
	uint8_t ipr_section = (IRQNumber % 4) * 8;

	*(NVIC_PR_BASE_ADDR + ipr_index) &= ~(0xFF << (ipr_section)); 					// Clear the existing priority bit
	*(NVIC_PR_BASE_ADDR + ipr_index) |= (IRQPriority << (ipr_section + 4));			// Set the new priority value
}

/**
  * @brief  Handles USART interrupt requests and manages transmit, receive,
  *         error, and status events based on USART flags and interrupts.
  * @param  pUSARTHandle: Pointer to the USART handle structure.
  * @retval None
  */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1,temp2,temp3;
	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    // Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 // Check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{

		// Close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				// Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				// Clear the TCIE control bit

				// Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				// Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				// Reset the length to zero
				pUSARTHandle->TxLen = 0;

				// Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	// Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	// Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				// Check the USART_WordLength
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// If 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						// Parity bit is used in this transfer . So 8bits of user data will be sent
						// The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					// This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					// Increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				// TxLen is zero
				// Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2 )
	{
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				// Check the USART_WordLength
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used , so all 9bits will be of user data

						// Read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						// Increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						// Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					// Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used , so all 8bits will be of user data

						// Read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}
					else
					{
						// Parity is used, so , 7 bits will be of user data and 1 bit is parity

						// Read only 7 bits, mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}

					// Increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}
			}

			if(! pUSARTHandle->RxLen)
			{
				// Disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/

//Note : CTS feature is not applicable for UART4 and UART5

	// Check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	// Check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	// Check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1 && temp2 && temp3)
	{
		// Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	// Check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	// Check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		// Clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	// Check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	// Check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1 && temp2)
	{
		// Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

// Noise Flag, Overrun error and Framing Error in multibuffer communication
// We dont discuss multibuffer communication in this course. please refer to the RM
// The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & (1 << USART_SR_NE))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}

/**
  * @brief  Weak callback function to notify the application about USART events.
  * @param  pUSARTHandle: Pointer to the USART handle structure.
  * @param  AppEv: Event type
  * @retval None
  * @note   The application can override this function to handle events.
  */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{
	// This is a weak implementation. The application may override this function
}







