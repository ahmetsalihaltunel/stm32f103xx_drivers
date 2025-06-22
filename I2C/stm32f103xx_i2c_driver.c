/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: May 23, 2025
 *      Author: ASA
 */


#include "stm32f103xx_I2C_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

/**
  * @brief	Enables or disables the peripheral clock for the given I2C port.
  * @param	pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void I2C_PeriClockControl (I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}

	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
	}

}

/**
  * @brief	Enables or disables the I2C peripheral.
  * @param	pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// Set the PE (I2C Enable) bit in CR1 to enable the I2C peripheral
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		// Clear the PE bit to disable the I2C peripheral
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/**
  * @brief	Initializes the I2C peripheral with the specified configuration.
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @retval None
  */
void I2C_Init (I2C_Handle_t *pI2CHandle)
{
	uint32_t RegValue = 0;
	uint8_t trise;

	// Enable peripheral clock for I2C
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Disable the I2C peripheral before configuration
	I2C_PeripheralControl(pI2CHandle->pI2Cx, DISABLE);

	// Configure the ACK control bit
	RegValue = pI2CHandle->pI2Cx->CR1; // Read to avoid clearing other bits
	RegValue |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = RegValue;

	// Configure FREQ field in CR2 register
	RegValue = 0;
	RegValue |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (RegValue & 0x3F);

	// Configure own device address in OAR1
	// Bit 14 (should be 1 as the reference manual)
	RegValue = 0;
	RegValue |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	RegValue |= (1 << 14);

	pI2CHandle->pI2Cx->OAR1 = RegValue;

	// CCR calculations

	uint16_t ccr_value = 0;
	RegValue = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standart mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		RegValue |= (ccr_value & 0xFFF);

	}else
	{
		// Mode is fast mode
		RegValue |= (1 << 15);
		RegValue |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		RegValue |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = RegValue;

	// TRISE configurations
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standart mode
		trise = (RCC_GetPCLK1Value() / 1000000U) + 1;

	}else
	{
		// Mode is fast mode
		trise = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);

}

/**
  * @brief	Resets the specified I2C peripheral registers.
  * @param	pI2Cx: Base address of the I2C peripheral (I2C1 OR I2C2).
  * @retval	None
  */
void I2C_DeInit (I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
}

/**
  * @brief	Checks the status of a specific flag in the I2C status register.
  * @param  pSPIx: Base address of the SPI peripheral (I2C1, I2C2, or I2C3).
  * @param  FlagName: Name of the flag to check. Possible values are:
  *         - I2C_SR_TXE  : Transmit buffer empty flag
  *         - I2C_SR_RXNE : Receive buffer not empty flag
  *         - I2C_SR_BUSY : Busy flag
  *         - I2C_SR_OVR  : Overrun flag
  *         - ....
  * @retval uint8_t: FLAG_SET if the flag is set, FLAG_RESET otherwise.
  * @note   This function helps in polling-based I2C communication.
  */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**
  * @brief	Sends data from the master to a slave device over I2C.
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @param	pTxBuffer   Pointer to the data buffer to be transmitted.
  * @param	Len         Number of bytes to transmit.
  * @param	SlaveAddr   7-bit address of the target slave device.
  * @param	Sr          Repeated start option:
  *                     - I2C_DISABLE_SR: Generate STOP condition after transmission.
  *                     - I2C_ENABLE_SR : Do not generate STOP (for repeated start).
  * @retval	None
  */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Confirm that start generation is completed by checking SB flag
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// Confirm that address  phase is completed by checking the ADDR flag
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// Clear the ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	// Send data until Len become 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) ); // Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// When Len become 0 wait for TXE and BTF flags
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

	// Generate the stop condition
	// Note: Generating stop will automatically clears the BTF bit
	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/**
  * @brief	Receives data from a slave device in I2C master mode.
  *
  * This function generates a START condition, sends the slave address with the read bit,
  * and reads the specified number of bytes from the slave device. It manages the ACK control
  * and optionally generates a STOP condition depending on the repeated start setting.
  *
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @param	pRxBuffer : Pointer to the buffer where received data will be stored.
  * @param	Len       : Number of bytes to receive from the slave.
  * @param	SlaveAddr : 7-bit address of the target slave device.
  * @param	Sr        : Repeated start control.
  *                     - I2C_DISABLE_SR: Generates STOP condition after reception.
  *                     - I2C_ENABLE_SR : Does not generate STOP condition (for repeated start).
  *
  * @retval	None
  */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Confirm that start generation is completed by checking SB flag
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	// Confirm that address  phase is completed by checking the ADDR flag
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// Procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// Disable ACK
		I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait till RXNE set
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

		// Generate the stop condition
		if(Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Read the data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	// Procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			// Wait till RXNE set
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			if(Len == 2) //
			{
				// Disable ACK
				I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate the stop condition
				if(Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

			}

			// Read the data into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// Increment the buffer address
			pRxBuffer++;
		}
	}

	// Enable the ACK if it was initially configured as enabled
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/**
 * @brief  Sends data to a slave device in I2C master mode using interrupts.
 *
 * This function initiates an interrupt-based I2C transmission by generating
 * a START condition, sending the slave address with the write bit, and enabling
 * the required interrupt control bits (ITBUFEN, ITEVTEN, ITERREN). It sets up the
 * transmission state and buffer pointers. The actual data transmission is handled
 * in the ISR.
 *
 * @param  pI2CHandle  Pointer to the I2C handle structure.
 * @param  pTxBuffer   Pointer to the data buffer to be transmitted.
 * @param  Len         Number of bytes to transmit.
 * @param  SlaveAddr   7-bit address of the target slave device.
 * @param  Sr          Repeated start control.
 *                     - I2C_DISABLE_SR: Generate STOP condition after transmission.
 *                     - I2C_ENABLE_SR : Do not generate STOP condition (repeated start).
 *
 * @retval uint8_t     Bus state before initiating transmission:
 *                     - I2C_READY: Transmission successfully started.
 *                     - I2C_BUSY_IN_TX or I2C_BUSY_IN_RX: Bus is currently busy.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/**
  * @brief	Initiates reception of data from a slave device in interrupt mode (non-blocking).
  *
  * This function prepares the I2C peripheral to receive data via interrupts. It sets up internal
  * states and configurations, generates a START condition, and enables necessary interrupt control bits.
  * The actual data reception is handled in the ISR.
  *
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @param	pRxBuffer : Pointer to the buffer where received data will be stored.
  * @param	Len       : Number of bytes to receive from the slave.
  * @param	SlaveAddr : 7-bit address of the target slave device.
  * @param	Sr        : Repeated start control.
  *                     - I2C_DISABLE_SR: Generates STOP condition after reception.
  *                     - I2C_ENABLE_SR : Does not generate STOP condition (for repeated start).
  *
  * @retval	uint8_t   : Returns the current state of the I2C bus.
  *                     - I2C_BUSY_IN_RX: If reception was successfully initiated.
  *                     - I2C_BUSY_IN_TX: If a transmission is currently ongoing.
  *                     - I2C_READY    	: If the bus was idle.
  */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/**
  * @brief	Handles the I2C TXE (Transmit Data Register Empty) interrupt event for master mode.
  *         This function sends the next byte of data if there is data remaining in the buffer.
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @retval	None
  */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		// Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		// Decrement the TxLen
		pI2CHandle->TxLen--;

		// Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

/**
  * @brief	Handles the RXNE (Receive Buffer Not Empty) interrupt event for Master mode.
  *         This function manages data reception when the RXNE flag is set.
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @retval	None
  */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			// Clear ACK bit
			I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		// Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		// Close the data reception and notify the application

		// Generate stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Close the rx
		I2C_CloseReceiveData(pI2CHandle);

		// Notify the application that rx is completed
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}


/**
  * @brief	Handles I2C event interrupt requests.
  *         This function manages the I2C peripheral events such as
  *         Start bit detection, Address phase, Byte Transfer Finished,
  *         Stop detection, TXE (Transmit buffer empty), and RXNE (Receive buffer not empty).
  *         It should be called from the I2C event IRQ handler.
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @retval	None
  */
void I2C_EV_IRQHandling (I2C_Handle_t *pI2CHandle)
{
	uint8_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// Check and handle SB event
	// Note: SB flag event only in master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if (temp1 && temp3)
	{
		// SB flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	// Check and handle ADDR event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if (temp1 && temp3)
	{
		// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// Check and handle BTF event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3)
	{
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				// BTF and TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//s
					// Generate the stop condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// Reset all the elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					// Notify the application about transmission is completed
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}
		}
	}

	// Check and handle STOPF event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

	if(temp1 && temp3)
	{
		// STOPF flag is set
		// Clear the STOPF flag
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	// Check and handle TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3)
	{
		// TXE flag is set
		// Do data transmission

		// Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
	}

	// Check and handle RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3)
	{
		// RXNE flag is set
		// Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// Do data reception
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}
}

/**
  * @brief	Handles I2C error interrupts.
  *         Checks and clears error flags such as bus error, arbitration lost,
  *         ACK failure, overrun/underrun, and timeout.
  *         Also notifies the application about error events via callback.
  * @param	pI2CHandle: Pointer to the I2C handle structure.
  * @retval None
  */
void I2C_ER_IRQHandling (I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	// Status of the ITERREN bit
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	// Check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1<< I2C_SR1_BERR);

	if(temp1 && temp2)
	{
		// Clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Notify the application about bus error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_BERR);
	}

	// Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);

	if(temp1 && temp2)
	{
		// Clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Notify the application about arbitration lost error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_ARLO);
	}

	// Check for ACK failure error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);

	if(temp1 && temp2)
	{
		// Clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Notify the application about ACK failure error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_AF);
	}

	// Check for overrun/underrun error
	temp1 =  (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);

	if(temp1 && temp2)
	{
		// Clear the overrun/underrun error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Notify the application about overrun/underrun error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_OVR);
	}

	// Check for timeout error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);

	if(temp1 && temp2)
	{
		// Clear the timeout error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Notify the application about timeout error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/**
  * @brief  Enables or disables the ACK (Acknowledge) control bit.
  * @param	pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @param	EnorDi: ENABLE or DISABLE macros.
  * @retval None
  */
void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		// Enable the ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

	}else
	{
		// Disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}

/**
  * @brief  Generates the START condition on the I2C bus.
  * @param  pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @retval None
  */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/**
  * @brief  Generates the STOP condition on the I2C bus.
  * @param  pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @retval None
  */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/**
  * @brief  Executes the address phase for a write operation by sending the slave address with the write bit (0).
  * @param  pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @param  SlaveAddr: 7-bit slave address.
  * @retval None
  */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

/**
  * @brief  Executes the address phase for a read operation by sending the slave address with the read bit (1).
  * @param  pI2Cx: Base address of the I2C peripheral (I2C1 or I2C2).
  * @param  SlaveAddr: 7-bit slave address.
  * @retval None
  */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

/**
  * @brief  Clears the ADDR flag by reading SR1 and SR2 registers. Handles special case in master RX mode when RxSize == 1.
  * @param  pI2CHandle: Pointer to the I2C handle structure.
  * @retval None
  */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;

	// Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// Device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// Disable the ACK
				I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);

				// Clear the ADDR flag
				dummyread = pI2CHandle->pI2Cx->SR1;
				dummyread = pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
		}else
		{
			// Clear the ADDR flag
			dummyread = pI2CHandle->pI2Cx->SR1;
			dummyread = pI2CHandle->pI2Cx->SR2;
			(void)dummyread;
		}

	}else
	{
		// Device is in slave mode
		// Clear the ADDR flag
		dummyread = pI2CHandle->pI2Cx->SR1;
		dummyread = pI2CHandle->pI2Cx->SR2;
		(void)dummyread;
	}
}

/**
  * @brief  Close the I2C transmission process by disabling relevant interrupts and resetting handle state.
  * @param  pI2CHandle: Pointer to the I2C handle structure.
  * @retval None
  */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset handle state
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/**
  * @brief  Close the I2C reception process by disabling relevant interrupts, resetting handle state, and managing ACK.
  * @param  pI2CHandle: Pointer to the I2C handle structure.
  * @retval None
  */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset handle state and buffers
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	// Enable the ACK if it was initially configured as enabled
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAck(pI2CHandle->pI2Cx,ENABLE);
	}
}

/**
  * @brief  Weak callback function to notify the application about I2C events.
  * @param  pI2CHandle: Pointer to the I2C handle structure.
  * @param  AppEv: Event type
  * @retval None
  * @note   The application can override this function to handle events.
  */
__attribute__((weak)) void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	// This is a weak implementation. The application may override this function
}
