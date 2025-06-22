/*
 * stm32f103xx_i2c_driver.h
 *
 *  Created on: May 22, 2025
 *      Author: ahmet
 */

#ifndef INC_STM32F103XX_I2C_DRIVER_H_
#define INC_STM32F103XX_I2C_DRIVER_H_


#include "stm32f103xx.h"

/*
 * Configuration structure for I2Cx Peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;					// Clock speed (Standard mode, Fast mode)								@I2C_SCLSpeed
	uint8_t I2C_DeviceAddress;				// Device's own I2C address												@I2C_DeviceAddress
	uint8_t I2C_ACKControl;					// Enables or disables ACKing (Acknowledge control)						@I2C_ACKControl
	uint16_t I2C_FMDutyCycle;				// Duty cycle in fast mode (2 or 16/9)   								@I2C_FMDutyCycle

}I2C_Config_t;

/*
 * Handle structure for I2Cx Peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;					// Pointer to the base address of the I2C peripheral
	I2C_Config_t I2C_Config;				// Holds the I2C configuration settings
	uint8_t *pTxBuffer;						// To store tx buffer address
	uint8_t *pRxBuffer;						// To store rx buffer address
	uint32_t TxLen;							// To store tx len
	uint32_t RxLen;							// To store rx len
	uint8_t TxRxState;						// To store the state of tx and rx
	uint8_t DevAddr;						// To store the device address
	uint32_t RxSize;						// To store the rx size
	uint8_t Sr;								// To store the repeated start value

}I2C_Handle_t;

/*
 * I2C Application States
 */
#define I2C_READY					0U
#define I2C_BUSY_IN_RX				1U
#define I2C_BUSY_IN_TX				2U

/*
 *	@I2C_SCLSpeed
 *	Clock Speed Configuration Macros
 */
#define I2C_SCL_SPEED_SM			100000U
#define I2C_SCL_SPEED_FM			400000U

/*
 *  @I2C_ACKControl
 *  Acknowledge Control Configuration Macros
 */
#define I2C_ACK_DISABLE				0U
#define I2C_ACK_ENABLE				1U

/*
 *  @I2C_FMDutyycle
 *  Duty Cycle in Fast Mode Configuration Macros
 */
#define I2C_FM_DUTY_2				0U
#define I2C_FM_DUTY_16_9			1U

/*
 * I2C Status Flags Definitions
 */
#define I2C_FLAG_SB						(1 << I2C_SR1_SB)					// Start bit (Master mode) flag
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)					// Address sent (master mode)/matched (slave mode) flag
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)					// Byte transfer finished flag
#define I2C_FLAG_STOPF					(1 << I2C_SR1_STOPF)				// Stop detection (slave mode) flag
#define I2C_FLAG_RXNE					(1 << I2C_SR1_RXNE)					// Receive buffer not empty flag
#define I2C_FLAG_TXE					(1 << I2C_SR1_TXE)					// Transmit buffer empty flag
#define I2C_FLAG_BERR					(1 << I2C_SR1_BERR)					// Bus error flag
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)					// Arbitration lost (master mode) flag
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)					// Acknowledge failure flag
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)					// Overrun/Underrun flag
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)				// Timeout or Tlow error flag

/*
 * I2C Application Events
 */
#define I2C_EV_TX_CMPLT					0
#define I2C_EV_STOP						1
#define I2C_EV_RX_CMPLT					2
#define I2C_ERROR_BERR					3
#define I2C_ERROR_ARLO					4
#define I2C_ERROR_AF					5
#define I2C_ERROR_OVR					6
#define I2C_ERROR_TIMEOUT				7

/*
 * I2C Other Definitions
 */
#define I2C_ENABLE_SR					SET
#define I2C_DISABLE_SR					RESET

/*************************************************************************************
 *  						APIs supported by this driver
 *  		For more information about the APIs check the function definitions
 *************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl (I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 *	Init and De-Init
 */
void I2C_Init (I2C_Handle_t *pI2CHandle);
void I2C_DeInit (I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/*
 * IRQ Configuration and IRQ Handling
 */
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling (I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling (I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Applicaion Callback
 */
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle,uint8_t AppEv);



#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */
