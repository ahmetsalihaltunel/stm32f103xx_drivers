/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: May 6, 2025
 *      Author: ASA
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_


#include "stm32f103xx.h"

/*
 * Configuration structure for SPIx Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;					// Device mode (Master or Slave)										@SPI_DeviceMode
	uint8_t SPI_BusConfig;					// Bus config (Full duplex, half dublex ...)							@SPI_BusConfig
	uint8_t SPI_SclkSpeed;					// Speed (/2, /4, /8 ...)												@SPI_SclkSpeed
	uint8_t SPI_DFF;						// Data Frame Format (8 bits, 16 bits) 									@SPI_DFF
	uint8_t SPI_CPOL;						// CPOL																	@SPI_CPOL
	uint8_t SPI_CPHA;						// CPHA																	@SPI_CPHA
	uint8_t SPI_SSM;						// SSM																	@SPI_SSM
}SPI_Config_t;

/*
 * Handle structure for SPIx Peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t	*pTxBuffer;						// To store tx buffer address
	uint8_t	*pRxBuffer;						// To store rx buffer address
	uint32_t TxLen;							// To store tx len
	uint32_t RxLen;							// To store rx len
	uint8_t	TxState;						// To store tx state
	uint8_t	RxState;						// To store rx state

}SPI_Handle_t;


/*
 *  @SPI_DeviceMode
 *  Device Mode Configuration Macros
 */
#define SPI_DEVICE_MODE_SLAVE			0U
#define SPI_DEVICE_MODE_MASTER			1U

/*
 *  @SPI_BusConfig
 *  Bus Config Configuration Macros
 */
#define SPI_BUS_CONFIG_FD				1U									// Full Duplex
#define SPI_BUS_CONFIG_HD				2U									// Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RX		3U									// Simplex only RX
#define SPI_BUS_CONFIG_SIMPLEX_TX		4U									// Simplex only TX

/*
 *  @SPI_SclkSpeed
 *  Sclk Speed Configuration Macros
 */
#define SPI_SCLK_SPEED_DIV2				0U									// SPI SCLK = fPCLK / 2
#define SPI_SCLK_SPEED_DIV4				1U									// SPI SCLK = fPCLK / 4
#define SPI_SCLK_SPEED_DIV8				2U									// SPI SCLK = fPCLK / 8
#define SPI_SCLK_SPEED_DIV16			3U									// SPI SCLK = fPCLK / 16
#define SPI_SCLK_SPEED_DIV32			4U									// SPI SCLK = fPCLK / 32
#define SPI_SCLK_SPEED_DIV64			5U									// SPI SCLK = fPCLK / 64
#define SPI_SCLK_SPEED_DIV128			6U									// SPI SCLK = fPCLK / 128
#define SPI_SCLK_SPEED_DIV256			7U									// SPI SCLK = fPCLK / 256

/*
 *  @SPI_DFF
 *  DFF Configuration Macros
 */
#define SPI_DFF_8BITS 					0U									// 8 bit data frame format
#define SPI_DFF_16BITS					1U									// 16 bit data frame format

/*
 *  @SPI_CPOL
 *  CPOL Configuration Macros
 */
#define SPI_CPOL_LOW					0U									// Clock idle state is low
#define SPI_CPOL_HIGH					1U									// Clock idle state is high

/*
 *  @SPI_CPHA
 *  CPHA Configuration Macros
 */
#define SPI_CPHA_LOW					0U									// Data captured on first clock transition
#define SPI_CPHA_HIGH					1U									// Data captured on second clock transition

/*
 *  @SPI_SSM
 *  SSM Configuration Macros
 */
#define SPI_SSM_DI						0U									// Hardware slave management enabled (SSM disabled)
#define SPI_SSM_EN						1U									// Software slave management enabled

/*
 * SPI Status Flags Definitions
 */
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)					// Transmit buffer empty flag
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)					// Receive buffer not empty flag
#define SPI_BUSY_FLAG					(1 << SPI_SR_BSY)					// SPI is busy in communication


/*
 * Possible SPI Application States
 */
#define SPI_READY						0U									// SPI is ready for communication
#define SPI_BUSY_IN_RX					1U									// SPI is busy receiving data
#define SPI_BUSY_IN_TX					2U									// SPI is busy transmitting data

/*
 * Possible SPI Applications Events
 */
#define SPI_EVENT_TX_CMPLT				1U									// Transmission complete event
#define SPI_EVENT_RX_CMPLT				2U									// Reception complete event
#define SPI_EVENT_OVR_ERR				3U									// Overrun error event


/*************************************************************************************
 *  						APIs supported by this driver
 *  		For more information about the APIs check the function definitions
 *************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl (SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 *  Init and De-Init
 */
void SPI_Init (SPI_Handle_t *pSPIHandle);
void SPI_DeInit (SPI_RegDef_t *pSPIOx);

/*
 *  Data Send and Receive
 */
void SPI_SendData (SPI_RegDef_t *pSPIOx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData (SPI_RegDef_t *pSPIOx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  IRQ Configuration and IRQ Handling
 */
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling (SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Applicaion Callback
 */
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
