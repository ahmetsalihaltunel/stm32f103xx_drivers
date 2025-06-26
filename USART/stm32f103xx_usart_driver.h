/*
 * stm32f103xx_usart_driver.h
 *
 *  Created on: Jun 22, 2025
 *      Author: ASA
 */

#ifndef INC_STM32F103XX_USART_DRIVER_H_
#define INC_STM32F103XX_USART_DRIVER_H_


#include "stm32f103xx.h"

/*
 * Configuration structure for USARTx Peripheral
 */
typedef struct
{
	uint8_t USART_Mode;						// USART mode of operation												@USART_Mode
	uint32_t USART_Baud;					// USART communication baud rate (9600, 115200...)						@USART_Baud
	uint8_t USART_NoOfStopBits;				// Number of stop bits (1, 0.5, 2, 1.5)									@USART_NoOfStopBits
	uint8_t USART_WordLength;				// Word length (8 bits or 9 bits)										@USART_WordLength
	uint8_t USART_ParityControl;			// Parity control setting (No parity, Even parity, Odd parity)			@USART_ParityControl
	uint8_t USART_HWFlowControl;			// Hardware flow control (None, RTS, CTS, or RTSCTS)					@USART_HWFlowControl

}USART_Config_t;

/*
 * Handle structure for USARTx Peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;				// Pointer to the base address of the USART peripheral
	USART_Config_t USART_Config;			// Holds the USART configuration settings
	uint8_t *pTxBuffer;						// To store tx buffer address
	uint8_t *pRxBuffer;						// To store rx buffer address
	uint32_t TxLen;							// To store tx len
	uint32_t RxLen;							// To store rx len
	uint8_t TxBusyState;					// To store the state of tx
	uint8_t RxBusyState;					// To store the state of rx
}USART_Handle_t;

/*
 *	@USART_Mode
 *	Mode Selection Macros
 */
#define USART_MODE_TX							0U
#define USART_MODE_RX							1U
#define USART_MODE_TXRX							2U

/*
 *	@USART_Baud
 *	BaudRate Selection Macros
 */
#define USART_STD_BAUD_1200						1200U
#define USART_STD_BAUD_2400						2400U
#define USART_STD_BAUD_9600						9600U
#define USART_STD_BAUD_19200					19200U
#define USART_STD_BAUD_38400					38400U
#define USART_STD_BAUD_57600					57600U
#define USART_STD_BAUD_115200					115200U

/*
 *	@USART_ParityControl
 *	Parity Control Macros
 */
#define USART_PARITY_DISABLE					0U
#define USART_PARITY_EVEN						1U
#define USART_PARITY_ODD						2U

/*
 *	@USART_WordLength
 *	Word Length Macros
 */
#define USART_WORDLEN_8BITS						0U
#define USART_WORDLEN_9BITS						1U

/*
 *	@USART_NoOfStopBits
 *	Number of Stop Bits Macros
 */
#define USART_STOPBITS_1						0U
#define USART_STOPBITS_0_5						1U
#define USART_STOPBITS_2						2U
#define USART_STOPBITS_1_5						3U

/*
 *	USART_HWFlowControl
 *	Hardware Flow Control Macros
 */
#define USART_HW_FLOW_CTRL_NONE					0U
#define USART_HW_FLOW_CTRL_CTS					1U
#define USART_HW_FLOW_CTRL_RTS					2U
#define USART_HW_FLOW_CTRL_CTSRTS				3U

/*
 * Application states
 */
#define USART_READY								0U
#define USART_BUSY_IN_RX						1U
#define USART_BUSY_IN_TX						2U

/*
 * USART Application Events
 */
#define 	USART_EVENT_TX_CMPLT				0U
#define		USART_EVENT_RX_CMPLT				1U
#define		USART_EVENT_IDLE					2U
#define		USART_EVENT_CTS						3U
#define		USART_EVENT_PE						4U
#define		USART_ERR_FE						5U
#define		USART_ERR_NE						6U
#define		USART_ERR_ORE						7U

/*
 * USART Status Flags Definitions
 */
#define USART_FLAG_PE						(1 << USART_SR_PE)					// Parity error
#define USART_FLAG_FE						(1 << USART_SR_FE)					// Framing error
#define USART_FLAG_NE						(1 << USART_SR_NE)					// Noise error flag
#define USART_FLAG_ORE						(1 << USART_SR_ORE)					// Overrun error
#define USART_FLAG_IDLE						(1 << USART_SR_IDLE)				// IDLE line detected
#define USART_FLAG_RXNE						(1 << USART_SR_RXNE)				// Read data register not emptY
#define USART_FLAG_TC						(1 << USART_SR_TC)					// Transmission complete
#define USART_FLAG_TXE						(1 << USART_SR_TXE)					// Transmit data register empty
#define USART_FLAG_LBD						(1 << USART_SR_LBD)					// LIN break detection flag
#define USART_FLAG_CTS						(1 << USART_SR_CTS)					// CTS flag


/*************************************************************************************
 *  						APIs supported by this driver
 *  		For more information about the APIs check the function definitions
 *************************************************************************************/

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and IRQ Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Applicaion Callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);


#endif /* INC_STM32F103XX_USART_DRIVER_H_ */
