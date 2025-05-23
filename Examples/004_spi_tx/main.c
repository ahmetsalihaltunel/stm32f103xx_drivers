/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*
 * Note: I used arduino as a slave.The code was published by niekiran.
 * Link:
 */

#include <string.h>
#include "stm32f103xx.h"

// Simple delay function
void delay(void)
{
    for(uint32_t i = 0; i < 500000/2 ; i++);
}


/*
 * SPI2 Pin Configurations
 * PB12 SPI2_NSS
 * PB13 SPI2_SCLK
 * PB14 SPI2_MISO  (Not used)
 * PB15 SPI2_MOSI
 */

// Configure GPIO pins for SPI2
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.PinMode = GPIO_MODE_AF_PUSH_PULL;
	SPIPins.GPIO_PinConfig.PinSpeed = GPIO_SPEED_OUTPUT_10MHZ;

	//NSS
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//SCLK
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	/*
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	*/

	//MOSI
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_15;

	// Initialize the GPIO pins with the configured settings
	GPIO_Init(&SPIPins);
}

// Configure SPI2 as Master
void SPI2_Inits(void)
{
    // Create and configure the handle structure for SPI2
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;			// Full-duplex
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;	// Master mode
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;		// Clock = fPCLK / 16
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;					// 8 bit data
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;					// Clock phase low
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;					// Clock polarity low
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;						// Hardware NSS management

	// Initialize the SPI2 with the configured settings
	SPI_Init(&SPI2handle);
}

// Initialize GPIO for button input (PB5)
void GPIO_ButtonInit(void)
{
    // Create and configure the handle structure for Button GPIO (PB5)
    GPIO_Handle_t GPIOButton;

    GPIOButton.pGPIOx = GPIOB;                           			// Set GPIO port to GPIOB
    GPIOButton.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_5; 			// Set pin number 5
    GPIOButton.GPIO_PinConfig.PinSpeed = GPIO_SPEED_INPUT;			// Set pin as input
    GPIOButton.GPIO_PinConfig.PinMode = GPIO_MODE_INPUT_PU_PD; 		// S pin with pull-up/pull-down

    // Initialize the GPIO pin 5 with the configured settings
    GPIO_Init(&GPIOButton);

}


int main(void)
{
	// Data to send over SPI
	char user_data[] = "Hello World";

	// Initialize SPI and button GPIO
	SPI2_GPIOInits();												// Configure SPI pins
	SPI2_Inits();													// Configure SPI2 peripheral
	SPI_SSOEConfig(SPI2, ENABLE);									// Enable automatic NSS control
	GPIO_ButtonInit();												// Initialize button pin

	while(1)
	{
		// Wait for button press
		while (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_5) == 0);

		delay();													// Simple debounce delay

		SPI_PeripheralControl(SPI2, ENABLE);						// Enable SPI2 peripheral

		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);							// Send data length first
		SPI_SendData(SPI2,(uint8_t*) user_data, strlen(user_data));	// Send actual data

		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));				// Wait for transmission to complete

		SPI_PeripheralControl(SPI2, DISABLE);						// Disable SPI2 peripheral

	}
	return 0;
}
