/*
 * stm32f103xx_rcc_driver.c
 *
 *  Created on: May 25, 2025
 *      Author: ahmet
 */

	#include "stm32f103xx_rcc_driver.h"

// AHB prescaler look-up table: Maps HPRE[3:0] values
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};

// APB1 prescaler look-up table: Maps PPRE1[2:0] values
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

/*
 * @brief	Returns the APB1 peripheral clock frequency (PCLK1)
 * @param	Uses system clock source and prescalers to compute the final PCLK1.
 * @retval 	PCLK1 frequency in Hz
*/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint16_t ahbp,apb1p;
	uint8_t clksrc,ahb_bits,apb1_bits;

    // Read system clock source from RCC_CFGR register
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	// Determine base system clock frequency
	if(clksrc == 0)
	{
		// Clock is HSI
		SystemClk = 16000000;
	}else if (clksrc == 1)
	{
		// Clock is HSE
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		// Clock is PLL
		//SystemClk = RCC_GetPLLOutputClock();
	}

    // Read AHB prescaler bits
	ahb_bits = ((RCC->CFGR >> 4) & 0xF);

	// Calculate the prescaler value of AHB
	if(ahb_bits < 8)
	{
		ahbp = 1;	// No division
	}
	else
	{
		ahbp = AHB_PreScaler[ahb_bits-8];
	}

	// Read the prescaler value of APB1
	apb1_bits = ((RCC->CFGR >> 10 ) & 0x7);

	// Calculate the prescaler value of APB1
	if(apb1_bits < 4)
	{
		apb1p = 1;	// No division
	}else
	{
		apb1p = APB1_PreScaler[apb1_bits-4];
	}

	// Calculate final PCLK1 frequency
	pclk1 =  (SystemClk / ahbp) / apb1p;

	return pclk1;
}
