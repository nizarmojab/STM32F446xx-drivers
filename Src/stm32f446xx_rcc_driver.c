/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Sep 22, 2023
 *      Author: MOJAB Nizar
 */



#include "STM32F446xx_rcc_driver.h"

/*
 * Useful Data
 */

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};


uint32_t RCC_GetPLL_ClkValue()
{
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, sysclk, AhbPrsc, ApbPrsc;
	uint8_t clksrc, temp;

	// 1. Specify the value of the clock source
	clksrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);
	if (clksrc == 0)
	{
		sysclk = HSI_Clock_Value;
	}
	else if (clksrc == 1)
	{
		sysclk = HSE_Clock_Value;
	}
	else if (clksrc == 2)
	{
		sysclk = RCC_GetPLL_ClkValue();
	}

	// 2. Calculate the value of the AHB PreScaler
	temp = ( (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF );

	if (temp < 8)
	{
		AhbPrsc = 1;
	}
	else
	{
		AhbPrsc = AHB_PreScaler[temp - 8];
	}

	// 2. Calculate the value of the APB1 PreScaler
	temp = ( (RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7 );
	if (temp < 4 )
	{
		ApbPrsc = 1;
	}
	else
	{
		ApbPrsc = APB1_PreScaler[temp - 4];
	}

	// 4. Calculate the APB1 Peripheral clock
	pclk1 = ( (sysclk / AhbPrsc) / ApbPrsc);

	// 5. return the value of the clock the APB1 peripheral clock
	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, sysclk, AhbPrsc, ApbPrsc;
	uint8_t clksrc, temp;

	// 1. Specify the value of the clock source
	clksrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);
	if (clksrc == 0)
	{
		sysclk = HSI_Clock_Value;
	}
	else if (clksrc == 1)
	{
		sysclk = HSE_Clock_Value;
	}
	else if (clksrc == 2)
	{
		sysclk = RCC_GetPLL_ClkValue();
	}

	// 2. Calculate the value of the AHB PreScaler
	temp = ( (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF );

	if (temp < 8)
	{
		AhbPrsc = 1;
	}
	else
	{
		AhbPrsc = AHB_PreScaler[temp - 8];
	}

	// 2. Calculate the value of the APB2 PreScaler
	temp = ( (RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7 );
	if (temp < 4 )
	{
		ApbPrsc = 1;
	}
	else
	{
		ApbPrsc = APB1_PreScaler[temp - 4];
	}

	// 4. Calculate the APB1 Peripheral clock
	pclk2 = ( (sysclk / AhbPrsc) / ApbPrsc);

	// 5. return the value of the clock the APB1 peripheral clock
	return pclk2;
}
