/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Sep 22, 2023
 *      Author: MOJAB Nizar
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "STM32F446xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLL_ClkValue();




#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
