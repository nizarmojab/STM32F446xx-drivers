
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"

/*
 *Peripheral Clock setup
*/

/**************************************************************************************
 * @fn                       -GPIO_PeriClockControl
 *
 * @brief                    -
 *
 * @param[in]                -
 * @param[in]                -
 * @param[in]                -
 *
 * @return                   - none
 *
 * @Note                     -
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
					GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
		GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
		GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
		GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
		GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
		GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
		GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
		GPIOH_PCLK_DI();
		}
	}


}
/*
 * Init and De-init
*/
/**************************************************************************************
 * @fn                       -GPIO_Init
 *
 * @brief                    -Reference:GPIO port mode register page 187
 *
 * @param[in]                -
 * @param[in]                -
 * @param[in]                -
 *
 * @return                   - none
 *
 * @Note                     -
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp;   //temp.register
	//1. Configure the mode of gpio pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG)
    {
    	//the non interrupt mode
    	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // on a multiplir par 2 car chaque moder est sur 2bits
    	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    	pGPIOHandle->pGPIOx->MODER |=temp; //setting
    	temp =0;
    }
    else
    {
    	//this part will code later (interrupt mode)
    	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT)
    	{
    		//1. Configure the FTSR
    		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		//Clear the corresponding RTSR bit
    		EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
    	{
    		//1. Configure the RTSR
    		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		//Clear the corresponding FTSR bit
    		EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RFT)
    	{
    		//1. Configure both FTSR and RTSR
    		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		//Clear the corresponding FTSR bit
    		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	}
    	//2. Configure the GPIO port selection in SYSCFG_EXTICR
    	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
    	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
    	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
    	SYSCFG_PCLK_EN();
    	SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
    	//3. Enable the exti interrupt delivery using IMR
    	EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
    temp =0;
	//2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OSPEEDR|=temp;

    temp =0;
	//3. Configure the pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->PUPDR |=temp;

    temp =0;
	//4. Configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OTYPER |=temp;

	//5. Configure the alt functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_ALTEN)
    {
    	//Configure the alt function registers
    	uint8_t temp1,temp2;

    	temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    	temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
    	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) ); //clearing
    	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );

    }
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
    }else if(pGPIOx == GPIOB)
	{
    	GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
*/
/**************************************************************************************
 * @fn                       -GPIO_ReadFromInputPin
 *
 * @brief                    -Reference:GPIO port mode register page 187
 *
 * @param[in]                -
 * @param[in]                -
 * @param[in]                -
 *
 * @return                   - 0 or 1
 *
 * @Note                     -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber)
{
	uint8_t value;
	value =(uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;

}

/**************************************************************************************
 * @fn                       -GPIO_ReadFromInputPort
 *
 * @brief                    -Reference:GPIO port mode register page 187
 *
 * @param[in]                -
 * @param[in]                -
 * @param[in]                -
 *
 * @return                   - 0 or 1
 *
 * @Note                     -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value =(uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber,uint8_t Value)
{
	if (Value ==GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding pinnumber
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber); //operation XOR afin de basculer le numero de broche requis
}

/*
 * IRQ Configuration and ISR handling
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{	if(EnorDi ==ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |=(1 << IRQNumber);
		}else if(IRQNumber >31 && IRQNumber <64) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |=(1 << IRQNumber % 32);
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//program ISER2 register  //64 to 95
			*NVIC_ISER3 |=(1 << IRQNumber % 64);

		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |=(1 << IRQNumber);
		}else if(IRQNumber >31 && IRQNumber <64)
		{
			//program ISER1 register
			*NVIC_ISER1 |=(1 << IRQNumber % 32);
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER3 |=(1 << IRQNumber % 64);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1. first lets find out the ipr regsiter
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NB_PR_BIT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx * 4) |=( IRQPriority << (shift_amount) );

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |=(1 << PinNumber);
	}

}



