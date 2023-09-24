/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Sep 19, 2023
 *      Author: MOJAB Nizar
 */

#include "STM32F446xx_spi_driver.h"

// Helper functions used in the "SPI_IRQHandling"

static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERROR_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * peripheral clock setup
 */

/********************************************************
 * @ fn 			- SPI_PeriClkControl
 *
 * @brief			- This function is responsible for Enabling or Disabling peripheral clock for a given SPI peripheral
 *
 * @param[0]		- Base address of SPI peripheral
 * @param[1]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else if(ENorDI == DISABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
}

/*
 * Init and De-init
 */

/********************************************************
 * @ fn 			- SPI_INIT
 *
 * @brief			- This function initialize the SPI peripheral and set its configuration
 *
 * @param[1]		- Base address of SPI Handle variable

 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandlex)
{
	//First lets enable the clock for SPI peripheral
	SPI_PeriClkControl(pSPIHandlex->pSPIx , ENABLE);

	//first let's configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//configure the device mode
	tempreg |= (pSPIHandlex->SPIConfig.SPI_DeviceMode << 2);

	//configure the bus
	if (pSPIHandlex -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidirectional mode should be cleared
		tempreg &= ~(1 << 15);
	}
	else if (pSPIHandlex ->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidirectional mode should be set
		tempreg |= (1 << 15);
	}
	else if (pSPIHandlex -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//bidirectional mode should be cleared
		tempreg &= ~(1 << 15);
		//RXONLY bit must be set
		tempreg |= (1 << 10);
	}

	//configure the serial clock
	tempreg |= pSPIHandlex-> SPIConfig.SPI_SclkSPeed << 3;
	//configure data frame format
	tempreg |= pSPIHandlex-> SPIConfig.SPI_DFF << 11;
	//configure clock polarization
	tempreg |= pSPIHandlex -> SPIConfig.SPI_CPOL << 1;
	//configure clock phase
	tempreg |= pSPIHandlex -> SPIConfig.SPI_CPHA << 0;
	//configure software slave management
	tempreg |= pSPIHandlex -> SPIConfig.SPI_SSM << 9;

	pSPIHandlex -> pSPIx -> SPI_CR1 = tempreg;
}

/********************************************************
 * @ fn 			- SPI_Dinit
 *
 * @brief			- This function reset the SPI peripheral and its registers
 *
 * @param[1]		- Base address of SPI Handle variable

 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void SPI_Dinit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}


/*
 * Data Send and Receive
 */

/********************************************************
 * @ fn 			- SPI_GetFlagStatus
 *
 * @brief			- This function return the value of the flags in the SPI status register
 *
 * @param[1]		- Base address of SPI config variable
 * @param[2]		- FLAG_NAME

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t FlagName)
{
	if (pSPIx -> SPI_SR & FlagName) {
		return FLAG_SET;
	}
	else{
		return FLAG_RESET;
	}
}

/********************************************************
 * @ fn 			- SPI_SendData
 *
 * @brief			- This function Send Frames to SPI TxBuffer
 *
 * @param[1]		- Base address of SPI config variable
 * @param[2]		- Base address of SPI Tx Buffer
 * @param[3]		- Length of the frame (1 or 2 Byte)

 * @return 			- none
 *
 * @note 			- This is a blocking call
 *
 *******************************************************/

void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0) {
		// 1. wait until TXE is set
		while ( SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET );

		// 2. check t he DFF bit in CR1
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load the data into the DR
			pSPIx->SPI_DR = *((uint16_t *) pTxBuffer);
			Len -= 2;

			//2. increment the TxBuffer pointer to the next frame to be sent
			(uint16_t*)pTxBuffer ++;
		}
		else
		{
			//8bit data format
			//1. Load the data into the DR
			pSPIx->SPI_DR = *( pTxBuffer);
			Len--;

			//2. increment the TxBuffer pointer to the next frame to be sent
			pTxBuffer ++;
		}

	}
}

/********************************************************
 * @ fn 			- SPI_ReceiveData
 *
 * @brief			- This function Receive Frames on the SPI RxBuffer
 *
 * @param[1]		- Base address of SPI Handle variable
 * @param[2]		- Base address of SPI Tx Buffer
 * @param[3]		- Length of the frame (1 or 2 Byte)

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0) {
			// 1. wait until RXNE is set
			while ( SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET );

			// 2. check t he DFF bit in CR1
			if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
			{
				//16 bit DFF
				//1. load the data from the DR to Rxbuffer address
				*((uint16_t *)pRxBuffer) = pSPIx->SPI_DR ;
				Len -= 2;

				//2. increment the TxBuffer pointer to the next frame to be sent
				(uint16_t*)pRxBuffer --;
			}
			else
			{
				//8bit data format
				//1. Load the data into the DR
				*(pRxBuffer) = pSPIx->SPI_DR;
				Len--;

				//2. increment the TxBuffer pointer to the next frame to be sent
				pRxBuffer --;
			}

		}
}

// *************************** Send and Receive API based on SPI interrupts **********************************
/********************************************************
 * @ fn 			- SPI_SendDataIT
 *
 * @brief			- This function Send Frames to SPI TxBuffer when an interrupt occur
 *
 * @param[1]		- Base address of SPI handle variable
 * @param[2]		- Base address of SPI Tx Buffer
 * @param[3]		- Length of the frame (1 or 2 Byte)

 * @return 			- none
 *
 * @note 			- This is a blocking call
 *
 *******************************************************/
uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_STATE_BUSY_IN_TX)
	{
		//1. Save the Tx Buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_STATE_BUSY_IN_TX;

		//3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data Transmission will be handled by the ISR code.
	}
	return state;
}

/********************************************************
 * @ fn 			- SPI_ReceiveDataIT
 *
 * @brief			- This function receive Frames from SPI RxBuffer when an interrupt occur
 *
 * @param[1]		- Base address of SPI handle variable
 * @param[2]		- Base address of SPI Rx Buffer
 * @param[3]		- Length of the frame (1 or 2 Byte)

 * @return 			- none
 *
 * @note 			- This is a blocking call
 *
 *******************************************************/
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_STATE_BUSY_IN_RX)
	{
		//1. Save the Tx Buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_STATE_BUSY_IN_RX;

		//3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data Transmission will be handled by the ISR code.
	}
	return state;
}


/*
 * IRQ configuration and ISR handling
 */

/********************************************************
 * @ fn 			- SPI_IrqInterruptConfig
 *
 * @brief			- This function Set configurations to enable or disable interrupt from SPI device.
 *
 * @param[1]		- The Interrupt request number
 * @param[2]		- ENABLE or DISABLE

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void SPI_IrqInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (IRQNumber <= 31) {
			//set ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//set ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96) {
			//set ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else if (ENorDI == DISABLE)
	{
		if (IRQNumber <= 31) {
			//set ICER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//set ICER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) {
			//set ICER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/********************************************************
 * @ fn 			- SPI_IRQ_PriorityConfig
 *
 * @brief			- This function Set configurations to set interrupt priority.
 *
 * @param[1]		- The Interrupt request number
 * @param[2]		- Interrupt priority

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void SPI_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NB_PR_BIT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) = (IRQPriority << shift_amount);
}

/********************************************************
 * @ fn 			- SPI_IrqHandling
 *
 * @brief			- This function Set configurations to set interrupt priority.
 *
 * @param[1]		- Base address of SPI Handle variable

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void SPI_IrqHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp[6] = {( (pSPIHandle->pSPIx->SPI_SR) & (1 << SPI_SR_TXE)),
					   ( (pSPIHandle->pSPIx->SPI_CR2) & (1 << SPI_CR2_TXEIE) ),
					   ( (pSPIHandle->pSPIx->SPI_SR) & (1 << SPI_SR_RXNE)),
					   ( (pSPIHandle->pSPIx->SPI_CR2) & (1 << SPI_CR2_RXNEIE) ),
					   ( (pSPIHandle->pSPIx->SPI_SR) & (1 << SPI_SR_OVR)),
					   ( (pSPIHandle->pSPIx->SPI_CR2) & (1 << SPI_CR2_ERRIE) )};

	if ( temp[0] && temp[1] )
	{
		SPI_TXE_interrupt_handle(pSPIHandle);
	}
	else if (temp[2] && temp[3])
	{
		SPI_RXE_interrupt_handle(pSPIHandle);
	}
	else if (temp[4] && temp[5])
	{
		SPI_OVR_ERROR_interrupt_handle(pSPIHandle);
	}
	else {
		return;
	}
}

//Implementation of helper functions declared in the scope of "SPI_IrqHandling" function
static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. test if the data frame format is 8 or 16 bits
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit data frame format

		//1. load the data in the TxBuffer into Data register
		(pSPIHandle->pSPIx->SPI_DR) = *( (uint16_t *)pSPIHandle->pTxBuffer );

		//2. Decrement the data length
		pSPIHandle->TxLen -= 2;

		//3. Increment the TxBuffer pointer to the next position
		(uint16_t *) pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit data frame

		//1. load the data in the TxBuffer into Data register
		(pSPIHandle->pSPIx->SPI_DR) = *pSPIHandle->pTxBuffer;

		//2. Decrement the data length
		pSPIHandle->TxLen --;

		//3. Increment the TxBuffer pointer to the next position
		(pSPIHandle->pTxBuffer)++;
	}
	if (pSPIHandle->TxLen == 0)
	{
		//TxLen is Zero, So the SPI transmission shall be closed and inform the application that Tx is over

		//closing the transmission
		SPI_CloseTransmission(pSPIHandle);

		//returning to the app that the event of sending data is complete
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);

	}
}
static void SPI_RXE_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. test if the data frame format is 8 or 16 bits
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit data frame format

		//1. load the data in the TxBuffer into Data register
		 *( (uint16_t *)pSPIHandle->pRxBuffer ) = (pSPIHandle->pSPIx->SPI_DR);

		//2. Decrement the data length
		pSPIHandle->RxLen -= 2;

		//3. Increment the TxBuffer pointer to the next position
		(uint16_t *) pSPIHandle->pRxBuffer--;
	}
	else
	{
		//8 bit data frame

		//1. load the data in the TxBuffer into Data register
		*pSPIHandle->pRxBuffer = (pSPIHandle->pSPIx->SPI_DR);

		//2. Decrement the data length
		pSPIHandle->RxLen --;

		//3. Increment the TxBuffer pointer to the next position
		(pSPIHandle->pRxBuffer)--;
	}
	if (pSPIHandle->RxLen == 0)
	{
		// Reception is complete

		// closing the reception
		SPI_CloseReception(pSPIHandle);

		//returning to the app that the event of receiving data is complete
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}
static void SPI_OVR_ERROR_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. clear the Overrun flag

	if (pSPIHandle->TxState != SPI_STATE_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// clearing the TXEIE prevents interrupts from setting up of TXE flag
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

	// Reseting the value of the variables used during ISR executing
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_STATE_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// 1. clear the RXNEIE bit to disable interrupt from its part
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

	// Reseting the value of the variables used during ISR executing
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_STATE_READY;
}

_weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//this is a weak definition of the function , it must be defined by the user
}

/*
 * Other Peripheral Control APIs
 */


/********************************************************
 * @ fn 			- SPI_PeripheralControl
 *
 * @brief			- This function is responsible for Enabling or Disabling peripheral.
 *
 * @param[0]		- Base address of SPI peripheral
 * @param[1]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/********************************************************
 * @ fn 			- SPI_SSIConfig
 *
 * @brief			- This function is responsible for Enabling or Disabling SSI Configuration.
 *
 * @param[0]		- Base address of SPI peripheral
 * @param[1]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
		{
			pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

/********************************************************
 * @ fn 			- SPI_SSOEConfig
 *
 * @brief			- This function is responsible for Enabling or Disabling SSOE Configuration.
 *
 * @param[0]		- Base address of SPI peripheral
 * @param[1]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
		{
			pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}
