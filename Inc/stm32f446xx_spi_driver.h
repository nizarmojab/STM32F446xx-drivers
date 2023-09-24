/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Sep 19, 2023
 *      Author: MOJAB Nizar
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_
#include "STM32F446xx.h"



/*
 * This is a Configuration structure for a SPIx peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t	SPI_SclkSPeed;
	uint8_t SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is a Handle structure for SPIx peripheral
 */

typedef struct {
	SPI_RegDef_t	*pSPIx; /* This holds the base address of SPIx(x:0,1,2,3) peripheral*/
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer; /* To store the app Tx buffer address*/
	uint8_t			*pRxBuffer; /* To store the app Rx buffer address*/
	uint32_t		TxLen;		/* To store the app Tx Length*/
	uint32_t		RxLen;		/* To store the app Rx Length*/
	uint8_t			TxState;	/* To store the app Tx State*/
	uint8_t			RxState;	/* To store the app Rx State*/
}SPI_Handle_t;

/******************************** STRUCTURE MACROS *********************/
/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE  0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD             1
#define SPI_BUS_CONFIG_HD             2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3


/*
 * @SPI_SclkSPeed : Serial Clock Speed
 */

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF : Data Frame Format
 */

#define SPI_DFF_8BITS  0
#define SPI_DFF_16BITS 1


/*
 * @SPI_CPOL : Clock Polarization
 */

#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW  0

/*
 * @SPI_CPHA : Clock Phase
 */

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW  0

/*
 * @SPI_SSM : Software Slave management
 */

#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

/*
 * SPI related status flags definitions
 */

#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG	(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG	(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG (1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG	(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG	(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG	(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG	(1 << SPI_SR_FRE)

/*
 * SPI application states
 */

#define SPI_STATE_READY				0
#define SPI_STATE_BUSY_IN_RX		1
#define SPI_STATE_BUSY_IN_TX		2

/*
 * Possible SPI Application Events
 */

#define SPI_EVENT_TX_COMPLETE		1
#define SPI_EVENT_RX_COMPLETE		2
#define SPI_EVENT_OVR_ERROR			3
#define SPI_EVENT_MODF_ERROR		4


/***********************************************************************/

/*********************************************************************************************************************************
 * 												APIs SUPPORTED BY THIS DRIVER													 *
 * 								 For More information about the APIs check the function definition							     *
 *********************************************************************************************************************************/

/*
 * peripheral clock setup
 */
void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandlex);
void SPI_Dinit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */

void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */

void SPI_IrqInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IrqHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHanlde);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application specific function
 */

_weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);



#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
