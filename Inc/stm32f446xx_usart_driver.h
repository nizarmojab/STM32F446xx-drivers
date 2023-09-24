/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Sep 22, 2023
 *      Author: MOJAB Nizar
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_


#include "STM32F446xx.h"

/*
 * This is a Configuration structure for a USARTx peripheral
 */

typedef struct {
	uint8_t  USART_Mode;
	uint32_t  USART_Baud;
	uint8_t  USART_NoOfStopBits;
	uint8_t  USART_WordLength;
	uint8_t  USART_ParityControl;
	uint8_t  USART_HWFlowControl;
}USART_Config_t;

/*
 * This is a Handle structure for a USARTx peripheral
 */

typedef struct {
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t 	   *pTxBuffer; /* To store the app Tx Buffer address */
	uint8_t 	   *pRxBuffer; /* To store the app Tx Buffer address */
	uint32_t 	   TxLen;		 /* To store Tx Len */
	uint32_t 	   RxLen;		 /* To store Rx Len */
	uint8_t		   TxBusyState;  /* To store Communication State */
	uint8_t		   RxBusyState;  /* To store Communication State */
}USART_Handle_t;


/*
 * Macros for possible values to set the configuration of the peripheral
 */


// @USART Mode

#define USART_MODE_ONLY_TX	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX   	2

// @USART Baud Rate

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

// @USART NoOfStopBits

#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

// @USART_WordLength

#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

// @USART_ParityControl

#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

// @USART_HWFlowControl

#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART related status flags definitions
 */

#define USART_FLAG_PE			(1 << USART_SR_PE)
#define USART_FLAG_FE			(1 << USART_SR_FE)
#define USART_FLAG_NF			(1 << USART_SR_NF)
#define USART_FLAG_ORE			(1 << USART_SR_ORE)
#define USART_FLAG_IDLE 		(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE			(1 << USART_SR_RXNE)
#define USART_FLAG_TC			(1 << USART_SR_TC)
#define USART_FLAG_TXE			(1 << USART_SR_TXE)
#define USART_FLAG_LBD			(1 << USART_SR_LBD)
#define USART_FLAG_CTS			(1 << USART_SR_CTS)

// Application State

#define USART_STATE_READY			0
#define USART_STATE_BUSY_IN_RX	1
#define USART_STATE_BUSY_IN_TX	2

// Interrupt EVENTS Macros
#define USART_EVENT_TX_COMPLETE		1
#define USART_EVENT_RX_COMPLETE		2
#define USART_EVENT_CTS				3
#define USART_EVENT_TC				4
#define USART_EVENT_ORE				5
#define USART_EVENT_IDLE			6
#define USART_ERREVENT_ORE			7
#define USART_ERREVENT_NE			8
#define USART_ERREVENT_FE			9

/*********************************************************************************************************************************
 * 												APIs SUPPORTED BY THIS DRIVER													 *
 * 								 For More information about the APIs check the function definition							     *
 *********************************************************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClkControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI);

/*
 * Init and De-init
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
 * IRQ Configuration and ISR handling
 */
void USART_IrqInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void USART_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint16_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application callback
 */
_weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);



#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
