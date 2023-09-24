/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Sep 22, 2023
 *      Author: MOJAB Nizar
 */

#include "STM32F446xx_i2c_driver.h"

/*
 * Functions used in the peripheral API
 */

#define WRITE 			0
#define READ			1

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t RorW);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);



/*
 * These functions are responsible for handling interrupt based on events
 */

static void I2C_SB_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_ADDR_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_BTF_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_STOPF_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_TXE_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_RXNE_interrupt_handle(I2C_Handle_t *pI2CHandle);
_weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

/*
 * Functions definitions for interrupt handlers
 */

static void I2C_SB_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX)
	{
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITE);
	}
	else
	{
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, READ);
	}
}
static void I2C_ADDR_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
	// Check if the device is in slave or master mode
	if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_MSL))
	{
		// Device is in Master Mode
		// Check if it busy on transmission or reception
		if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				// Disable the Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Clear the ADDR flag
				I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
			}
		}
		else
		{
			// In this case the master is transmitting, so it's not responsible for sending ACK
			// Clear the ADDR flag
			I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		}
	}
	else
	{
		// Device is in Slave Mode
		// Clear the ADDR flag to prevent clock stretching from the slave
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}
}
static void I2C_BTF_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX)
	{
		// Make sure that TXE is also set
		if (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE))
		{
			// BTF = 1, TXE = 1 , the transmission is done
			// Test to verify that the length is ZERO
			if (pI2CHandle->TxLen == 0)
			{
				//1. generate the STOP Condition
				if (pI2CHandle->SR == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				//2. reset the members elements of Handle structure
				I2C_CloseSendData(pI2CHandle);

				//3. Notify the Application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_COMPLETE);
			}

		}
	}
	else if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX)
	{
		return;
	}
}
static void I2C_STOPF_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
	// To clear the STOPF bit, we have to read the SR1 register, than write to CR2 register
	uint32_t temp = pI2CHandle->pI2Cx->I2C_SR1;
	pI2CHandle->pI2Cx->I2C_CR2 |= 0x0000;
	(void)temp;

	// Inform the application that slave detects STOP condition
	I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);
}
static void I2C_TXE_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
	// First we have the check the device is master
	if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX)
		{
			if (pI2CHandle->TxLen > 0)
			{
				//1. Load the data into DR
				pI2CHandle->pI2Cx->I2C_DR = * (pI2CHandle->pTxBuffer);

				//2. Decrement the TxLen
				pI2CHandle->TxLen --;

				//3. Increment the buffer address
				pI2CHandle->pTxBuffer ++;
			}
		}
	}
	else
	{
		// Slave Mode
		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
		{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_REQ);
		}

	}
}
static void I2C_RXNE_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
	if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_MSL))
	{
		if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				//1. Load the data from the data register
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

				//2. Decrement the RxLen
				pI2CHandle->RxLen --;

			}

			if (pI2CHandle->RxSize > 1)
			{
				// Verify if the RxSize is 2 to disable the acking, so when the data will be received
				// the ack will not be sent, so the transmitter will generate stop condition
				if (pI2CHandle->RxSize == 2)
				{
					// Clear the Ack bit
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				}
				//1. Load the data from the data register
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

				//2. Decrement the RxLen
				pI2CHandle->RxLen--;

				//3. Increment the buffer address
				pI2CHandle->pRxBuffer ++;
			}
			if (pI2CHandle->RxLen == 0)
			{
				// Close the I2C data reception and notif the APP

				// 1. Generate the STOP Condition
				if (pI2CHandle->SR == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				// 2. Close the reception
				I2C_CloseReceiveData(pI2CHandle);

				// 3. Notify the APP
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_COMPLETE);
			}
		}
	}
	else
	{
		// Slave Mode
		if (! (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
		{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_RCV);
		}
	}

}

/*
 * These functions are responsible for handling errors generated while communication is happening
 */

static void I2C_BERR_error_handle(I2C_Handle_t *pI2CHandle);
static void I2C_ARLO_error_handle(I2C_Handle_t *pI2CHandle);
static void I2C_AF_error_handle(I2C_Handle_t *pI2CHandle);
static void I2C_OVR_error_handle(I2C_Handle_t *pI2CHandle);
static void I2C_PECERR_error_handle(I2C_Handle_t *pI2CHandle);
static void I2C_TIMEOUT_error_handle(I2C_Handle_t *pI2CHandle);

/*
 * Functions definition for error handlers
 */

static void I2C_BERR_error_handle(I2C_Handle_t *pI2CHandle)
{
	//This is Bus error

	// clear the buss error flag
	pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

	//notify the application about the error
   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);

}
static void I2C_ARLO_error_handle(I2C_Handle_t *pI2CHandle)
{
	//This is arbitration lost error

	//the arbitration lost error flag
	pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_ARLO);

	//notify the application about the error
	I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
}
static void I2C_AF_error_handle(I2C_Handle_t *pI2CHandle)
{
    // This is ACK failure error

	//clear the ACK failure error flag
	pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_AF);

	//notify the application about the error
	I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
}
static void I2C_OVR_error_handle(I2C_Handle_t *pI2CHandle)
{
	//This is Overrun/under-run

    //clear the Overrun/under-run error flag
	pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_OVR);

	//notify the application about the error
	I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
}

static void I2C_PECERR_error_handle(I2C_Handle_t *pI2CHandle)
{
	//This is Parity Error Check in reception

    //clear the PEC Error flag
	pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_PECERR);

	//notify the application about the error
	I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_PECERR);
}

static void I2C_TIMEOUT_error_handle(I2C_Handle_t *pI2CHandle)
{
	//This is Time out error

    //Implement the code to clear the Time out error flag
	pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_TIMEOUT);

	//Implement the code to notify the application about the error
	I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
}

/*
 * Functions definition
 */

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr , uint8_t RorW)
{
	SlaveAddr |= (SlaveAddr << I2C_OAR1_ADD71);
	if (RorW == WRITE)
	{
		SlaveAddr &= ~(0x1);
	}
	else
	{
		SlaveAddr |= 0x1;
	}
	SlaveAddr &= ~(1);
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = 0;
	dummyRead = pI2Cx->I2C_SR1;
	dummyRead = pI2Cx->I2C_SR2;
	(void)dummyRead;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset the content of I2C Handle structure
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->TxRxState = I2C_STATE_READY;
	pI2CHandle->RxSize = 0;

	// enable the acking
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset the content of I2C Handle structure
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_STATE_READY;

}

/*
 * peripheral clock setup
 */


/********************************************************
 * @ fn 			- I2C_PeriClkControl
 *
 * @brief			- This function is responsible for Enabling or Disabling peripheral clock for a given I2C peripheral
 *
 * @param[0]		- Base address of I2C peripheral
 * @param[1]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if(ENorDI == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}

/*
 * Init and De-init
 */

/********************************************************
 * @ fn 			- I2C_INIT
 *
 * @brief			- This function initialize the I2C peripheral and set its configuration
 *
 * @param[1]		- Base address of I2C Handle variable

 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandlex)
{
	uint32_t tempreg = 0;

	// ENABLE THE CLOCK FOR THE PERIPHERAL
	I2C_PeriClkControl(pI2CHandlex->pI2Cx, ENABLE);

	// 1. Enable the ACK to be send on the bus
	tempreg |= ( pI2CHandlex->I2C_Config.I2C_ACKControl << I2C_CR1_ACK );
	pI2CHandlex->pI2Cx->I2C_CR1 = tempreg;

	// 2. Configure the FREQ field of the CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / One_MHZ);
	pI2CHandlex->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	// 3. program the device own address
	tempreg = 0;
	tempreg |= (pI2CHandlex->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD71);
	tempreg |= (1 << 14); // in the reference manual it is specified that this bit must be set to zero by the software
	pI2CHandlex->pI2Cx->I2C_OAR1 |= tempreg;

	// 4. Configure the Clock control register
	uint16_t ccr_value = 0;
	uint32_t Uclk_speed = pI2CHandlex->I2C_Config.I2C_SCLSpeed; // Clock speed specified by the user
	uint16_t UdutyCycle = pI2CHandlex->I2C_Config.I2C_FMDutyCycle; // Duty cycle specified by the user
	tempreg = 0;
	if (Uclk_speed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard mode
		// Apply the formula on the reference manual to get the value CCR
		ccr_value = (RCC_GetPCLK1Value() / (2 * Uclk_speed) );
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// Mode is fast mode
		// 4.1 Specify Fast mode on the CCR register
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (UdutyCycle << I2C_CCR_DUTY);

		// 4.2 Check the duty cycle chosen by the user to apply the coordinate formula
		if (UdutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * Uclk_speed) );
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * Uclk_speed) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandlex->pI2Cx->I2C_CCR = tempreg;

	//TRISE register configuration
	if (Uclk_speed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard
		tempreg = (RCC_GetPCLK1Value() / One_MHZ) + 1;
	}
	else
	{
		// Mode is Fast
		tempreg = ((RCC_GetPCLK1Value() * 300) / One_GHZ) + 1;
	}
	pI2CHandlex->pI2Cx->I2C_TRISE |= (tempreg & 0x3F);

}

/********************************************************
 * @ fn 			- I2C_Dinit
 *
 * @brief			- This function reset the I2C peripheral and its registers
 *
 * @param[1]		- Base address of I2C Handle variable

 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void I2C_Dinit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/*
 * Data Send and Receive
 */

/********************************************************
 * @ fn 			- I2C_GetFlagStatus
 *
 * @brief			- This function return the value of the flags in the I2C status register
 *
 * @param[1]		- Base address of I2C config variable
 * @param[2]		- FLAG_NAME

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t * pI2Cx, uint32_t FlagName)
{
	if ((pI2Cx -> I2C_SR1 & FlagName) || (pI2Cx -> I2C_SR2 & FlagName)) {
		return FLAG_SET;
	}
	else{
		return FLAG_RESET;
	}
}

/********************************************************
 * @ fn 			- I2C_MasterSendData
 *
 * @brief			- This function send on the I2C bus the data pointed by pTxBuffer
 * 					  to the slave who's address matches the The one specified by the caller.
 *
 * @param[1]		- Base address of I2C Handle variable
 * @param[2]		- pointer to the data to be send
 * @param[3]		- Length of the data to be send
 * @param[4]		- Slave Address
 * @param[5]		- Enable or disable repeated start

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr , uint8_t SR)
{
	// 1. Generate the Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flage in the SR1
	// Note : while SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with R/nW bit set to W(0) (Total 1byte)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, WRITE);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear the ADDR flag according to its software sequence
	// Note : while ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send the data until Length reaches 0
	while (Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer ++;
		Len--;
	}

	// 7. When Length becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note : TXE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition
	// Note : Generating STOP, automatically clears the BTF
	if (SR == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

/********************************************************
 * @ fn 			- I2C_MasterReceiveData
 *
 * @brief			- This function receive the data on the I2C bus sent by the slave
 *  			      who's address matches the The one specified by the caller.
 *
 * @param[1]		- Base address of I2C Handle variable
 * @param[2]		- pointer to where the data will be stored
 * @param[3]		- Length of the data to be received
 * @param[4]		- Slave Address
 * @param[5]		- Enable or disable repeated start

 * @return 			- none
 * @note 			- none
 *
 *******************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB Flag in the SR1
	// Note : while SB is cleared SCL will be stretched (Pulled to low)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the R/nW bit set to R(1) (Total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, READ);

	// 4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// Disable Acknowledging
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// wait until RXNE Becomes 0
		while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// generate STOP condition
		if (SR == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

	}

	if (Len > 1)
	{
		//Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Read the data until Len becomes Zero
		for(uint32_t i = Len ; i > 0 ;i--)
		{
			//wait until RXNE becomes 1
			while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2)
			{
				// Disable Acknowledging
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// generate STOP condition
				if (SR == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			// read data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			// increment the buffer address
			pRxBuffer++;

		}
	}
	// re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE );
	}
}

/********************************************************
 * @ fn 			- I2C_MasterSendDataIT
 *
 * @brief			- This function send on the I2C bus the data pointed by pTxBuffer
 * 					  to the slave who's address matches the The one specified by the caller,
 * 					  only when an interrupt is triggered
 *
 * @param[1]		- Base address of I2C Handle variable
 * @param[2]		- pointer to the data to be send
 * @param[3]		- Length of the data to be send
 * @param[4]		- Slave Address
 * @param[5]		- Enable or disable repeated start

 * @return 			- Application State
 *
 * @note 			- none
 *
 *******************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_STATE_BUSY_IN_TX) && (busystate != I2C_STATE_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->SR = SR;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/********************************************************
 * @ fn 			- I2C_MasterReceiveData
 *
 * @brief			- This function receive the data on the I2C bus sent by the slave
 *  			      who's address matches the The one specified by the caller.
 *					  only when an interrupt is triggered
 *
 * @param[1]		- Base address of I2C Handle variable
 * @param[2]		- pointer to where the data will be stored
 * @param[3]		- Length of the data to be received
 * @param[4]		- Slave Address
 * @param[5]		- Enable or disable repeated start

 * @return 			- Application State
 * @note 			- none
 *
 *******************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_STATE_BUSY_IN_TX) && (busystate != I2C_STATE_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR cod to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->SR = SR;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |=(1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |=(1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |=(1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

/*
 * Slave Send/Receive Data
 */

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx )
{
	return (uint8_t) pI2Cx->I2C_DR;
}

/*
 * IRQ configuration and ISR handling
 */

/********************************************************
 * @ fn 			- I2C_IrqInterruptConfig
 *
 * @brief			- This function Set configurations to enable or disable interrupt from I2C device.
 *
 * @param[1]		- The Interrupt request number
 * @param[2]		- ENABLE or DISABLE

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void I2C_IrqInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
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
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//set ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) {
			//set ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/********************************************************
 * @ fn 			- I2C_IRQ_PriorityConfig
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

void I2C_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NB_PR_BIT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) = (IRQPriority << shift_amount);
}

/********************************************************
 * @ fn 			- I2C_IrqHandling
 *
 * @brief			- This function Handle the interrupt event when it is Triggered
 *
 * @param[1]		- Base address of I2C Handle variable

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device
	uint32_t SR1_REG = pI2CHandle->pI2Cx->I2C_SR1;
	uint32_t CR2_REG = pI2CHandle->pI2Cx->I2C_CR2;
	uint32_t temp[8] = { CR2_REG & (1 << I2C_CR2_ITEVTEN),
						 CR2_REG & (1 << I2C_CR2_ITBUFEN),
						 SR1_REG & (1 << I2C_SR1_SB),
						 SR1_REG & (1 << I2C_SR1_ADDR),
						 SR1_REG & (1 << I2C_SR1_BTF),
						 SR1_REG & (1 << I2C_SR1_STOPF),
						 SR1_REG & (1 << I2C_SR1_TXE),
						 SR1_REG & (1 << I2C_SR1_RXNE)
						};

	// 1. Handle for interrupt generated by SB event
	// Note : SB flag is only applicable on master mode

	if (temp[0] & temp[2])
	{
		// The interrupt generated because of SB event
		// This block will not be executed in slave mode because for slave SB is always zero
		I2C_SB_interrupt_handle(pI2CHandle);
	}

	// 2. Handle for interrupt generated by ADDR event
	// Note : when master mode : Address is sent
	// 		  when Slave mode  : Address matched with own address

	if (temp[0] & temp[3])
	{
		// The interrupt generated because of ADDR event
		I2C_ADDR_interrupt_handle(pI2CHandle);
	}

	// 3. Handle For interrupt generated by BTF (Byte transfer Finished) event

	if (temp[0] & temp[4])
	{
		// BTF flag is set
		I2C_BTF_interrupt_handle(pI2CHandle);
	}

	// 4. Handle for interrupt generated by STOPF event
	// Note : STOP detection flag is applicable only slave mode. For master this flag will

	if (temp[0] & temp[5])
	{
		// STOPF flag is set, indicate that a slave detects STOP condition
		I2C_STOPF_interrupt_handle(pI2CHandle);
	}

	// 5. Handle For interrupt generated by TXE event

	if (temp[0] & temp[1] & temp[6])
	{
		// TXE flag is set, so we have to send the data
		I2C_TXE_interrupt_handle(pI2CHandle);
	}

	// 6. Handle for interrupt generated by RXNE event

	if (temp[0] & temp[1] & temp[7])
	{
		// RXNE flag is set, we have to receive the data
		I2C_RXNE_interrupt_handle(pI2CHandle);
	}
}
/********************************************************
 * @ fn 			- I2C_IrqHandling
 *
 * @brief			- This function Handle the interrupt error when it is Triggered
 *
 * @param[1]		- Base address of I2C Handle variable

 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device
	uint32_t SR1_REG = pI2CHandle->pI2Cx->I2C_SR1;
	uint32_t CR2_REG = pI2CHandle->pI2Cx->I2C_CR2;

	uint32_t temp[7] = { CR2_REG & (1 << I2C_CR2_ITERREN),
						 SR1_REG & (1 << I2C_SR1_BERR),
						 SR1_REG & (1 << I2C_SR1_ARLO),
						 SR1_REG & (1 << I2C_SR1_AF),
						 SR1_REG & (1 << I2C_SR1_OVR),
						 SR1_REG & (1 << I2C_SR1_PECERR),
						 SR1_REG & (1 << I2C_SR1_TIMEOUT)
						};

	// 1. Handle for interrupt generated by BERR error
	// Note :

	if (temp[0] & temp[1])
	{
		// BERR flag is set
		I2C_BERR_error_handle(pI2CHandle);
	}

	// 2. Handle for interrupt error generated by ARLO event
	//

	if (temp[0] & temp[2])
	{
		// ARLO flag is set
		I2C_ARLO_error_handle(pI2CHandle);
	}

	// 3. Handle For interrupt generated by AF error

	if (temp[0] & temp[3])
	{
		// AF flag is set
		I2C_AF_error_handle(pI2CHandle);
	}

	// 4. Handle for interrupt generated by OVR Error
	// Note :

	if (temp[0] & temp[4])
	{
		// OVR flag is set
		I2C_OVR_error_handle(pI2CHandle);
	}

	// 5. Handle For interrupt generated by PECERR Error

	if (temp[0] & temp[5])
	{
		// PECERR flag is set
		I2C_PECERR_error_handle(pI2CHandle);
	}

	// 6. Handle for interrupt generated by TIMEOUT error

	if (temp[0] & temp[6])
	{
		// TIMEOUT flag is set
		I2C_TIMEOUT_error_handle(pI2CHandle);
	}
}

_weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	//this is a weak definition of the function , it must be defined by the user
}

/*
 * Other Peripheral Control APIs
 */


/********************************************************
 * @ fn 			- I2C_PeripheralControl
 *
 * @brief			- This function is responsible for Enabling or Disabling peripheral.
 *
 * @param[1]		- Base address of I2C peripheral
 * @param[2]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/********************************************************
 * @ fn 			- I2C_ManageAcking
 *
 * @brief			- This function is responsible for Enabling or Disabling Acknowledgment.
 *
 * @param[1]		- Base address of I2C peripheral
 * @param[2]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	 if (ENorDI == I2C_ACK_ENABLE)
	 {
		 // Enable ACK
		 pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	 }
	 else
	 {
		 // Disable ACK
		 pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	 }
}

/********************************************************
 * @ fn 			- I2C_SlaveControlCallBackEvents
 *
 * @brief			- This function is responsible for Enabling or Disabling events field the status register
 * 					  so we can receive those events.
 *
 * @param[1]		- Base address of I2C peripheral
 * @param[2]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void I2C_SlaveControlCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	}

}

