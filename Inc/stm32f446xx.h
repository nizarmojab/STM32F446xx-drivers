
#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#include <stdint.h>
#include <stddef.h>

#define _weak __attribute__((weak))

/*************************************************START: PROCESSOR Specific Details***************************************************/
/*
 *ARM CORTEX M4 PROCESSOR NVIC ISERx register Addresses
 */

#define NVIC_ISER0		((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*) 0xE000E10C)

/*
 *ARM CORTEX M4 PROCESSOR NVIC ICERx register Addresses
 */

#define NVIC_ICER0		((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*) 0xE000E18C)

/*
 *ARM CORTEX M4 PROCESSOR NVIC Priority register Address
 */
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*) 0xE000E400)

#define NB_PR_BIT_IMPLEMENTED	4

/*************************************************START: PROCESSOR Specific Details***************************************************/
/*
* Base addresses of Flash and SRAM Memory
*/

#define FLASH_BASEADDR			0x08000000U  /*Givi ng first address of flash memory alias of Flash_BaseAddr */
#define SRAM1_BASEADDR			0x20000000U	 /*Giving first address of SRAM1 memory alias of SRAM1_BaseAddr */
#define SRAM2_BASEADDR			0x2001C000U	 /*Giving first address of SRAM2 memory alias of SRAM2_BaseAddr */
#define ROM_BASEADDR			0x1FFF0000U	 /*Giving first address of ROM memory alias ROM_BaseAddr */
#define SRAM					SRAM1_BASEADDR /*Giving SRAM1_BASEADDR the alias of SRAM*/

/*
* AHBx and APBx Base addresses
*/

#define PERIPH_BASEADDR				0x40000000U	/*Giving first peripherals address alias of PERIPH_BaseAddr */
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR	/*Giving base memory address of peripherals connected to APB1 alias of APB1PERIPH_BaseAddr */
#define APB2PERIPH_BASEADDR			0x40010000U /*Giving base memory address of peripherals connected to APB2 alias of APB2PERIPH_BaseAddr */
#define AHB1PERIPH_BASEADDR			0x40020000U	/*Giving base memory address of peripherals connected to AHB1 alias of AHB1PERIPH_BaseAddr */
#define AHB2PERIPH_BASEADDR			0x50000000U /*Giving base memory address of peripherals connected to AHB2 alias of AHB2PERIPH_BaseAddr */

/*
* BASE ADRRESSES OF PERIPHERALS THAT ARE HANGING ON AHB1 BUS
*/

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000) /*Defining the address of GPIOA as the base address of peripherals connected to AHB1 plus the offset related to GPIOA*/
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400) /*Defining the address of GPIOB as the base address of peripherals connected to AHB1 plus the offset related to GPIOB*/
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800) /*Defining the address of GPIOC as the base address of peripherals connected to AHB1 plus the offset related to GPIOC*/
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00) /*Defining the address of GPIOD as the base address of peripherals connected to AHB1 plus the offset related to GPIOD*/
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000) /*Defining the address of GPIOE as the base address of peripherals connected to AHB1 plus the offset related to GPIOE*/
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400) /*Defining the address of GPIOF as the base address of peripherals connected to AHB1 plus the offset related to GPIOF*/
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800) /*Defining the address of GPIOG as the base address of peripherals connected to AHB1 plus the offset related to GPIOG*/
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00) /*Defining the address of GPIOH as the base address of peripherals connected to AHB1 plus the offset related to GPIOH*/
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800) /*Defining the address of RCC as the base address of peripherals connected to AHB1 plus the offset related to RCC */

/*
* BASE ADRRESSES OF PERIPHERALS THAT ARE HANGING ON APB1 BUS
*/

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800) /*Defining the address of SPI2 as the base address of peripherals connected to APB1 plus the offset related to SPI2*/
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00) /*Defining the address of SPI3 as the base address of peripherals connected to APB1 plus the offset related to SPI3*/

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400) /*Defining the address of USART2 as the base address of peripherals connected to APB1 plus the offset related to USART2*/
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800) /*Defining the address of USART3 as the base address of peripherals connected to APB1 plus the offset related to USART3*/

#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00) /*Defining the address of UART4 as the base address of peripherals connected to APB1 plus the offset related to UART4*/
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000) /*Defining the address of UART5 as the base address of peripherals connected to APB1 plus the offset related to UART5*/

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400) /*Defining the address of I2C1 as the base address of peripherals connected to APB1 plus the offset related to I2C1*/
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800) /*Defining the address of I2C2 as the base address of peripherals connected to APB1 plus the offset related to I2C2*/
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00) /*Defining the address of I2C3 as the base address of peripherals connected to APB1 plus the offset related to I2C3*/


/*
* BASE ADRRESSES OF PERIPHERALS THAT ARE HANGING ON APB2 BUS
*/

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00) /*Defining the address of EXTI as the base address of peripherals connected to APB2 plus the offset related to EXTI*/
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000) /*Defining the address of SPI1 as the base address of peripherals connected to APB2 plus the offset related to SPI1*/
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400) /*Defining the address of SPI4 as the base address of peripherals connected to APB2 plus the offset related to SPI4*/
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800) /*Defining the address of SYSCFG as the base address of peripherals connected to APB2 plus the offset related to SYSCFG*/
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000) /*Defining the address of USART1 as the base address of peripherals connected to APB2 plus the offset related to USART1*/
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400) /*Defining the address of USART6 as the base address of peripherals connected to APB2 plus the offset related to USART6*/




/*****************************************    PERIPHERAL REGISTER DEFINITION STRUCTURES    *******************************************/
/*************************************************************************************************************************************/
/*
	NOTE : REGISTERS OF A PERIPHERAL ARE SPECIFIC TO MCU
*/

/*
* Peripheral register definition structure for GPIO
*/

typedef struct {
	volatile uint32_t MODER;   /* GPIO port mode register  ; Offset from Base GPIO ADDRESS IS : 0x00*/
	volatile uint32_t OTYPER;  /* GPIO port output type register ; Offset from Base GPIO ADDRESS IS :  0x04*/
	volatile uint32_t OSPEEDR; /* GPIO port output speed register ; Offset from Base GPIO ADDRESS IS : 0x08*/
	volatile uint32_t PUPDR;   /* GPIO port pull-up/pull-down register ; Offset from Base GPIO ADDRESS IS : 0x0C*/
	volatile uint32_t IDR;     /* GPIO port input data register ; Offset from Base GPIO ADDRESS IS : 0x10*/
	volatile uint32_t ODR;     /* GPIO port output data register ; Offset from Base GPIO ADDRESS IS : 0x14*/
	volatile uint32_t BSRR;    /* GPIO port bit set/reset register ; Offset from Base GPIO ADDRESS IS : 0x18*/
	volatile uint32_t LCKR;    /* GPIO port configuration lock register ; Offset from Base GPIO ADDRESS IS : 0x1C */
	volatile uint32_t AFR[2];  /* GPIO alternate function low register AFR[0] ; GPIO alternate function high register AFR[1]; AFRL Offset from Base GPIO ADDRESS IS : 0x20 ; AFRH Offset from Base GPIO ADDRESS IS : 0x24 */
}GPIO_RegDef_t;

/*
* Peripheral register definition structure for RCC
*/

typedef struct {
	volatile uint32_t CR;         /* RCC clock control register             						;Offset from Base RCC REG ADDRESS IS : 0x00 */
	volatile uint32_t PLLCFGR;    /* RCC PLL configuration register									;Offset from Base RCC REG ADDRESS IS : 0x04 */
	volatile uint32_t CFGR;       /* RCC clock configuration register       						;Offset from Base RCC REG ADDRESS IS : 0x08 */
	volatile uint32_t CIR;        /* RCC clock interrupt register									;Offset from Base RCC REG ADDRESS IS : 0x0C */
	volatile uint32_t AHB1RSTR;   /* RCC AHB1 peripheral reset register								;Offset from Base RCC REG ADDRESS IS : 0x10 */
	volatile uint32_t AHB2RSTR;   /* RCC AHB2 peripheral reset register								;Offset from Base RCC REG ADDRESS IS : 0x14 */
	volatile uint32_t AHB3RSTR;   /* RCC AHB3 peripheral reset register								;Offset from Base RCC REG ADDRESS IS : 0x18 */
	volatile uint32_t RESERVED0	; /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x1C*/
	volatile uint32_t APB1RSTR;   /* RCC APB1 peripheral reset register								;Offset from Base RCC REG ADDRESS IS : 0x20 */
	volatile uint32_t APB2RSTR;   /* RCC APB2 peripheral reset register		   						;Offset from Base RCC REG ADDRESS IS : 0x24 */
	volatile uint32_t RESERVED1;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x28*/
	volatile uint32_t RESERVED2;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x2C*/
	volatile uint32_t AHB1ENR;    /* RCC AHB1 peripheral clock enable register  					;Offset from Base RCC REG ADDRESS IS : 0x30 */
	volatile uint32_t AHB2ENR;    /* RCC AHB2 peripheral clock enable register						;Offset from Base RCC REG ADDRESS IS : 0x34 */
	volatile uint32_t AHB3ENR;    /* RCC AHB3 peripheral clock enable register						;Offset from Base RCC REG ADDRESS IS : 0x38 */
	volatile uint32_t RESERVED3;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x3C*/
	volatile uint32_t APB1ENR;    /* RCC APB1 peripheral clock enable register						;Offset from Base RCC REG ADDRESS IS : 0x40 */
	volatile uint32_t APB2ENR;    /* RCC APB2 peripheral clock enable register						;Offset from Base RCC REG ADDRESS IS : 0x44 */
	volatile uint32_t RESERVED4;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x48*/
	volatile uint32_t RESERVED5;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x4C*/
	volatile uint32_t AHB1LPENR;  /* RCC AHB1 peripheral clock enable in low power mode register	;Offset from Base RCC REG ADDRESS IS : 0x50 */
	volatile uint32_t AHB2LPENR;  /* RCC AHB2 peripheral clock enable in low power mode register	;Offset from Base RCC REG ADDRESS IS : 0x54 */
	volatile uint32_t AHB3LPENR;  /* RCC AHB3 peripheral clock enable in low power mode register	;Offset from Base RCC REG ADDRESS IS : 0x58 */
	volatile uint32_t RESERVED6;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x5C*/
	volatile uint32_t APB1LPENR;  /* RCC APB1 peripheral clock enable in low power mode register	;Offset from Base RCC REG ADDRESS IS : 0x60 */
	volatile uint32_t APB2LPENR;  /* RCC APB2 peripheral clock enable in low power mode register	;Offset from Base RCC REG ADDRESS IS : 0x64 */
	volatile uint32_t RESERVED7;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x68*/
	volatile uint32_t RESERVED8;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x6C*/
	volatile uint32_t BDCR;       /* RCC Backup domain control register								;Offset from Base RCC REG ADDRESS IS : 0x70 */
	volatile uint32_t CSR;        /* RCC clock control & status register							;Offset from Base RCC REG ADDRESS IS : 0x74 */
	volatile uint32_t RESERVED9;  /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x78*/
	volatile uint32_t RESERVED10; /* RESERVED REGISTER												;Offset from Base RCC REG ADDRESS IS : 0x7C*/
	volatile uint32_t SSCGR;      /* RCC spread spectrum clock generation register					;Offset from Base RCC REG ADDRESS IS : 0x80 */
	volatile uint32_t PLLI2SCFGR; /* RCC PLLI2S configuration register								;Offset from Base RCC REG ADDRESS IS : 0x84 */
	volatile uint32_t PLLSAICFGR; /* RCC PLL configuration register									;Offset from Base RCC REG ADDRESS IS : 0x88 */
	volatile uint32_t DCKCFGR;    /* RCC dedicated clock configuration register						;Offset from Base RCC REG ADDRESS IS : 0x8C */
	volatile uint32_t CKGATENR;	  /* RCC clocks gated enable register								;Offset from Base RCC REG ADDRESS IS : 0x90 */
	volatile uint32_t DCKCFGR2;   /* RCC dedicated clocks configuration register 2					;Offset from Base RCC REG ADDRESS IS : 0x94 */
}RCC_RegDef_t;

/*
* Peripheral register definition structure for EXTI
*/

typedef struct{
	volatile uint32_t IMR; 		  /* Interrupt mask register										;Offset from Base EXTI REG ADDRESS IS : 0x00*/
	volatile uint32_t EMR; 		  /* Event mask register 											;Offset from Base EXTI REG ADDRESS IS : 0x04*/
	volatile uint32_t RTSR; 	  /* Rising trigger selection register								;Offset from Base EXTI REG ADDRESS IS : 0x08*/
	volatile uint32_t FTSR; 	  /* Falling trigger selection register								;Offset from Base EXTI REG ADDRESS IS : 0x0C*/
	volatile uint32_t SWIER; 	  /* Software interrupt event register								;Offset from Base EXTI REG ADDRESS IS : 0x10*/
	volatile uint32_t PR; 		  /* Pending register												;Offset from Base EXTI REG ADDRESS IS : 0x14*/
}EXTI_RegDef_t;

/*
* Peripheral register definition structure for SYSCFG
*/

typedef struct{
	volatile uint32_t MEMRMP; 	  /* SYSCFG memory remap register									;Offset from Base SYSCFG register ADDRESS IS : 0x00*/
	volatile uint32_t PMC; 		  /* SYSCFG peripheral mode configuration register					;Offset from Base SYSCFG register ADDRESS IS : 0x04*/
	volatile uint32_t EXTICR[4];  /* SYSCFG external interrupt configuration register 1-4			;Offset from Base SYSCFG register ADDRESS IS : 0x08-0x14*/
	volatile uint32_t RESERVED1;  /* RESERVED Register 												;Offset from Base SYSCFG register ADDRESS IS : 0X18*/
	volatile uint32_t RESERVED2;  /* RESERVED Register 												;Offset from Base SYSCFG register ADDRESS IS : 0X1C*/
	volatile uint32_t CMPCR; 	  /* Compensation cell control register								;Offset from Base SYSCFG register ADDRESS IS : 0x20*/
	volatile uint32_t RESERVED3;  /* RESERVED Register 												;Offset from Base SYSCFG register ADDRESS IS : 0X24*/
	volatile uint32_t RESERVED4;  /* RESERVED Register 												;Offset from Base SYSCFG register ADDRESS IS : 0X28*/
	volatile uint32_t CFGR; 	  /* SYSCFG configuration register									;Offset from Base SYSCFG register ADDRESS IS : 0x2C*/
}SYSCFG_RegDef_t;


/*
 * This is Peripheral register definition structure for a SPIx peripheral
 */

typedef struct {
	volatile uint32_t SPI_CR1;	  /* SPI control register 1 	  	 (SPI_CR1) 						;Offset from Base SPI register ADDRESS IS : 0x00*/
	volatile uint32_t SPI_CR2;	  /* SPI control register 2 	 	 (SPI_CR2)						;Offset from Base SPI register ADDRESS IS : 0x04*/
	volatile uint32_t SPI_SR;	  /* SPI status register 		   	 (SPI_SR)  						;Offset from Base SPI register ADDRESS IS : 0x08*/
	volatile uint32_t SPI_DR;	  /* SPI data register 			  	 (SPI_DR)						;Offset from Base SPI register ADDRESS IS : 0x0C*/
	volatile uint32_t SPI_CRCPR;  /* SPI CRC polynomial register 	 (SPI_CRCPR)					;Offset from Base SPI register ADDRESS IS : 0x10*/
	volatile uint32_t SPI_RXCRCR; /* SPI RX CRC register 		 	 (SPI_RXCRC						;Offset from Base SPI register ADDRESS IS : 0x14*/
	volatile uint32_t SPI_TXCRCR; /* SPI TX CRC register 		 	 (SPI_TXCRCR)					;Offset from Base SPI register ADDRESS IS : 0x18*/
	volatile uint32_t SPI_I2SCFGR;/* SPI_I2 S configuration register (SPI_I2SCFGR)					;Offset from Base SPI register ADDRESS IS : 0x1C*/
	volatile uint32_t SPI_I2SPR;  /* SPI_I2 S prescaler register 	 (SPI_I2SPR)					;Offset from Base SPI register ADDRESS IS : 0x20*/

}SPI_RegDef_t;

/*
 * This is Peripheral register definition structure for a I2Cx peripheral
 */

typedef struct {
	volatile uint32_t I2C_CR1;	  /* I2C control register 1 		(I2C_CR1) 						;Offset from Base I2C register ADDRESS IS : 0x00*/
	volatile uint32_t I2C_CR2;	  /* I2C control register 2 		(I2C_CR2) 						;Offset from Base I2C register ADDRESS IS : 0x04*/
	volatile uint32_t I2C_OAR1;   /* I2C own address register 1		(I2C_OAR1) 						;Offset from Base I2C register ADDRESS IS : 0x08*/
	volatile uint32_t I2C_OAR2;	  /* I2C own address register 2		(I2C_OAR2) 						;Offset from Base I2C register ADDRESS IS : 0x0C*/
	volatile uint32_t I2C_DR;	  /* I2C data register 				(I2C_DR) 						;Offset from Base I2C register ADDRESS IS : 0x10*/
	volatile uint32_t I2C_SR1;	  /* I2C status register 1			(I2C_SR1) 						;Offset from Base I2C register ADDRESS IS : 0x14*/
	volatile uint32_t I2C_SR2;	  /* I2C status register 2			(I2C_SR2) 						;Offset from Base I2C register ADDRESS IS : 0x18*/
	volatile uint32_t I2C_CCR;	  /* I2C clock control register 1	(I2C_CCR) 						;Offset from Base I2C register ADDRESS IS : 0x1C*/
	volatile uint32_t I2C_TRISE;  /* I2C TRISE	register			(I2C_TRISE) 					;Offset from Base I2C register ADDRESS IS : 0x20*/
	volatile uint32_t I2C_FLTR;   /* I2C FLTR register 1			(I2C_FLTR) 						;Offset from Base I2C register ADDRESS IS : 0x24*/
}I2C_RegDef_t;

/*
 * This is Peripheral register definition structure for a I2Cx peripheral
 */

typedef struct {
	volatile uint32_t USART_SR;	  /* USART Status register*/
	volatile uint32_t USART_DR;	  /* USART DATA register  */
	volatile uint32_t USART_BRR;  /* USART Baud Rate register*/
	volatile uint32_t USART_CR1;  /* USART Control register 1*/
	volatile uint32_t USART_CR2;  /* USART Control register 2*/
	volatile uint32_t USART_CR3;  /* USART Control register 3*/
	volatile uint32_t USART_GTPR; /* USART Guard time and pre-scaler register*/
}USART_RegDef_t;

/*
* Peripheral Definitions (Peripheral base Address type-casted to xxx_RegDef_t)
*/
//defining GPIOs PERIPHERAL
#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*) GPIOH_BASEADDR)

//defining SPI PERIPHERAL

#define SPI1			((SPI_RegDef_t*)  SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)  SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)  SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)  SPI4_BASEADDR)

//defining I2C PERIPHERAL

#define I2C1			((I2C_RegDef_t*)  I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)  I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)  I2C3_BASEADDR)

//defining USART PERIPHERAL

#define USART1			((USART_RegDef_t*)  USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)  USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)  USART3_BASEADDR)
#define UART4			((USART_RegDef_t*)  UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)  UART5_BASEADDR)
#define USART6			((USART_RegDef_t*)  USART6_BASEADDR)

//defining RCC PERIPHERAL
#define RCC				((RCC_RegDef_t*)  RCC_BASEADDR)

//defining EXTI interrupt peripheral
#define EXTI			((EXTI_RegDef_t*) EXTI_BASEADDR)

//defining SYSTEM CONFIGURATION peripheral
#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


/************************************************ CLOCK ENABLING FOR PERIPHERALS *****************************************************/
/*************************************************************************************************************************************/
/*
* CLOCK ENABLE MACROS FOR GPIOx peripherals
*/

#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1 << 7))

/*
* CLOCK ENABLE MACROS FOR I2Cx peripherals
*/

#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1 << 23))

/*
* CLOCK ENABLE MACROS FOR SPIx peripherals
*/

#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			( RCC->APB2ENR |= (1 << 13))

/*
* CLOCK ENABLE MACROS FOR USARTx peripherals
*/

#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			( RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			( RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 << 5))

/*
* CLOCK ENABLE MACROS FOR SYSCFGx peripherals
*/

#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14))

/************************************************ CLOCK DISABLING FOR PERIPHERALS ****************************************************/
/*************************************************************************************************************************************/


/*
* CLOCK DISABLE MACROS FOR GPIOx peripherals
*/

#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7))

/*
* CLOCK DISABLE MACROS FOR I2Cx peripherals
*/

#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23))

/*
* CLOCK DISABLE MACROS FOR SPIx peripherals
*/

#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21))
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22))
#define SPI4_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 13))

/*
* CLOCK DISABLE MACROS FOR USARTx peripherals
*/

#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 5))

/*
* CLOCK DISABLE MACROS FOR SYSCFGx peripherals
*/

#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 14))

/*
* MACROS to reset GPIOx Peripherals
*/

#define GPIOA_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 0)); ( RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 1)); ( RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 2)); ( RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 3)); ( RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 4)); ( RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 5)); ( RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 6)); ( RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET() 				do { ( RCC->AHB1RSTR |= (1 << 7)); ( RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
* MACROS to reset SPIx Peripherals
*/

#define SPI1_REG_RESET() 				do { ( RCC->APB2RSTR |= (1 << 12)); ( RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET() 				do { ( RCC->APB1RSTR |= (1 << 14)); ( RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET() 				do { ( RCC->APB1RSTR |= (1 << 15)); ( RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET() 				do { ( RCC->APB2RSTR |= (1 << 13)); ( RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
* MACROS to reset SPIx Peripherals
*/

#define I2C1_REG_RESET()				do { ( RCC->APB1RSTR |= (1 << 21)); ( RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()				do { ( RCC->APB1RSTR |= (1 << 22)); ( RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()				do { ( RCC->APB1RSTR |= (1 << 23)); ( RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/*
* MACROS to reset SPIx Peripherals
*/

#define USART1_REG_RESET()				do { ( RCC->APB2RSTR |= (1 << 4));  ( RCC->APB2RSTR &= ~(1 << 4));	}while(0)
#define USART2_REG_RESET()				do { ( RCC->APB2RSTR |= (1 << 17)); ( RCC->APB2RSTR &= ~(1 << 17));	}while(0)
#define USART3_REG_RESET()				do { ( RCC->APB2RSTR |= (1 << 18)); ( RCC->APB2RSTR &= ~(1 << 18));	}while(0)
#define UART4_REG_RESET()				do { ( RCC->APB2RSTR |= (1 << 19)); ( RCC->APB2RSTR &= ~(1 << 19));	}while(0)
#define UART5_REG_RESET()				do { ( RCC->APB2RSTR |= (1 << 20)); ( RCC->APB2RSTR &= ~(1 << 20));	}while(0)
#define USART6_REG_RESET()				do { ( RCC->APB2RSTR |= (1 << 5));  ( RCC->APB2RSTR &= ~(1 << 4));	}while(0)


/*
 * Macro function to convert GPIOx address to a code
 */

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)? 0 :\
										 (x == GPIOB)? 1 :\
										 (x == GPIOC)? 2 :\
										 (x == GPIOD)? 3 :\
										 (x == GPIOE)? 4 :\
										 (x == GPIOF)? 5 :\
										 (x == GPIOG)? 6 :0)

/*
 * IRQ (Interrupt Request ) Numbers of STM32F446 MCU
 */

//External interrupt IRQ number
#define IRQ_NO_EXI0			6
#define IRQ_NO_EXI1			7
#define IRQ_NO_EXI2			8
#define IRQ_NO_EXI3			9
#define IRQ_NO_EXI4			10
#define IRQ_NO_EXI9_5		23
#define IRQ_NO_EXI15_10		40

//SPI IRQ number
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

//I2C IRQ Number

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

//UART IRQ Number

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71

/*
* ALternate Function Macros
*/

#define ALTFN0					0
#define ALTFN1					1
#define ALTFN2					2
#define ALTFN3					3
#define ALTFN4					4
#define ALTFN5					5
#define ALTFN6					6
#define ALTFN7					7
#define ALTFN8					8
#define ALTFN9					9
#define ALTFN10					10
#define ALTFN11					11
#define ALTFN12					12
#define ALTFN13					13
#define ALTFN14					14
#define ALTFN15					15
/*
* Generic Macros
*/

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define AFR_PINS			8
#define FLAG_RESET			RESET
#define FLAG_SET			SET

// Bit Position Macros for RCC
#define RCC_CFGR_SWS		2
#define RCC_CFGR_HPRE		4
#define RCC_CFGR_PPRE1		10
#define RCC_CFGR_PPRE2		13

// Clock Sources values
#define HSI_Clock_Value		16000000U
#define HSE_Clock_Value		8000000U
#define One_MHZ				1000000U
#define One_GHZ				1000000000U



/*****************************************************************
 * ********** BIT Position Macros for SPI peripheral *************
 *****************************************************************/

// SPI_CR1 Register

#define SPI_CR1_CPHA 	  0
#define SPI_CR1_CPOL 	  1
#define SPI_CR1_MSTR 	  2
#define SPI_CR1_BR		  3
#define SPI_CR1_SPE  	  6
#define SPI_CR1_LSBFIRST  7
#define SPI_CR1_SSI 	  8
#define SPI_CR1_SSM 	  9
#define SPI_CR1_RXONLY	  10
#define SPI_CR1_DFF 	  11
#define SPI_CR1_CRCNEXT   12
#define SPI_CR1_CRCEN 	  13
#define SPI_CR1_BIDIOE 	  14
#define SPI_CR1_BIDIMODE  15

// SPI_CR2 Register

#define SPI_CR2_RXDMAEN   0 // Rx buffer DMA enable
#define SPI_CR2_TXDMAEN   1 // Tx buffer DMA enable
#define SPI_CR2_SSOE	  2 // SS output enable
#define SPI_CR2_FRF  	  4 // Frame format
#define SPI_CR2_ERRIE	  5 // Error interrupt enable
#define SPI_CR2_RXNEIE 	  6 // Rx buffer not empty interrupt enable
#define SPI_CR2_TXEIE 	  7 // Tx buffer empty interrupt enable


// SPI_SR Register

#define SPI_SR_RXNE 	  0 // Receive buffer not empty
#define SPI_SR_TXE	   	  1 // Transmit buffer empty
#define SPI_SR_CHSIDE  	  2 // Channel Side
#define SPI_SR_UDR  	  3 // underrun flag
#define SPI_SR_CRCERR	  4 // CRC error flag
#define SPI_SR_MODF 	  5 // Mode fault
#define SPI_SR_OVR	 	  6 // Overrun flag
#define SPI_SR_BSY	 	  7 // Busy flag
#define SPI_SR_FRE	 	  8 // Frame error

/*****************************************************************
 * ********** BIT Position Macros for I2C peripheral *************
 *****************************************************************/

//Control register 1

#define I2C_CR1_PE		  	0  // Peripheral Enable
#define I2C_CR1_SMBUS	  	1  // SMBus mode (0 : I2C Mode,1 : SMBus Mode)
#define I2C_CR1_SMBTYPE	  	3  // SMBTYPE (0 : SMBus Device,1 : SMBus Host)
#define I2C_CR1_ENARP	  	4  // ARP enable (0 : Disable,1 : ARP enable)
#define I2C_CR1_ENPEC	  	5  // Packet Error Checking Enable
#define I2C_CR1_ENGC	  	6  // General call enable
#define I2C_CR1_NOSTRETCH 	7  // Clock stretching
#define I2C_CR1_START	  	8  // Start generation
#define I2C_CR1_STOP	  	9  // Stop generation
#define I2C_CR1_ACK		  	10 // Acknowledge enable
#define I2C_CR1_POS		  	11 // Acknowledge/PEC Position for data reception
#define I2C_CR1_PEC		  	12 // Packet Error Checking (0 : no PEC transfer, 1 :  PEC transfer)
#define I2C_CR1_ALERT	  	13 // ALERT : SMBus alert
#define I2C_CR1_SWRST	  	15 // Software reset

// I2C Control register 2

#define I2C_CR2_FREQ	  	0  // Peripheral clock frequency
#define I2C_CR2_ITERREN	  	8  // Error interrupt enable
#define I2C_CR2_ITEVTEN   	9  // Event Interrupt enable
#define I2C_CR2_ITBUFEN   	10 // Buffer interrupt enable
#define I2C_CR2_DMAEN	  	11 // DMA request enable
#define I2C_CR2_LAST	  	12 // DMA Last transfer

// I2C Own address register

#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_ADDMODE	15

// I2C Status Register 1

#define I2C_SR1_SB		  	0  // Start Bit (Master Mode)
#define I2C_SR1_ADDR	  	1  // Address sent (Master Mode)/ Matched (Slave Mode)
#define I2C_SR1_BTF		  	2  // Byte transfer finished
#define I2C_SR1_ADD10	  	3  // 10-bit header sent(Master mode)
#define I2C_SR1_STOPF	  	4  // Stop detection (slave mode)
#define I2C_SR1_RXNE	  	6  // Data register not empty (Receiver)
#define I2C_SR1_TXE		  	7  // Data register empty
#define I2C_SR1_BERR  	  	8  // Bus error
#define I2C_SR1_ARLO      	9  // Arbitration lost (Master mode)
#define I2C_SR1_AF		  	10 // Acknowledge failure
#define I2C_SR1_OVR       	11 // Overrun/ Underrun
#define I2C_SR1_PECERR    	12 // PEC Error in reception
#define I2C_SR1_TIMEOUT	  	14 // Timeout or Tlow error
#define I2C_SR1_SMBALERT  	15 // SMBus alert

// I2C Status register 2

#define I2C_SR2_MSL		  	0  // Master/Slave
#define I2C_SR2_BUSY	  	1  // Bus busy
#define I2C_SR2_TRA	      	2  // Transmitter / Receiver
#define I2C_SR2_GENCALL   	4  // General call address
#define I2C_SR2_SMBDEFAULT  5  // SMBus device default address (slave mode)
#define I2C_SR2_SMBHOST		6  // SMBus host header (slave mode)
#define I2C_SR2_DUALF		7  // Dual flag (Slave mode)
#define I2C_SR2_PEC			8  // Packet error checking register

// I2C Clock control register

#define I2C_CCR_CCR			0  // clock control register in Fm/Sm mode (Master mode)
#define I2C_CCR_DUTY		14 // Fast mode duty cycle
#define I2C_CCR_FS			15 // I2C master mode selection (0: Standard Mode, 1: Fast Mode)

/*****************************************************************
 * ********** BIT Position Macros for UART peripheral *************
 *****************************************************************/

// USART STATUS REGISTER
#define USART_SR_PE		  	0  // PARITY ERROR
#define USART_SR_FE	  		1  // FRAMING ERROR
#define USART_SR_NF	      	2  // NOISE DETECTED FLAG
#define USART_SR_ORE   		3  // OVERRUN ERROR
#define USART_SR_IDLE	 	4  // IDLE LINE DETECTED
#define USART_SR_RXNE		5  // Read Data register Not Empty
#define USART_SR_TC			6  // Transmission Complete
#define USART_SR_TXE		7  // Transmit Data register Empty
#define USART_SR_LBD		8  // LIN break detection flag
#define USART_SR_CTS		9  // CTS flag

// USART BAUD RATE REGISTER
#define USART_BRR_DIV_Fra	0  // The fraction of the USART Divider
#define USART_BRR_DIV_Man	4  // The Mantissa of the USART Divider

// USART CONTROL REGISTER 1
#define USART_CR1_SBK	  	0  // SendBreak
#define USART_CR1_RWU 		1  // Receiver wake-up
#define USART_CR1_RE      	2  // Receiver enable
#define USART_CR1_TE   		3  // Transmitter enable
#define USART_CR1_IDLEIE 	4  // IDLE interrupt enable
#define USART_CR1_RXNEIE	5  // RXNE interrupt enable
#define USART_CR1_TCIE		6  // TC interrupt enable
#define USART_CR1_TXEIE		7  // TXE interrupt enable
#define USART_CR1_PEIE		8  // Peripheral interrupt enable
#define USART_CR1_PS		9  // Parity selection
#define USART_CR1_PCE		10 // Parity control enable
#define USART_CR1_WAKE		11 // Wake-up method
#define USART_CR1_M			12 // Word length
#define USART_CR1_UE		13 // USART enable
#define USART_CR1_OVER8		15 // Over-sampling mode

// USART CONTROL REGISTER 2
#define USART_CR2_ADD	  	0  // Address of the USART node
#define USART_CR2_LBDL		5  // LIN break detection length
#define USART_CR2_LBDIE		6  // LIN  break detection
#define USART_CR2_LBCL		8  // Last bit Clock pulse
#define USART_CR2_CPHA		9  // Clock Phase
#define USART_CR2_CPOL		10 // Clock Polarity
#define USART_CR2_CLKEN		11 // Clock enable
#define USART_CR2_STOP		12 // STOP bit
#define USART_CR2_LINEN		14 // LIN Mode enable

// USART CONTROL REGISTER 3
#define USART_CR3_EIE	  	0  // Error interrupt enable
#define USART_CR3_IREN 		1  // IrDA mode enable
#define USART_CR3_IRLP     	2  // IrDA low-Power
#define USART_CR3_HDSEL  	3  // Half-duplex selection
#define USART_CR3_NACK 		4  // SmartCard NACK enable
#define USART_CR3_SCEN		5  // SmartCard mode enable
#define USART_CR3_DMAR		6  // DMA enable receiver
#define USART_CR3_DMAT		7  // DMA enable transmitter
#define USART_CR3_RTSE		8  // RTS enable
#define USART_CR3_CTSE		9  // CTS enable
#define USART_CR3_CTSIE		10 // CTS interrupt enable
#define USART_CR3_ONEBIT	11 // ONE sample bit method enable

// USART GUARD TIME AND PRESCALER REGISTER
#define USART_GTPR_PSC		0 // Pre-scaler Value
#define USART_GTPR_GT		8 // Guard time value

//****************************************************************

#include "STM32F446xx_gpio_driver.h"
#include "STM32F446xx_spi_driver.h"
#include "STM32F446xx_i2c_driver.h"
#include "STM32F446xx_usart_driver.h"
#include "STM32F446xx_rcc_driver.h"


#endif /* INC_STM32F446XX_H_ */
