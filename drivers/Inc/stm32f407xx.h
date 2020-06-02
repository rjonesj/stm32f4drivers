/*
 * stm32f407xx.h
 *
 *  Created on: May 20, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/**********************************************START: Processor Specific Details ************************************************/

/**
 * 		ARM Cortex M4 Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

/**
 * 		ARM Cortex M4 Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

/**
 * 		ARM Cortex M4 Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)


/**
 * 		ARM Cortex M4 Processor number of priority bits implemented in Priority Register
 */

#define NO_PR_BITS_IMPLEMENTED	4


/**********************************************END: Processor Specific Details ************************************************/

/**
 * Base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U //112 Kb
#define SRAM2_BASEADDR 		0x2001C000U //16 Kb
#define ROM_BASEADDR		0x1FFF0000U	//System memory
#define SRAM 				SRAM1_BASEADDR

/**
 * Base address of bus domains
 * AHB bus is used for peripherals which need high speed data communications
 * APB bus is used for peripherals which can use low speed communication
 */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/**
 * Base addresses of peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000U)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800U)

/**
 * Base addresses of peripherals on APB1 bus
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00U)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00U) /** UART does not support synchronous communication */
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000U)

/**
 * Base addresses of peripherals on APB2 bus
 */

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000U)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400U)
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800U)


/*******						Peripheral register definition structures 						********/

/**
 * Note: Registers of a peripheral are specific to the MCU
 * Ex: Number of registers of SPI peripheral of STM32F4xx family of MCUs may be different
 * 		compared to number of registers of SPI peripherals of STM32Lx or STM32F0x family of MCUs
 * Please consult your device reference manual
 */

typedef struct
{
	__vo uint32_t MODER;	/** Address Offset: 0x00 - GPIO Port Mode Offset */
	__vo uint32_t OTYPER;	/** Address Offset: 0x04 - GPIO port output type register */
	__vo uint32_t OSPEEDR;	/** Address Offset: 0x08 - GPIO port output speed register */
	__vo uint32_t PUPDR;	/** Address Offset: 0x0C - GPIO port pull-up/pull-down register */
	__vo uint32_t IDR;		/** Address Offset: 0x10 - GPIO port input data register */
	__vo uint32_t ODR;		/** Address Offset: 0x14 - GPIO port output data register */
	__vo uint32_t BSRR;		/** Address Offset: 0x18 - GPIO port bit set/reset register */
	__vo uint32_t LCKR;		/** Address Offset: 0x1C - GPIO port configuration lock register */
	__vo uint32_t AFR[2];	/** Address Offset: 0x20 - AFR[0] GPIO alternate function low register, AFR[1] GPIO alternate function high register */
} GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR;			/** Address Offset: 0x00 - RCC clock control register */
	__vo uint32_t PLLCFGR;		/** Address Offset: 0x04 - RCC PLL configuration register */
	__vo uint32_t CFGR;			/** Address Offset: 0x08 - RCC clock configuration register  */
	__vo uint32_t CIR;			/** Address Offset: 0x0C - RCC clock interrupt register */
	__vo uint32_t AHB1RSTR;		/** Address Offset: 0x10 - RCC AHB1 peripheral reset register */
	__vo uint32_t AHB2RSTR;		/** Address Offset: 0x14 - RCC AHB2 peripheral reset register */
	__vo uint32_t AHB3RSTR;		/** Address Offset: 0x18 - RCC AHB3 peripheral reset register */
	uint32_t RESVR1;			/** Address Offset: 0x1C - Reserved */
	__vo uint32_t APB1RSTR;		/** Address Offset: 0x20 - RCC APB1 peripheral reset register */
	__vo uint32_t APB2RSTR;		/** Address Offset: 0x24 - RCC APB2 peripheral reset register */
	uint32_t RESVR2;			/** Address Offset: 0x28 - Reserved */
	uint32_t RESVR3;			/** Address Offset: 0x2C - Reserved */
	__vo uint32_t AHB1ENR;		/** Address Offset: 0x30 - RCC AHB1 peripheral clock enable register */
	__vo uint32_t AHB2ENR;		/** Address Offset: 0x34 - RCC AHB2 peripheral clock enable register */
	__vo uint32_t AHB3ENR;		/** Address Offset: 0x38 - RCC AHB3 peripheral clock enable register */
	uint32_t RESVR4;			/** Address Offset: 0x3C - Reserved */
	__vo uint32_t APB1ENR;		/** Address Offset: 0x40 - RCC APB1 peripheral clock enable register */
	__vo uint32_t APB2ENR;		/** Address Offset: 0x44 - RCC APB2 peripheral clock enable register */
	uint32_t RESVR5;			/** Address Offset: 0x48 - Reserved */
	uint32_t RESVR6;			/** Address Offset: 0x4C - Reserved */
	__vo uint32_t AHB1LPENR;	/** Address Offset: 0x50 - RCC AHB1 peripheral clock enable in low power mode register */
	__vo uint32_t AHB2LPENR;	/** Address Offset: 0x54 - RCC AHB2 peripheral clock enable in low power mode register */
	__vo uint32_t AHB3LPENR;	/** Address Offset: 0x58 - RCC AHB3 peripheral clock enable in low power mode register */
	uint32_t RESVR7;			/** Address Offset: 0x5C - Reserved */
	__vo uint32_t APB1LPENR;	/** Address Offset: 0x60 - RCC APB1 peripheral clock enable in low power mode register */
	__vo uint32_t APB2LPENR;	/** Address Offset: 0x64 - RCC APB2 peripheral clock enable in low power mode register*/
	uint32_t RESVR8;			/** Address Offset: 0x68 - Reserved */
	uint32_t RESVR9;			/** Address Offset: 0x6C - Reserved */
	__vo uint32_t BDCR;			/** Address Offset: 0x70 - RCC Backup domain control register */
	__vo uint32_t CSR;			/** Address Offset: 0x74 - RCC clock control & status register  */
	uint32_t RESVR10;			/** Address Offset: 0x78 - Reserved */
	uint32_t RESVR11;			/** Address Offset: 0x7C - Reserved */
	__vo uint32_t SSCGR;		/** Address Offset: 0x80 - RCC spread spectrum clock generation register */
	__vo uint32_t PLLI2SCFGR;	/** Address Offset: 0x84 - RCC PLLI2S configuration register */
} RCC_RegDef_t;


typedef struct
{
	__vo uint32_t IMR;			/** Address Offset: 0x00 - Interrupt mask register */
	__vo uint32_t EMR;			/** Address Offset: 0x04 - Event mask register */
	__vo uint32_t RTSR;			/** Address Offset: 0x08 - Rising trigger selection register  */
	__vo uint32_t FTSR;			/** Address Offset: 0x0C - Falling trigger selection register  */
	__vo uint32_t SWIER;		/** Address Offset: 0x10 - Software interrupt event register */
	__vo uint32_t PR;			/** Address Offset: 0x14 - Pending register  */
} EXTI_RegDef_t;


typedef struct
{
	__vo uint32_t MEMRMP;		/** Address Offset: 0x00 - SYSCFG memory remap register */
	__vo uint32_t PMC;			/** Address Offset: 0x04 - SYSCFG peripheral mode configuration register  */
	__vo uint32_t EXTICR[4];	/** Address Offset: 0x08 - 0x14 - SYSCFG external interrupt configuration register 1-4  */
	uint32_t RESERVED1[2];		/** Address Offset: 0x18 - 0x1C - Reserved */
	__vo uint32_t CMPCR;		/** Address Offset: 0x20 - Compensation cell control register  */
} SYSCFG_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;			/** Address Offset: 0x00 - SPI control register 1 */
	__vo uint32_t CR2;			/** Address Offset: 0x04 - SPI control register 2 */
	__vo uint32_t SR;			/** Address Offset: 0x08 - SPI status register  */
	__vo uint32_t DR;			/** Address Offset: 0x0C - SPI data register  */
	__vo uint32_t CRCPR;		/** Address Offset: 0x10 - SPI CRC polynomial register  */
	__vo uint32_t RXCRCR;		/** Address Offset: 0x14 - SPI RX CRC register */
	__vo uint32_t TXCRCR;		/** Address Offset: 0x18 - SPI TX CRC register */
	__vo uint32_t I2SCFGR;		/** Address Offset: 0x1C - SPI_I2S configuration register  */
	__vo uint32_t I2SPR;		/** Address Offset: 0x20 - SPI_I2S prescaler register  */
} SPI_RegDef_t;


/**
 * peripheral definitions (Peripheral base addresses typecasted to XXX_RegDef_t)
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

/**
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/**
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))


/**
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))


/**
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/**
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/**
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 8))

/**
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 23))

/**
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DS()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 15))

/**
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DS()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 5))

/**
 * Clock Disable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 14))

/**
 * Register Reset Macros for GPIOx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */

#define GPIOA_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0) //technique to execute multiple statements in single macro
#define GPIOB_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)


/**
 * Register Reset Macros for SPIx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */
#define SPI1_REG_RESET()		do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while(0)


/**
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDRE_TO_CODE(x)  ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 : 8)
/**
 * Generic Macros
 */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

/*******************************************************************************************************************
 * Bit position definitions of SPI peripheral
 *******************************************************************************************************************/

/*
 * Bit position definitions for SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions for SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit position definitions for SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/**
 * IRQ (Interrupt Request) Number of STM32F407x MCU
 * TODO: Complete the list for other peripherals
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXT15_10		40

/**
 * IRQ Priority levels
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
