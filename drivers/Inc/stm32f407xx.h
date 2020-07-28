/*
 * stm32f407xx.h
 *
 *  Created on: May 20, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>
#include "constants.h"

/**
 * Clock macros
 */
#define HSE_STARTUP_TIMEOUT     ((uint16_t)0x05000)   /*!< Time out for HSE start up */
#define HSE_VALUE    			((uint32_t)8000000)   /*!< Value of the External oscillator in Hz */
#define HSI_VALUE    			((uint32_t)16000000)  /*!< Value of the Internal oscillator in Hz*/

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
#define DMA1_BASEADDR		(AHB1PERIPH_BASE + 0x6000U)
#define DMA1_Stream0_BASE   (DMA1_BASEADDR + 0x010UL)
#define DMA1_Stream1_BASE   (DMA1_BASEADDR + 0x028UL)
#define DMA1_Stream2_BASE   (DMA1_BASEADDR + 0x040UL)
#define DMA1_Stream3_BASE   (DMA1_BASEADDR + 0x058UL)
#define DMA1_Stream4_BASE   (DMA1_BASEADDR + 0x070UL)
#define DMA1_Stream5_BASE   (DMA1_BASEADDR + 0x088UL)
#define DMA1_Stream6_BASE   (DMA1_BASEADDR + 0x0A0UL)
#define DMA1_Stream7_BASE   (DMA1_BASEADDR + 0x0B8UL)
#define DMA2_BASEADDR		(AHB1PERIPH_BASE + 0x6400U)
#define DMA2_Stream0_BASE   (DMA2_BASEADDR + 0x010UL)
#define DMA2_Stream1_BASE   (DMA2_BASEADDR + 0x028UL)
#define DMA2_Stream2_BASE   (DMA2_BASEADDR + 0x040UL)
#define DMA2_Stream3_BASE   (DMA2_BASEADDR + 0x058UL)
#define DMA2_Stream4_BASE   (DMA2_BASEADDR + 0x070UL)
#define DMA2_Stream5_BASE   (DMA2_BASEADDR + 0x088UL)
#define DMA2_Stream6_BASE   (DMA2_BASEADDR + 0x0A0UL)
#define DMA2_Stream7_BASE   (DMA2_BASEADDR + 0x0B8UL)
#define FLASH_INTF_BASEADDR (AHB1PERIPH_BASE + 0x3C00U)

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
#define TIM6_BASEADDR		(APB1PERIPH_BASE + 0x1000U)
#define TIM7_BASEADDR		(APB1PERIPH_BASE + 0x1400U)

/**
 * Base addresses of peripherals on APB2 bus
 */
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000U)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400U)
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800U)
#define ADC1_BASEADDR		(APB2PERIPH_BASE + 0x2000U)
#define ADC2_BASEADDR		(ADC1_BASEADDR + 0x100U)
#define ADC3_BASEADDR		(ADC1_BASEADDR + 0x200U)
#define ADCC_BASEADDR		(ADC1_BASEADDR + 0x300U)


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

typedef struct
{
	__vo uint32_t CR1;			/** Address Offset: 0x00 - I2C Control register 1 */
	__vo uint32_t CR2;			/** Address Offset: 0x04 - I2C Control register 2 */
	__vo uint32_t OAR1;			/** Address Offset: 0x08 - I2C Own address register 1  */
	__vo uint32_t OAR2;			/** Address Offset: 0x0C - I2C Own address register 2   */
	__vo uint32_t DR;			/** Address Offset: 0x10 - I2C Data register  */
	__vo uint32_t SR1;			/** Address Offset: 0x14 - I2C Status register 1 */
	__vo uint32_t SR2;			/** Address Offset: 0x18 - I2C Status register 2 */
	__vo uint32_t CCR;			/** Address Offset: 0x1C - I2C Clock control register  */
	__vo uint32_t TRISE;		/** Address Offset: 0x20 - I2C TRISE register  */
	__vo uint32_t FLTR;			/** Address Offset: 0x24 - I2C FLTR register  */
} I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;			/** Address Offset: 0x00 - Status register */
	__vo uint32_t DR;			/** Address Offset: 0x04 - Data register */
	__vo uint32_t BRR;			/** Address Offset: 0x08 - Baud rate register   */
	__vo uint32_t CR1;			/** Address Offset: 0x0C - Control register 1   */
	__vo uint32_t CR2;			/** Address Offset: 0x10 - Control register 2  */
	__vo uint32_t CR3;			/** Address Offset: 0x14 - Control register 3 */
	__vo uint32_t GTPR;			/** Address Offset: 0x18 - Guard time and prescaler register */
} USART_RegDef_t;

typedef struct
{
	__vo uint32_t SR;			/** Address Offset: 0x00 - ADC status register */
	__vo uint32_t CR1;			/** Address Offset: 0x04 - ADC control register 1 */
	__vo uint32_t CR2;			/** Address Offset: 0x08 - ADC control register 2   */
	__vo uint32_t SMPR1;		/** Address Offset: 0x0C - ADC sample time register 1   */
	__vo uint32_t SMPR2;		/** Address Offset: 0x10 - ADC sample time register 2  */
	__vo uint32_t JOFR1;		/** Address Offset: 0x14 - ADC injected channel data offset register 1 */
	__vo uint32_t JOFR2;		/** Address Offset: 0x18 - ADC injected channel data offset register 2 */
	__vo uint32_t JOFR3;		/** Address Offset: 0x1C - ADC injected channel data offset register 3 */
	__vo uint32_t JOFR4;		/** Address Offset: 0x20 - ADC injected channel data offset register 4 */
	__vo uint32_t HTR;			/** Address Offset: 0x24 - ADC watchdog higher threshold register */
	__vo uint32_t LTR;			/** Address Offset: 0x28 - ADC watchdog lower threshold register */
	__vo uint32_t SQR1;			/** Address Offset: 0x2C - ADC regular sequence register 1 */
	__vo uint32_t SQR2;			/** Address Offset: 0x30 - ADC regular sequence register 2 */
	__vo uint32_t SQR3;			/** Address Offset: 0x34 - ADC regular sequence register 3 */
	__vo uint32_t JSQR;			/** Address Offset: 0x38 - ADC injected sequence register */
	__vo uint32_t JDR1;			/** Address Offset: 0x3C - ADC injected data register 1 */
	__vo uint32_t JDR2;			/** Address Offset: 0x40 - ADC injected data register 2 */
	__vo uint32_t JDR3;			/** Address Offset: 0x44 - ADC injected data register 3 */
	__vo uint32_t JDR4;			/** Address Offset: 0x48 - ADC injected data register 4 */
	__vo uint32_t DR;			/** Address Offset: 0x4C - ADC regular data register */
} ADC_RegDef_t;

typedef struct
{
	__vo uint32_t CSR;			/** Address Offset: 0x00 - ADC Common status register  */
	__vo uint32_t CCR;			/** Address Offset: 0x04 - ADC common control register */
	__vo uint32_t CDR;			/** Address Offset: 0x08 - ADC common regular data register for dual and triple modes  */
} ADC_Common_RegDef_t;

typedef struct
{
	__vo uint32_t CR;     /*!< DMA stream x configuration register, Address offset: 0x10 + 0x18 × stream number      */
	__vo uint32_t NDTR;   /*!< DMA stream x number of data register, Address offset: 0x14 + 0x18 × stream number     */
	__vo uint32_t PAR;    /*!< DMA stream x peripheral address register, Address offset: 0x18 + 0x18 × stream number */
	__vo uint32_t M0AR;   /*!< DMA stream x memory 0 address register, Address offset: 0x1C + 0x18 × stream number   */
	__vo uint32_t M1AR;   /*!< DMA stream x memory 1 address register, Address offset: 0x20 + 0x18 × stream number   */
	__vo uint32_t FCR;    /*!< DMA stream x FIFO control register, Address offset: 0x24 + 0x24 × stream number       */
} DMA_Stream_RegDef_t;

typedef struct
{
	__vo uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
	__vo uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
	__vo uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
	__vo uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;   /*!< TIM6 and TIM7 control register 1,      Address offset: 0x00 */
	__vo uint32_t CR2;   /*!< TIM6 and TIM7 control register 2,     Address offset: 0x04 */
	__vo uint32_t RESVR;   /*!< Reserved,     Address offset: 0x08 */
	__vo uint32_t DIER;  /*!< TIM6 and TIM7 DMA/Interrupt enable register,  Address offset: 0x0C */
	__vo uint32_t SR;  /*!< TIM6 and TIM7 status register, Address offset: 0x10 */
	__vo uint32_t EGR;  /*!< TIM6 and TIM7 event generation register, Address offset: 0x14 */
	__vo uint32_t RESVR2[3];   /*!< Reserved,     Address offset: 0x18 - 0x20 */
	__vo uint32_t CNT;  /*!< TIM6 and TIM7 counter, Address offset: 0x24 */
	__vo uint32_t PSC;  /*!< TIM6 and TIM7 prescaler, Address offset: 0x28 */
	__vo uint32_t ARR;  /*!< TIM6 and TIM7 auto-reload register, Address offset: 2C */
} TIMB_RegDef_t;

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

#define DMA1				((DMA_RegDef_t*)DMA1_BASEADDR)
#define DMA1_Stream0        ((DMA_Stream_RegDef_t *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_RegDef_t *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_RegDef_t *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_RegDef_t *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_RegDef_t *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_RegDef_t *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_RegDef_t *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_RegDef_t *) DMA1_Stream7_BASE)
#define DMA2				((DMA_RegDef_t*)DMA2_BASEADDR)
#define DMA2_Stream0        ((DMA_Stream_RegDef_t *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_RegDef_t *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_RegDef_t *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_RegDef_t *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_RegDef_t *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_RegDef_t *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_RegDef_t *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_RegDef_t *) DMA2_Stream7_BASE)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6				((USART_RegDef_t*)USART6_BASEADDR)

#define ADC1				((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2				((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3				((ADC_RegDef_t*)ADC3_BASEADDR)
#define ADCC				((ADC_Common_RegDef_t*)ADCC_BASEADDR)

#define TIM6				((TIMB_RegDef_t*)TIM6_BASEADDR)
#define TIM7				((TIMB_RegDef_t*)TIM7_BASEADDR)

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

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
#define IRQ_NO_SPI6			86

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71

#define IRQ_NO_ADC			18

#define IRQ_NO_DMA1_STREAM0 11
#define IRQ_NO_DMA1_STREAM1 12
#define IRQ_NO_DMA1_STREAM2 13
#define IRQ_NO_DMA1_STREAM3 14
#define IRQ_NO_DMA1_STREAM4 15
#define IRQ_NO_DMA1_STREAM5 16
#define IRQ_NO_DMA1_STREAM6 17
#define IRQ_NO_DMA1_STREAM7 47

#define IRQ_NO_DMA2_STREAM0 56
#define IRQ_NO_DMA2_STREAM1 57
#define IRQ_NO_DMA2_STREAM2 58
#define IRQ_NO_DMA2_STREAM3 59
#define IRQ_NO_DMA2_STREAM4 60
#define IRQ_NO_DMA2_STREAM5 68
#define IRQ_NO_DMA2_STREAM6 69
#define IRQ_NO_DMA2_STREAM7 70

#define IRQ_NO_TIM6_DAC 	54
#define IRQ_NO_TIM7		 	55

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

#include "armcortexm4_nvic_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_adc_driver.h"
#include "stm32f407xx_dma_driver.h"
#include "stm32f407xx_timer_driver.h"

#endif /* INC_STM32F407XX_H_ */
