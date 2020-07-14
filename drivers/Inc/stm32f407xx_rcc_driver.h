/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jun 25, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

uint32_t RCC_GetPLLOutputClock();
uint32_t RCC_GetPCLK1_Value();
uint32_t RCC_GetPCLK2_Value();

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
 * Clock Enable Macros for DMAx peripherals
 */
#define DMA1_PCLK_EN()		(RCC->AHB1ENR |= (1 << 21))
#define DMA2_PCLK_EN()		(RCC->AHB1ENR |= (1 << 22))

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
 * Clock Enable Macros for ADCx peripherals
 */
#define ADC1_PCLK_EN()	(RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN()	(RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN()	(RCC->APB2ENR |= (1 << 10))

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
 * Clock Disable Macros for DMAx peripherals
 */
#define DMA1_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 21))
#define DMA2_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 22))

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
 * Clock Disable Macros for ADCx peripherals
 */
#define ADC1_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 10))

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
 * Register Reset Macros for DMAx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */
#define DMA1_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 21)); (RCC->AHB1RSTR &= ~(1 << 21)); } while(0)
#define DMA2_REG_RESET()		do {(RCC->AHB1RSTR |= (1 << 22)); (RCC->AHB1RSTR &= ~(1 << 22)); } while(0)

/**
 * Register Reset Macros for SPIx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */
#define SPI1_REG_RESET()		do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while(0)

/**
 * Register Reset Macros for I2Cx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */
#define I2C1_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); } while(0)

/**
 * Register Reset Macros for USARTx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */
#define USART1_REG_RESET()		do {(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); } while(0)
#define USART2_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); } while(0)
#define USART3_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); } while(0)
#define UART4_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); } while(0)
#define UART5_REG_RESET()		do {(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); } while(0)
#define USART6_REG_RESET()		do {(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); } while(0)

/**
 * Register Reset Macros for ADCx peripherals
 * Bit must first be set then cleared so it does not stay in reset state
 */
#define ADC_REG_RESET()		do {(RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8)); } while(0)


#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
