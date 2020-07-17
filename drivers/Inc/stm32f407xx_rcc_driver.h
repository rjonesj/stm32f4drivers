/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jun 25, 2020
 *      Author: rjonesj
 * 	@note Some code has been taken / modified from the STM32F4xx_StdPeriph_driver, please see notice below
 *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

/**
 * @RCC_HSE_configuration
 * Possible values for RCC_HSEConfig
  */
#define RCC_HSE_OFF                      ((uint8_t)0x00)
#define RCC_HSE_ON                       ((uint8_t)0x01)
#define RCC_HSE_Bypass                   ((uint8_t)0x05)

/**
 * @RCC_Flag_Status
 * Possible values for RCC_GetFlagStatus
  */
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3B)
#define RCC_FLAG_PLLSAIRDY               ((uint8_t)0x3D)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_BORRST                  ((uint8_t)0x79)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

/**
 * @RCC_PLL_Clock_Source
 * Possible values for RCC_PLLConfig RCC_PLLSource
  */
#define RCC_PLLSource_HSI                ((uint32_t)0x00000000)
#define RCC_PLLSource_HSE                ((uint32_t)0x00400000)

/**
 * @Flash_Latency
 * Possible values for FLASH_SetLatency
  */
#define FLASH_Latency_0                ((uint8_t)0x0000)  /*!< FLASH Zero Latency cycle      */
#define FLASH_Latency_1                ((uint8_t)0x0001)  /*!< FLASH One Latency cycle       */
#define FLASH_Latency_2                ((uint8_t)0x0002)  /*!< FLASH Two Latency cycles      */
#define FLASH_Latency_3                ((uint8_t)0x0003)  /*!< FLASH Three Latency cycles    */
#define FLASH_Latency_4                ((uint8_t)0x0004)  /*!< FLASH Four Latency cycles     */
#define FLASH_Latency_5                ((uint8_t)0x0005)  /*!< FLASH Five Latency cycles     */
#define FLASH_Latency_6                ((uint8_t)0x0006)  /*!< FLASH Six Latency cycles      */
#define FLASH_Latency_7                ((uint8_t)0x0007)  /*!< FLASH Seven Latency cycles    */

/**
 *  @RCC_AHB_Clock_Source
 *  Possible values for RCC_HCLKConfig
  */
#define RCC_SYSCLK_Div1                  ((uint32_t)0x00000000)
#define RCC_SYSCLK_Div2                  ((uint32_t)0x00000080)
#define RCC_SYSCLK_Div4                  ((uint32_t)0x00000090)
#define RCC_SYSCLK_Div8                  ((uint32_t)0x000000A0)
#define RCC_SYSCLK_Div16                 ((uint32_t)0x000000B0)
#define RCC_SYSCLK_Div64                 ((uint32_t)0x000000C0)
#define RCC_SYSCLK_Div128                ((uint32_t)0x000000D0)
#define RCC_SYSCLK_Div256                ((uint32_t)0x000000E0)
#define RCC_SYSCLK_Div512                ((uint32_t)0x000000F0)

/**
 * @RCC_APB1_APB2_Clock_Source
  * Possible values for RCC_PCLK1Config and RCC_PCLK2Config
  */
#define RCC_HCLK_Div1                    ((uint32_t)0x00000000)
#define RCC_HCLK_Div2                    ((uint32_t)0x00001000)
#define RCC_HCLK_Div4                    ((uint32_t)0x00001400)
#define RCC_HCLK_Div8                    ((uint32_t)0x00001800)
#define RCC_HCLK_Div16                   ((uint32_t)0x00001C00)

/**
 * @RCC_System_Clock_Source
  * Possible values for RCC_SYSCLKConfig
  */
#define RCC_SYSCLKSource_HSI             ((uint32_t)0x00000000)
#define RCC_SYSCLKSource_HSE             ((uint32_t)0x00000001)
#define RCC_SYSCLKSource_PLLCLK          ((uint32_t)0x00000002)

/* RCC Flag Mask */
#define FLAG_MASK                 ((uint8_t)0x1F)

/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/* Function used to set the RCC clock configuration to the default reset state */
void        RCC_DeInit(void);

/* Internal/external clocks, PLL, CSS and MCO configuration functions *********/
void        RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void        RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);
void        RCC_PLLCmd(uint8_t NewState);

/* System, AHB and APB busses clocks configuration functions ******************/
void        RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
void        RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void        RCC_PCLK1Config(uint32_t RCC_HCLK);
void        RCC_PCLK2Config(uint32_t RCC_HCLK);

/* Get clock speed functions *********/
uint32_t RCC_GetPLLOutputClock();
uint32_t RCC_GetPCLK1_Value();
uint32_t RCC_GetPCLK2_Value();

/* Flag management functions **********************************/
FlagStatus  RCC_GetFlagStatus(uint8_t RCC_FLAG);

/* FLASH Interface configuration functions ************************************/
void FLASH_SetLatency(uint8_t FLASH_Latency);

/*******************************************************************************************************************
 * Bit position definitions of RCC peripheral
 *******************************************************************************************************************/

/*
 * Bit position definitions for RCC_CR
 */
#define RCC_CR_HSION			0
#define RCC_CR_HSIRDY			1
#define RCC_CR_HSITRIM			3
#define RCC_CR_HSICAL			8
#define RCC_CR_HSEON			16
#define RCC_CR_HSERDY			17
#define RCC_CR_HSEBYP			18
#define RCC_CR_CSSON			19
#define RCC_CR_PLLON			24
#define RCC_CR_PLLRDY			25
#define RCC_CR_PLLI2SON			26
#define RCC_CR_PLLI2SRDY		27

/*
 * Bit position definitions for RCC_CFGR
 */
#define RCC_CFGR_SW0			0
#define RCC_CFGR_SW1			1
#define RCC_CFGR_SWS0			2
#define RCC_CFGR_SWS1			3
#define RCC_CFGR_HPRE			4
#define RCC_CFGR_PPRE			10
#define RCC_CFGR_PPRE2			13
#define RCC_CFGR_RTCPRE			16
#define RCC_CFGR_MCO1			21
#define RCC_CFGR_I2SSCR			23
#define RCC_CFGR_MCO1PRE		24
#define RCC_CFGR_MCO2PRE		27
#define RCC_CFGR_MCO2			30


/**
 * End Bit Position Definitions
 */

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
