/*
 * stm32f407xx_rcc_driver.c
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

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL and PLLI2S OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  * @param  None
  * @retval None
  */
void RCC_DeInit(void)
{
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON, PLLON, and PLLI2S bits */
  RCC->CR &= (uint32_t)0xEAF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset PLLI2SCFGR register */
  RCC->PLLI2SCFGR = 0x20003000;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;
}

/**
  * @brief  Configures the External High Speed oscillator (HSE).
  * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the CSSON bit, so if the Clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  RCC_HSE: specifies the new state of the HSE.
  *          This parameter can be one of the following values:
  *            @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
  *                              6 HSE oscillator clock cycles.
  *            @arg RCC_HSE_ON: turn ON the HSE oscillator
  *            @arg RCC_HSE_Bypass: HSE oscillator bypassed with external clock
  * @retval None
  */
void RCC_HSEConfig(uint8_t RCC_HSE)
{
  /* Reset HSEON and HSEBYP bits before configuring the HSE ------------------*/
  RCC->CR &= ~(0xF << RCC_CR_HSEON);

  /* Set the new HSE configuration -------------------------------------------*/
  RCC->CR |= (RCC_HSE << RCC_CR_HSEON);
}

/**
  * @brief  Waits for HSE start-up.
  * @note   This functions waits on HSERDY flag to be set and return SUCCESS if
  *         this flag is set, otherwise returns ERROR if the timeout is reached
  *         and this flag is not set. The timeout value is defined by the constant
  *         HSE_STARTUP_TIMEOUT in stm32f4xx.h file. You can tailor it depending
  *         on the HSE crystal used in your application.
  * @param  None
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: HSE oscillator is stable and ready to use
  *          - ERROR: HSE oscillator not yet ready
  */
ErrorStatus RCC_WaitForHSEStartUp(void)
{
  __vo uint32_t startupcounter = 0;
  ErrorStatus status = ERROR;
  FlagStatus hsestatus = RESET;
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    hsestatus = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
    startupcounter++;
  } while((startupcounter != HSE_STARTUP_TIMEOUT) && (hsestatus == RESET));

  if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
  {
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }
  return (status);
}

/**
  * @brief  Configures the main PLL clock source, multiplication and division factors.
  * @note   This function must be used only when the main PLL is disabled.
  *
  * @param  RCC_PLLSource: specifies the PLL entry clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_PLLSource_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCC_PLLSource_HSE: HSE oscillator clock selected as PLL clock entry
  * @note   This clock source (RCC_PLLSource) is common for the main PLL and PLLI2S.
  *
  * @param  PLLM: specifies the division factor for PLL VCO input clock
  *          This parameter must be a number between 0 and 63.
  * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  *         VCO input frequency = PLL input clock frequency / PLLM with 2 ≤PLLM ≤63
  *
  * @param  PLLN: specifies the multiplication factor for PLL VCO output clock
  *          This parameter must be a number between 50 and 432.
  * @note   You have to set the PLLN parameter correctly to ensure that the VCO
  *         output frequency is between 100 and 432 MHz.
  *
  * @param  PLLP: specifies the division factor for main system clock (SYSCLK)
  *          This parameter must be a number in the range {2, 4, 6, or 8}.
  * @note   You have to set the PLLP parameter correctly to not exceed 168 MHz on
  *         the System clock frequency.
  *
  * @param  PLLQ: specifies the division factor for OTG FS, SDIO and RNG clocks
  *          This parameter must be a number between 4 and 15.
  * @note   If the USB OTG FS is used in your application, you have to set the
  *         PLLQ parameter correctly to have 48 MHz clock for the USB. However,
  *         the SDIO and RNG need a frequency lower than or equal to 48 MHz to work
  *         correctly.
  *
  * @retval None
  */
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ)
{

  RCC->PLLCFGR = PLLM | (PLLN << 6) | (((PLLP >> 1) -1) << 16) | (RCC_PLLSource) |
                 (PLLQ << 24);
}

/**
  * @brief  Enables or disables the main PLL.
  * @note   After enabling the main PLL, the application software should wait on
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL can not be disabled if it is used as system clock source
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  * @param  ENABLE or DISABLE macros
  * @retval None
  */
void RCC_PLLCmd(uint8_t enOrDis)
{
  RCC->CR |= (enOrDis << RCC_CR_PLLON);
}

/**
  * @brief  Configures the system clock (SYSCLK).
  * @note   The HSI is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  * @param  RCC_SYSCLKSource: specifies the clock source used as system clock.
  *          This parameter can be one of the following values:
  *            @arg RCC_SYSCLKSource_HSI: HSI selected as system clock source
  *            @arg RCC_SYSCLKSource_HSE: HSE selected as system clock source
  *            @arg RCC_SYSCLKSource_PLLCLK: PLL selected as system clock source
  * @retval None
  */
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource)
{
  uint32_t tmpreg = 0;

  tmpreg = RCC->CFGR;

  /* Clear SW[1:0] bits */
  tmpreg &= ~(0x3 << RCC_CFGR_SW0);

  /* Set SW[1:0] bits according to RCC_SYSCLKSource value */
  tmpreg |= RCC_SYSCLKSource;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/**
  * @brief  Configures the AHB clock (HCLK).
  * @note   Depending on the device voltage range, the software has to set correctly
  *         these bits to ensure that HCLK not exceed the maximum allowed frequency.
  * @param  RCC_SYSCLK: defines the AHB clock divider. This clock is derived from
  *         the system clock (SYSCLK).
  *          This parameter can be one of the following values:
  *            @arg RCC_SYSCLK_Div1: AHB clock = SYSCLK
  *            @arg RCC_SYSCLK_Div2: AHB clock = SYSCLK/2
  *            @arg RCC_SYSCLK_Div4: AHB clock = SYSCLK/4
  *            @arg RCC_SYSCLK_Div8: AHB clock = SYSCLK/8
  *            @arg RCC_SYSCLK_Div16: AHB clock = SYSCLK/16
  *            @arg RCC_SYSCLK_Div64: AHB clock = SYSCLK/64
  *            @arg RCC_SYSCLK_Div128: AHB clock = SYSCLK/128
  *            @arg RCC_SYSCLK_Div256: AHB clock = SYSCLK/256
  *            @arg RCC_SYSCLK_Div512: AHB clock = SYSCLK/512
  * @retval None
  */
void RCC_HCLKConfig(uint32_t RCC_SYSCLK)
{
  uint32_t tmpreg = 0;

  tmpreg = RCC->CFGR;

  /* Clear HPRE[3:0] bits */
  tmpreg &= ~(0xF << RCC_CFGR_HPRE);

  /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
  tmpreg |= RCC_SYSCLK;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/**
  * @brief  Configures the Low Speed APB clock (PCLK1).
  * @param  RCC_HCLK: defines the APB1 clock divider. This clock is derived from
  *         the AHB clock (HCLK).
  *          This parameter can be one of the following values:
  *            @arg RCC_HCLK_Div1:  APB1 clock = HCLK
  *            @arg RCC_HCLK_Div2:  APB1 clock = HCLK/2
  *            @arg RCC_HCLK_Div4:  APB1 clock = HCLK/4
  *            @arg RCC_HCLK_Div8:  APB1 clock = HCLK/8
  *            @arg RCC_HCLK_Div16: APB1 clock = HCLK/16
  * @retval None
  */
void RCC_PCLK1Config(uint32_t RCC_HCLK)
{
  uint32_t tmpreg = 0;

  tmpreg = RCC->CFGR;

  /* Clear PPRE1[2:0] bits */
  tmpreg &= ~(0x7 << RCC_CFGR_PPRE);

  /* Set PPRE1[2:0] bits according to RCC_HCLK value */
  tmpreg |= RCC_HCLK;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/**
  * @brief  Configures the High Speed APB clock (PCLK2).
  * @param  RCC_HCLK: defines the APB2 clock divider. This clock is derived from
  *         the AHB clock (HCLK).
  *          This parameter can be one of the following values:
  *            @arg RCC_HCLK_Div1:  APB2 clock = HCLK
  *            @arg RCC_HCLK_Div2:  APB2 clock = HCLK/2
  *            @arg RCC_HCLK_Div4:  APB2 clock = HCLK/4
  *            @arg RCC_HCLK_Div8:  APB2 clock = HCLK/8
  *            @arg RCC_HCLK_Div16: APB2 clock = HCLK/16
  * @retval None
  */
void RCC_PCLK2Config(uint32_t RCC_HCLK)
{
  uint32_t tmpreg = 0;

  tmpreg = RCC->CFGR;

  /* Clear PPRE2[2:0] bits */
  tmpreg &= ~(0x7 << RCC_CFGR_PPRE2);

  /* Set PPRE2[2:0] bits according to RCC_HCLK value */
  tmpreg |= RCC_HCLK << 3;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

uint32_t RCC_GetPLLOutputClock() {
	//TODO: Implement function
	return 0;
}


/**
 * @fn			- RCC_GetPCLK1_Value
 * @brief		- This function returns the speed of the clock in the APB1 bus in hertz
 *
 * @return		- Fpclk value
 * @note		- none
 */
uint32_t RCC_GetPCLK1_Value() {
	uint32_t pclk1, systemclk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0) {
		//HSI oscillator
		systemclk = HSI_VALUE;
	} else if(clksrc == 1) {
		//HSE oscillator
		systemclk = HSE_VALUE;
	} else if(clksrc == 2) {
		//PLL
		systemclk = RCC_GetPLLOutputClock();
	}

	//AHBP prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	//APB1 prescaler
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (systemclk / ahbp) / apb1p;

	return pclk1;
}


/**
 * @fn			- RCC_GetPCLK2_Value
 * @brief		- This function returns the speed of the clock in the APB2 bus in hertz
 *
 * @return		- Fpclk2 value
 * @note		- none
 */
uint32_t RCC_GetPCLK2_Value() {
	uint32_t pclk2, systemclk;
	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0) {
		//HSI oscillator
		systemclk = HSI_VALUE;
	} else if(clksrc == 1) {
		//HSE oscillator
		systemclk = HSE_VALUE;
	} else if(clksrc == 2) {
		//PLL
		systemclk = RCC_GetPLLOutputClock();
	}

	//AHBP prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	//APB2 prescaler
	temp = ((RCC->CFGR >> 13) & 0x7);
	if(temp < 4) {
		apb2p = 1;
	} else {
		apb2p = APB1_PreScaler[temp-4];
	}

	pclk2 = (systemclk / ahbp) / apb2p;

	return pclk2;
}

/**
 * @brief  Checks whether the specified RCC flag is set or not.
 * @param  RCC_FLAG: specifies the flag to check.
 *          This parameter can be one of the following values:
 *            @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready
 *            @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
 *            @arg RCC_FLAG_PLLRDY: main PLL clock ready
 *            @arg RCC_FLAG_PLLI2SRDY: PLLI2S clock ready
 *            @arg RCC_FLAG_PLLSAIRDY: PLLSAI clock ready (only for STM32F42xxx/43xxx/446xx/469xx/479xx devices)
 *            @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
 *            @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
 *            @arg RCC_FLAG_BORRST: POR/PDR or BOR reset
 *            @arg RCC_FLAG_PINRST: Pin reset
 *            @arg RCC_FLAG_PORRST: POR/PDR reset
 *            @arg RCC_FLAG_SFTRST: Software reset
 *            @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
 *            @arg RCC_FLAG_WWDGRST: Window Watchdog reset
 *            @arg RCC_FLAG_LPWRRST: Low Power reset
 * @retval The state of RCC_FLAG (SET or RESET).
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
 uint32_t tmp = 0;
 uint32_t statusreg = 0;
 FlagStatus bitstatus = FS_RESET;

 /* Get the RCC register index */
 tmp = RCC_FLAG >> 5;
 if (tmp == 1)               /* The flag to check is in CR register */
 {
   statusreg = RCC->CR;
 }
 else if (tmp == 2)          /* The flag to check is in BDCR register */
 {
   statusreg = RCC->BDCR;
 }
 else                       /* The flag to check is in CSR register */
 {
   statusreg = RCC->CSR;
 }

 /* Get the flag position */
 tmp = RCC_FLAG & FLAG_MASK;
 if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)FS_RESET)
 {
   bitstatus = FS_SET;
 }
 else
 {
   bitstatus = FS_RESET;
 }
 /* Return the flag status */
 return bitstatus;
}

/**
  * @brief  Sets the code latency value.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *          This parameter can be one of the following values:
  *            @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *            @arg FLASH_Latency_1: FLASH One Latency cycle
  *            @arg FLASH_Latency_2: FLASH Two Latency cycles
  *            @arg FLASH_Latency_3: FLASH Three Latency cycles
  *            @arg FLASH_Latency_4: FLASH Four Latency cycles
  *            @arg FLASH_Latency_5: FLASH Five Latency cycles
  *            @arg FLASH_Latency_6: FLASH Six Latency cycles
  *            @arg FLASH_Latency_7: FLASH Seven Latency cycles
  *
  * @retval None
  */
void FLASH_SetLatency(uint8_t FLASH_Latency)
{
  /* Perform Byte access to FLASH_ACR[8:0] to set the Latency value */
  *(__vo uint8_t *) FLASH_INTF_BASEADDR |= FLASH_Latency;
}
