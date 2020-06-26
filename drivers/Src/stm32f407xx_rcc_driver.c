/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jun 25, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

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
		systemclk = 16000000;
	} else if(clksrc == 1) {
		//HSE oscillator
		systemclk = 8000000;
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
		systemclk = 16000000;
	} else if(clksrc == 1) {
		//HSE oscillator
		systemclk = 8000000;
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
