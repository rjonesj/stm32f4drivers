/*
 * stm32f407xx_adc_driver.c
 *
 *  Created on: Jul 1, 2020
 *      Author: rjonesj
 */
#include "stm32f407xx_adc_driver.h"
#include <string.h>

/**
 * Private functions
 */
static void delay(void) {
	for(int i = 0; i < 500000; i++);
}

/**
 * Peripheral clock setup
 */

/**
 * @fn			- ADC_PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given ADC peripheral
 *
 * @param[in]	- Base address of the ADC peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @note		- none
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDis) {
	if(EnOrDis == ENABLE) {
		if(pADCx == ADC1) {
			ADC1_PCLK_EN();
		} else if(pADCx == ADC2) {
			ADC2_PCLK_EN();
		} else if(pADCx == ADC3) {
			ADC3_PCLK_EN();
		}
	} else {
		if(pADCx == ADC1) {
			ADC1_PCLK_DS();
		} else if(pADCx == ADC2) {
			ADC2_PCLK_DS();
		} else if(pADCx == ADC3) {
			ADC3_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */

/**
 * @fn			- ADC_Init
 * @brief		- This function configures the settings for a ADC peripheral
 *
 * @param[in]	- Address to ADC_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void ADC_Init(ADC_Handle_t *pADCHandle) {
	//Temporary variable
	uint32_t tempreg=0;

	//Initialize ADCC register
	pADCHandle->pADCCx = ADCC;

	/******************************** Configuration of CR1******************************************/

	//Enable the Clock for given ADC peripheral
	ADC_PeriClockControl(pADCHandle->pADCx, ENABLE);

	//Set Resolution
	tempreg |= (pADCHandle->ADC_Config.ADC_Resolution << ADC_CR1_RES);
	pADCHandle->pADCx->CR1 |= tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Configure continuous conversion mode
	tempreg |= (pADCHandle->ADC_Config.ADC_Mode << ADC_CR2_CONT);

	//Configure data alignment
	tempreg |= (pADCHandle->ADC_Config.ADC_Align << ADC_CR2_ALIGN);

	//Program the CR2 register
	pADCHandle->pADCx->CR2 |= tempreg;

	/******************************** Configuration of SQR (Regular Sequence Register) ******************************************/
		//Note: Only one conversion is currently supported using regular channel conversion

	tempreg=0;

	//Configure 1st conversion in regular sequence
	tempreg |= (pADCHandle->ADC_Config.ADC_Channel);

	//Program the SQR3 register
	pADCHandle->pADCx->SQR3 |= tempreg;
}

/**
 * @fn			- ADC_DeInit
 * @brief		- This function resets all the registers for all ADC peripherals
 *
 *
 * @return		- none
 * @note		- none
 */
void ADC_DeInit() {
	ADC_REG_RESET();
}

/**
 * Data Send and Receive
 */

/**
 * @fn			- ADC_ReadData
 * @brief		- This function returns the value of the DR register if EOC bit is set in SR register
 *
 * @param[in]	- Base address of the ADC peripheral
 * @return		- DR contents
 * @note		- none
 */
uint16_t ADC_ReadData(ADC_RegDef_t *pADCx) {
	uint16_t data = 0;

	//Wait for EOC bit to be set
	while(!((pADCx->SR >> ADC_SR_EOC) & 0x1));

	//Return DR if EOC bit is set
	data = pADCx->DR;

	return data;
}



/**
 * IRQ Configuration and ISR handling
 */


/**
 * @fn			- ADC_IRQConfig
 * @brief		- This function enables or disables interrupts in the NVIC ISER registers
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void ADC_IRQConfig(uint8_t irqNumber, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		if(irqNumber <= 31) {
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber < 64) {
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber >= 64 && irqNumber < 96) {
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (irqNumber % 64));
		}
	} else {
		if(irqNumber <= 31) {
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber < 64) {
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber >= 64 && irqNumber < 96) {
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (irqNumber % 64));
		}
	}
}

/**
 * @fn			- ADC_IRQPriorityConfig
 * @brief		- This function sets the priority of an IRQ number
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- priority to be assigned to irqNumber
 *
 * @return		- none
 * @note		- none
 */
void ADC_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
	//First, find the IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber% 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Then set priority at register
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - ADC_IRQHandler
 * @brief             - This function handles IRQ interrupt events for ADC peripheral
 *
 * @param[in]         - Address to USART Handle struct
 *
 * @return            - none
 * @Note              - none

 */
void ADC_IRQHandling(ADC_Handle_t *pADCHandle)
{
	//TODO: Implement function
}

/**
 * Other Control APIs
 */

/**
 * @fn			- ADC_PeripheralControl
 * @brief		- This function enables or disables a ADC peripheral by setting the CR2 ADON bit
 *
 * @param[in]	- Base address of the ADC peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void ADC_PeripheralControl(ADC_RegDef_t *pADCx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pADCx->CR2 |= (1 << ADC_CR2_ADON);
	} else {
		pADCx->CR2 &= ~(1 << ADC_CR2_ADON);
	}

	//Wait for tSTAB time for accurate readings
	delay();
}

/**
 * @fn			- ADC_StartConversion
 * @brief		- This function starts ADC conversion by setting the SWSTART bit in CR2
 *
 * @param[in]	- Base address of the ADC peripheral
 *
 * @return		- none
 * @note		- none
 */
void ADC_StartConversion(ADC_RegDef_t *pADCx) {
	//Clear EOC bit
	pADCx->SR &= ~(1 << ADC_SR_EOC);
	//Set SWSTART bit
	pADCx->CR2 |= (1 << ADC_CR2_SWSTART);
}

/**
 * Application callback
 */
__weak void ADC_ApplicationEventCallback(ADC_Handle_t *pADCHandle, uint8_t appEvent) {

}
