/*
 * stm32f407xx_timer_driver.c
 *
 *  Created on: Jul 27, 2020
 *      Author: rjonesj
 */
#include "stm32f407xx_timer_driver.h"

/**
 * Peripheral clock setup
 */
void TIMB_PeriClockControl(TIMB_RegDef_t *pTIMBx, uint8_t EnOrDis) {
	if(EnOrDis == ENABLE) {
		if(pTIMBx == TIM6) {
			TIM6_PCLK_EN();
		} else if(pTIMBx == TIM7) {
			TIM7_PCLK_EN();
		}
	} else {
		if(pTIMBx == TIM6) {
			TIM6_PCLK_DS();
		} else if(pTIMBx == TIM7) {
			TIM7_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */
void TIMB_Init(TIMB_Handle_t *pTIMBHandle) {
	//Enable peripheral clock
	TIMB_PeriClockControl(pTIMBHandle->pTIMBx, ENABLE);

	//Set the prescaler value
	pTIMBHandle->pTIMBx->PSC = pTIMBHandle->pTIMB_Config.prescaler;

	//Set the auto-reload register value
	pTIMBHandle->pTIMBx->ARR = pTIMBHandle->pTIMB_Config.autoReloadValue;
}

void TIMB_DeInit(TIMB_RegDef_t *pTIMBx) {
	if(pTIMBx == TIM6) {
		TIM6_REG_RESET();
	} else if(pTIMBx == TIM7) {
		TIM7_REG_RESET();
	}
}

/**
 * Control APIs
 */
void TIMB_Start(TIMB_RegDef_t *pTIMBx) {
	//Enable update interrupts
	pTIMBx->DIER |= (1 << TIMx_DIER_UIE);
	//Enable the counter
	pTIMBx->CR1 |= (1 << TIMx_CR1_CEN);
}

void TIMB_Stop(TIMB_RegDef_t *pTIMBx) {
	//Disable update interrupts
	pTIMBx->DIER &= ~(1 << TIMx_DIER_UIE);
	//Disable the counter
	pTIMBx->CR1 &= ~(1 << TIMx_CR1_CEN);
}

/**
 * IRQ Configuration and ISR handling
 */
void TIMB_IRQHandling(TIMB_Handle_t *pTIMBHandle) {

}

/**
 * Application callback
 */
void TIMB_ApplicationEventCallback(TIMB_Handle_t *pTIMBHandle, uint8_t appEvent);
