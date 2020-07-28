/*
 * stm32f407xx_timer_driver.h
 *
 *  Created on: Jul 27, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_TIMER_DRIVER_H_
#define INC_STM32F407XX_TIMER_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a Basic Timer peripheral (TIM6 or TIM7)
 */
typedef struct {
	uint16_t prescaler;				/* Prescaler value - Contains the value to be loaded in the active prescaler register at each update event */
	uint16_t autoReloadValue;		/* The value to be loaded into ARR register. Update event occurs when ARR = CNT */
} TIMB_Config_t;

/**
 * This is a Handle structure for a Basic Timer peripheral
 */
typedef struct {
	TIMB_RegDef_t *pTIMBx;				/* This holds the base address of the basic timer peripheral */
	TIMB_Config_t pTIMB_Config;		/* This holds basic timer configuration settings */
} TIMB_Handle_t;


/*******************************************************************************************************************
 * Bit position definitions of TIM peripheral
 *******************************************************************************************************************/

/*
 * Bit position definitions for TIMx_CR1
 */
#define TIMx_CR1_CEN			0
#define TIMx_CR1_UDIS			1
#define TIMx_CR1_URS			2
#define TIMx_CR1_OPM			3
#define TIMx_CR1_ARPE			7

/*
 * Bit position definitions for TIMx_CR2
 */
#define TIMx_CR2_MMS			4

/*
 * Bit position definitions for TIMx_DIER
 */
#define TIMx_DIER_UIE			0
#define TIMx_DIER_UDE			8

/*
 * Bit position definitions for TIMx_SR
 */
#define TIMx_SR_UIF				0

/*
 * Bit position definitions for TIMx_EGR
 */
#define TIMx_EGR_UG				0

/*
 * Bit position definitions for TIMx_CNT
 */
#define TIMx_CNT_CNT				0

/*
 * Bit position definitions for TIMx_PSC
 */
#define TIMx_PSC_PSC				0

/*
 * Bit position definitions for TIMx_ARR
 */
#define TIMx_ARR_ARR				0


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void TIMB_PeriClockControl(TIMB_RegDef_t *pTIMBx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void TIMB_Init(TIMB_Handle_t *pTIMBHandle);
void TIMB_DeInit(TIMB_RegDef_t *pTIMBx);

/**
 * Control APIs
 */
void TIMB_Start(TIMB_RegDef_t *pTIMBx);
void TIMB_Stop(TIMB_RegDef_t *pTIMBx);

/**
 * IRQ Configuration and ISR handling
 */
void TIMB_IRQHandling(TIMB_Handle_t *pTIMBHandle);

/**
 * Application callback
 */
void TIMB_ApplicationEventCallback(TIMB_Handle_t *pTIMBHandle, uint8_t appEvent);


#endif /* INC_STM32F407XX_TIMER_DRIVER_H_ */
