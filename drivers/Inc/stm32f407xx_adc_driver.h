/*
 * stm32f407xx_adc_driver.h
 *
 *  Created on: Jun 30, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_ADC_DRIVER_H_
#define INC_STM32F407XX_ADC_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for an Analog-to-Digital Converter (ADC) peripheral
 */
typedef struct {
	uint8_t ADC_Resolution;				/* possible values from @ADC_RESOLUTION */
	uint8_t ADC_Mode;					/* possible values from @ADC_MODES */
	uint8_t ADC_Align;					/* possible values from @ADC_ALIGN */
	uint8_t ADC_Channel;				/* possible values from @ADC_CHANNEL */
} ADC_Config_t;

/**
 * This is a Handle structure for an ADC peripheral
 */
typedef struct {
	ADC_RegDef_t *pADCx; 				/* This holds the base address of the ADC peripheral */
	ADC_Common_RegDef_t *pADCCx;		/* This holds the base address to the common ADC register */
	ADC_Config_t ADC_Config;			/* This holds ADC configuration settings */
} ADC_Handle_t;

/**
 * @ADC_MODES
 * ADC possible modes
 */
#define ADC_MODE_SINGLE		0			/* Single conversion mode */
#define ADC_MODE_CONTINUOUS	1			/* Continuous conversion mode */

/**
 * @ADC_RESOLUTION
 * ADC possible conversion resolution
 */
#define ADC_RESOLUTION_12		0			/* 12-bit resolution (15 ADCCLK cycles) */
#define ADC_RESOLUTION_10		1			/* 10-bit resolution (13 ADCCLK cycles) */
#define ADC_RESOLUTION_8		2			/* 8-bit resolution (11 ADCCLK cycles) */
#define ADC_RESOLUTION_6		3			/* 6-bit resolution (9 ADCCLK cycles) */

/**
 * @ADC_ALIGN
 * ADC Data alignment
 */
#define ADC_ALIGN_RIGHT			0			/* Right Alignment */
#define ADC_ALIGN_LEFT			1			/* Left Alignment */

/**
 * @ADC_CHANNEL
 * ADC regular group channels
 */
#define ADC_CH_IN0					0			/* ADC Channel 0 */
#define ADC_CH_IN1					1			/* ADC Channel 1 */
#define ADC_CH_IN2					2			/* ADC Channel 2 */
#define ADC_CH_IN3					3			/* ADC Channel 3 */
#define ADC_CH_IN4					4			/* ADC Channel 4 */
#define ADC_CH_IN5					5			/* ADC Channel 5 */
#define ADC_CH_IN6					6			/* ADC Channel 6 */
#define ADC_CH_IN7					7			/* ADC Channel 7 */
#define ADC_CH_IN8					8			/* ADC Channel 8 */
#define ADC_CH_IN9					9			/* ADC Channel 9 */
#define ADC_CH_IN10					10			/* ADC Channel 10 */
#define ADC_CH_IN11					11			/* ADC Channel 11 */
#define ADC_CH_IN12					12			/* ADC Channel 12 */
#define ADC_CH_IN13					13			/* ADC Channel 13 */
#define ADC_CH_IN14					14			/* ADC Channel 14 */
#define ADC_CH_IN15					15			/* ADC Channel 15 */
#define ADC_CH_IN16					16			/* ADC Channel 16 - Vsense (temperature sensor) */
#define ADC_CH_IN17					17			/* ADC Channel 17 - Vrefint (internal reference voltage) */
#define ADC_CH_IN18					18			/* ADC Channel 18 - Vsense and Vbat (Backup operating voltage) */


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(ADC_RegDef_t *pADCx);

/**
 * Data read and write
 */
uint16_t ADC_ReadData(ADC_RegDef_t *pADCx);

/**
 * IRQ Configuration and ISR handling
 */
void ADC_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void ADC_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void ADC_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_ADC_DRIVER_H_ */
