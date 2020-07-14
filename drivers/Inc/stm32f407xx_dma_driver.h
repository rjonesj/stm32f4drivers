/*
 * stm32f407xx_dma_driver.h
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_DMA_DRIVER_H_
#define INC_STM32F407XX_DMA_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a DMA controller
 */
typedef struct {
	uint8_t GPIO_PinNumber;				/* possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;				/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;				/* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;		/* possible values from @GPIO_PUPD */
	uint8_t GPIO_PinOPType;				/* possible values from @GPIO_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;
} DMA_Config_t;

/**
 * This is a Handle structure for a DMA controller
 */
typedef struct {
	DMA_RegDef_t *pDMAx; 		/* This holds the base address of the DMA controller */
	DMA_Config_t DMA_Config;	/* This holds DMA configuration settings */
} DMA_Handle_t;


/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void DMA_Init(DMA_Handle_t *pDMAHandle);
void DMA_DeInit(DMA_RegDef_t *pDMAx);

/**
 * Data read and write
 */

/**
 * IRQ Configuration and ISR handling
 */
void DMA_IRQHandling(uint8_t pinNumber);


#endif /* INC_STM32F407XX_DMA_DRIVER_H_ */
