/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 25, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;				/* possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;				/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;				/* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;		/* possible values from @GPIO_PUPD */
	uint8_t GPIO_PinOPType;				/* possible values from @GPIO_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/**
 * This is a Handle structure for a GPIO port
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx; 				/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/**
 * This is a Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;
	uint8_t pinNo;
} GPIO_Pin_Handle_t;

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

/**
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0			/* Input mode */
#define GPIO_MODE_OUT 		1			/* Output mode */
#define GPIO_MODE_ALTFN 	2			/* Alternate Function mode */
#define GPIO_MODE_ANALOG 	3			/* Analog Function mode */
#define GPIO_MODE_IT_FT 	4			/* Input Interrupt mode Falling edge trigger */
#define GPIO_MODE_IT_RT 	5			/* Input Interrupt mode Rising edge trigger */
#define GPIO_MODE_IT_RFT 	6			/* Input Interrupt mode Rising edge, Falling Edge trigger */

/**
 * @GPIO_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0			/* Output Push Pull type */
#define GPIO_OP_TYPE_OD		1			/* Output Open Drain type */

/**
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0			/* Output low speed */
#define GPIO_SPEED_MEDIUM	1			/* Output medium speed */
#define GPIO_SPEED_FAST		2			/* Output high speed */
#define GPIO_SPEED_HIGH		3			/* Output very high speed */

/**
 * @GPIO_PUPD
 * GPIO pin pull up / pull down configurations
 */
#define GPIO_NO_PUPD		0			/* No pull up, pull down */
#define GPIO_PIN_PU			1			/* Enable pull up */
#define GPIO_PIN_PD			2			/* Enable pull down */


/**
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDRESS_TO_CODE(x)  ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 : 8)

/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_Pin_Init(GPIO_Handle_t *gpioHandle, uint8_t pinNo);

/**
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/**
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
