/*
 * armcortexm4_nvic_driver.h
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

#ifndef INC_ARMCORTEXM4_NVIC_DRIVER_H_
#define INC_ARMCORTEXM4_NVIC_DRIVER_H_

#include <stdint.h>
#include "constants.h"


/**
 * 		ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

/**
 * 		ARM Cortex M4 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

/**
 * 		ARM Cortex M4 Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)


/**
 * 		ARM Cortex M4 Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED	4


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * IRQ Configuration and ISR handling
 */
void NVIC_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void NVIC_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);


#endif /* INC_ARMCORTEXM4_NVIC_DRIVER_H_ */
