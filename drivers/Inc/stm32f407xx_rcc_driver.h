/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jun 25, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

uint32_t RCC_GetPLLOutputClock();
uint32_t RCC_GetPCLK1_Value();
uint32_t RCC_GetPCLK2_Value();


#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
