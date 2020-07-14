/*
 * constants.h
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_


/**
 * Function modifier macros
 */
#define __vo volatile
#define __weak __attribute__((weak))

/**
 * Generic Macros
 */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


#endif /* INC_CONSTANTS_H_ */
