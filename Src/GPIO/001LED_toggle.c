/*
 * 001LED_toggle.c
 *
 *  Created on: May 26, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx.h"
#define PP_MODE			0
#define OD_MODE			1

void delay(void) {
	for(int i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t GpioLed;
	uint8_t mode = PP_MODE;

	if(mode == PP_MODE) {
		GpioLed.pGPIOx = GPIOD;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
		GPIO_Init(&GpioLed);

		while(1) {
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
			delay();
		}
	} else if(mode == OD_MODE) {
		GpioLed.pGPIOx = GPIOD;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pin can be pulled to ground, but not high in Open drain mode with no PUPD
																  // Due to internal resistor, small current will flow and hard to see LED if PU enabled
																  // Need to use external resistor to get full brightness

		GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
		GPIO_Init(&GpioLed);

		while(1) {
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
			delay();
		}
	}

}
