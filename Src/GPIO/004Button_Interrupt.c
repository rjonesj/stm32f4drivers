/*
 * 004Button_Interrupt.c
 *
 *  Created on: May 31, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx.h"
#include <string.h>

GPIO_Handle_t GpioLed, GpioButton;

void delay(void) {
	for(int i = 0; i < 500000/2; i++);
}

int main(void) {
	//Set all member elements to 0
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));

	//Configure GPIO for external LED
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	//Configure GPIO Handler for user button
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
	GPIO_Init(&GpioButton);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}

void EXTI9_5_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5); //Clear pending event from EXTI line
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
}
