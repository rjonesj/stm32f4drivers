/*
 * 002LED_Button_ext.c
 *
 *  Created on: May 26, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx.h"
#define HIGH			1
#define LOW				0
#define BTN_PRESSED		LOW

void delay(void) {
	for(int i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t GpioLed, GpioButton;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	//Configure GPIO Handler for external button on PB12
	GpioButton.pGPIOx = GPIOB;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //No internal resistor required as one is built into the button

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
	GPIO_Init(&GpioButton);


	while(1) {
		if(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED) {
			delay(); //de-bounce button
//			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
			GPIO_WriteToOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber, HIGH);
		} else {
			GPIO_WriteToOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber, LOW);
		}
	}

}
