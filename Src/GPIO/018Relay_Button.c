/*
 * 018Relay_Button.c
 *
 *  Created on: Jul 1, 2020
 *      Author: rjonesj
 *
 *  Demonstrates using a 5V relay to control the power to another device of higher voltage than the STM32.
 *  Device will turn on when button is pressed, and turn off when pressed again.
 *  Device can be small battery operated appliance such as a fan for demonstration / safety purposes that should be in ON state.
 *  Cut the negative wire and attach one end to COM and the other to NO on the relay.
 *
 *  Connections
 *  Relay		STM32
 *  VCC  <-----> 5V
 *  GND  <-----> GND
 *  IN   <-----> PD10
 */

#include "stm32f407xx.h"
#define HIGH			1
#define LOW				0
#define BTN_PRESSED		HIGH

GPIO_Handle_t GpioRelay, GpioButton;
uint8_t state = LOW;

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

void GPIO_Init_Relay_ON() {
	GpioRelay.pGPIOx = GPIOD;
	GpioRelay.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioRelay.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioRelay.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioRelay.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioRelay.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIO_Init(&GpioRelay);

	GPIO_WriteToOutputPin(GpioRelay.pGPIOx, GpioRelay.GPIO_PinConfig.GPIO_PinNumber, HIGH);

	state = HIGH;
}

void GPIO_Init_Relay_OFF() {
	GPIO_WriteToOutputPin(GpioRelay.pGPIOx, GpioRelay.GPIO_PinConfig.GPIO_PinNumber, LOW);

	GPIO_DeInit(GpioRelay.pGPIOx);

	GPIO_PeriClockControl(GpioRelay.pGPIOx, DISABLE);

	state = LOW;
}

int main(void) {
	//Configure GPIO Handler for user button
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //No internal resistor required as one is built into the button

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
	GPIO_Init(&GpioButton);

	while(1) {
		//Wait for button to be pressed
		if(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED) {
			delay(); //de-bounce button
			//Turn device OFF if ON
			if(state == HIGH) {
				GPIO_Init_Relay_OFF();
			//Turn device ON if OFF
			} else if(state == LOW) {
				GPIO_Init_Relay_ON();
			}
		}
	}
}
