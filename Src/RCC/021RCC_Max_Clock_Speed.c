/*
 * 021RCC_Max_Clock_Speed.c
 *
 *  Created on: Jul 17, 2020
 *      Author: rjonesj
 *
 *  Application configures the system and bus clocks to maximum speed using the main PLL (Phase-locked loop)
 *
 *  AHB 168MHz
 *  APB1 42MHz
 *  APB2 84 MHz
 *
 *  System clock signal (PLL) will be sent over PA8 to read using a logic analyzer or oscilloscope.
 */


#include "stm32f407xx.h"
#include <string.h>

int main(void)
{

	//1. Set System and Bus Clocks to max frequency
	ErrorStatus status = setClocksToMaxFrequency();
	if(status == ERROR) {
        // Do something to indicate error clock configuration
		while(1);
	}

	//2. Configure PA8 to AF0 mode to behave as MCO signal
    GPIO_Handle_t MCO1Pin;

	//Set all member elements to 0
	memset(&MCO1Pin,0,sizeof(MCO1Pin));

    MCO1Pin.pGPIOx = GPIOA;
    MCO1Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    MCO1Pin.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    MCO1Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    MCO1Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;

    GPIO_Init(&MCO1Pin);

    while (1);
}
