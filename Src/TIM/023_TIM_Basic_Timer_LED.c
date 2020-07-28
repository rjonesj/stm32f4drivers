/*
 * 023_TIM_Basic_Timer_LED.c
 *
 * Program starts a basic timer which triggers an update event every 100ms and toggles an LED after clearing event.
 *
 *  Created on: Jul 27, 2020
 *      Author: rjonesj
 */

# include "stm32f407xx.h"

GPIO_Handle_t GpioLed;
TIMB_Handle_t timb6Handle;

//Configure psc = 24, arr = 64000-1 to produce update events at 100 ms intervals when using HSI 16 MHz clock
#define PSC 24
#define ARR 63999

void GPIOLED_Init(void) {
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);
}

void TIMB6_Init(void) {
	timb6Handle.pTIMBx = TIM6;
	timb6Handle.pTIMB_Config.prescaler = PSC;
	timb6Handle.pTIMB_Config.autoReloadValue = ARR;

	TIMB_Init(&timb6Handle);
}

int main(void) {
	//Initialize LED GPIO
	GPIOLED_Init();

	//Initialize Basic Timer TIM6
	TIMB6_Init();

	//Start timer
	TIMB_Start(timb6Handle.pTIMBx);

	while(1) {
	  //Wait for update event
	  while(!(TIM6->SR & (1 << TIMx_SR_UIF))) ;
	  //Clear interrupt flag
	  TIM6->SR = 0;
	  //Toggle LED
	  GPIOD->ODR ^= (1 << 15);
	}
}
