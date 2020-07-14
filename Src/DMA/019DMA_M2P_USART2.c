/*
 * 019DMA_M2P_USART2.c
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

#include <stdint.h>
#include "stm32f407xx.h"

void USART2_GPIOInit(void);
void USART2_Init(void);
void GPIO_ButtonInit(void);
void DMA1_Init(void);
void sendSomeData(void) ;

USART_Handle_t USART2Handle;

void USART2_GPIOInit(void) {
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOD;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//USART2_TX (PA2, PD5)
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&USARTPins);

	//USART2_RX (PA3, PD6)
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USARTPins);
}

void USART2_Init(void) {
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2Handle);

	//Enable peripheral
	USART_PeripheralControl(USART2, ENABLE);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GpioButton;

	//Configure User button for interrupts (PA0)
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //No internal resistor required as one is built into the button
	GPIO_Init(&GpioButton);

	//IRQ Configurations
	NVIC_IRQConfig(IRQ_NO_EXTI0, ENABLE);
}

void DMA1_Init(void) {

}

void sendSomeData(void) {
	char someData[] = "Hello World!\n";
	USART_SendData(&USART2Handle, (uint8_t *)someData, sizeof(someData));
}

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

int main(void) {
	GPIO_ButtonInit();
	USART2_GPIOInit();
	USART2_Init();
	DMA1_Init();
	while(1);
	return 0;
}

void EXTI0_IRQHandler(void) {
	//debounce button
	delay();
	sendSomeData();
	GPIO_IRQHandling(GPIO_PIN_NO_0); //Clear pending event from EXTI line
}
