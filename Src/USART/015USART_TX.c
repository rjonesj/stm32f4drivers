/*
 * 015_USART_TX.c
 *
 *  Created on: Jun 26, 2020
 *      Author: rjonesj
 */

/*
 * Test the USART_SendData API to send the string "Hello world!" with the following configurations
 * 1. USART1
 * 2. Baudrate: 115200bps
 * 3. Frame format: 1 stop bit, 8 bits, no parity
 *
 * USART1 pins
 * PB6 --> TX
 * PB7 --> RX
 *
 * Arduino
 * 0 --> RX
 * 1 --> TX
 *
 * Connect Tx to Rx and Rx to Tx
 * ALT function mode: 7
 */

# include "stm32f407xx.h"
# include <string.h>

USART_Handle_t USART1Handle;

void USART1_GPIOInit(void) {
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOB;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//USART1_TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&USARTPins);

	//USART1_RX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&USARTPins);
}

void USART1_Init(void) {
	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART1Handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GpioButton;

	//Configure Button
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //No internal resistor required as one is built into the button
	GPIO_Init(&GpioButton);
}

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

int main(void) {
	char user_data[] = "Hello World!\n";

	//Initialize Button
	GPIO_ButtonInit();

	//This function is used to initialize the GPIO pins to behave as USART1 pins
	USART1_GPIOInit();

	//Configure USART1 peripheral
	USART1_Init();

	while(1) {
		//Wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay(); // debounce button

		//Enable the USART1 peripheral
		USART_PeripheralControl(USART1, ENABLE);

		//Send the data over USART line
		USART_SendData(&USART1Handle, (uint8_t*)user_data, strlen(user_data));

		//Disable the USART1 peripheral
		USART_PeripheralControl(USART1, DISABLE);
	}

	return 0;
}
