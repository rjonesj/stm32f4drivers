/*
 * 016_USART_Case.c
 *
 *  Created on: Jun 27, 2020
 *      Author: rjonesj
 */

/*
 * Test the USART interrupt APIs to send the strings to Arduino over UART communication.
 * For every message STM32 board sends, Arduino code will change the case of alphabet chars
 * and send message back to the STM32 board.  STM32 board should capture the reply and display using semi-hosting.
 * 1. USART1
 * 2. Baudrate: 115200bps
 * 3. Frame format: 1 stop bit, 8 bits, no parity
 *
 * Upload 002UARTTxString.ino to connected Arduino
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
# include <stdio.h>

USART_Handle_t USART1Handle;

//Array to hold messages to Arduino
char *msg[3] = {"helloHihelloHi123\n", "Good morning Sunshine!\n", "My name is Fred.\n" };
//Reply from Arduino will be stored here
char rx_buf[1024];
//This flag indicates reception completion
uint8_t rxComplete = RESET;
uint8_t g_data = 0;

extern void initialise_monitor_handles();

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
	USART1Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
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
	uint32_t cnt = 0;

	initialise_monitor_handles();

	//Initialize Button
	GPIO_ButtonInit();

	//This function is used to initialize the GPIO pins to behave as USART1 pins
	USART1_GPIOInit();

	//Configure USART1 peripheral
	USART1_Init();

	//Enable interrupts for USART1 on NVIC
	USART_IRQConfig(IRQ_NO_USART1, ENABLE);

	//Enable USART1 peripheral
	USART_PeripheralControl(USART1, ENABLE);

	printf("Application is running...\n");

	while(1) {
		//Wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay(); // debounce button

		//Store the next message index and ensure value is not greater than msg len
		cnt = cnt % 3;

		//Enable the reception in interrupt mode
		while(USART_ReceiveDataIT(&USART1Handle, (uint8_t *)rx_buf, strlen(msg[cnt])) != USART_READY);

		//Send the msg indexed by cnt in blocking mode
		USART_SendData(&USART1Handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));
		printf("Transmitted: %s\n", msg[cnt]);

		//Wait until all bytes are received from Arduino
		//When all bytes are received, rxComplete will be SET in application callback
		while(rxComplete != SET);

		//Set last byte to null for proper string termination
		rx_buf[strlen(msg[cnt]) +1] = '\0';

		//Print what was received from Arduino
		printf("Received: %s\n", rx_buf);

		//Invalidate the flag
		rxComplete = RESET;

		//Increment to next message index
		cnt++;
	}

	return 0;
}

void USART1_IRQHandler(void) {
	USART_IRQHandling(&USART1Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEvent) {
	if(appEvent == USART_EVENT_RX_CMPLT) {
		rxComplete = SET;
	}
}
