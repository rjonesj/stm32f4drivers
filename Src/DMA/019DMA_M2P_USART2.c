/*
 * 019DMA_M2P_USART2.c
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

/*
 * Test sending data over USART2 peripheral using Memory to Peripheral (M2P) DMA transfer after user button is pressed.
 * The string "Hello world!" will be sent to the VCOM port on the PC with the following configurations
 *
 * 1. USART2
 * 2. Baudrate: 115200bps
 * 3. Frame format: 1 stop bit, 8 bits, no parity
 *
 *
 * USART2 pins			PC
 * PD5  (TX)  	--->	RX
 * PD6  (RX)    --->	TX
 * 5V			--->	5V
 * GND			--->	GND
 *
 * ALT function mode: 7
 */

#include <stdint.h>
#include "stm32f407xx.h"

void USART2_GPIOInit(void);
void USART2_Init(void);
void GPIO_ButtonInit(void);
void DMA1_Init(void);
void sendSomeData(void) ;

void HT_Complete_calbback(void);
void FT_Complete_calbback(void);
void TE_Complete_calbback(void);
void DME_Complete_calbback(void);
void FE_Complete_calbback(void);

USART_Handle_t USART2Handle;
DMA_Handle_t DMA1Handle;
char dataStream[] = "Hello World!\n";

#define is_it_HT() 		DMA1->HISR & ( 1 << DMA_HISR_HTIF6)
#define is_it_FT() 		DMA1->HISR & ( 1 << DMA_HISR_TCIF6)
#define is_it_TE() 		DMA1->HISR & ( 1 << DMA_HISR_TEIF6)
#define is_it_FE() 		DMA1->HISR & ( 1 << DMA_HISR_FEIF6)
#define is_it_DME()		DMA1->HISR & ( 1 << DMA_HISR_DMEIF6)


void USART2_GPIOInit(void) {
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOD;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//USART2_TX (PD5)
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&USARTPins);

	//USART2_RX (PD6)
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
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
	//Configure DMA stream
	DMA1Handle.DMA_Config.sourceAddress = (uint32_t *)dataStream;
	DMA1Handle.DMA_Config.destAddress = &USART2Handle.pUSARTx->DR;
	DMA1Handle.DMA_Config.len = sizeof(dataStream);
	DMA1Handle.DMA_Config.transferDirection = DMA_DIRECTION_M2P;
	DMA1Handle.DMA_Config.memDataSize = DMA_DATA_SIZE_BYTE;
	DMA1Handle.DMA_Config.memIncrementMode = ENABLE;
	DMA1Handle.DMA_Config.periphDataSize = DMA_DATA_SIZE_BYTE;
	DMA1Handle.DMA_Config.periphIncrementMode = DISABLE;
	DMA1Handle.DMA_Config.fifoMode = ENABLE;
	DMA1Handle.DMA_Config.fifoThreshold = DMA_FIFO_THLD_FULL;
	DMA1Handle.DMA_Config.circularMode = DISABLE;
	DMA1Handle.DMA_Config.priority = DMA_PRIORITY_LOW;
	DMA1Handle.DMA_Config.channel = DMA_CHANNEL_4;

	//Configure DMA for M2P transfer over USART2 TX (Stream 6, Channel 4)
	DMA1Handle.pDMAx = DMA1;
	DMA1Handle.pDMAStream = DMA1_Stream6;
	DMA_Init(&DMA1Handle);
}

void sendSomeData(void) {
	USART_SendData(&USART2Handle, (uint8_t *)dataStream, sizeof(dataStream));
}

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

int main(void) {
	GPIO_ButtonInit();
	USART2_GPIOInit();
	USART2_Init();
	while(1);
	return 0;
}

//IRQ Handler for user button global interrupt
void EXTI0_IRQHandler(void) {
	//debounce button
	delay();

	//Send data over USART2
//	sendSomeData();

	//Initialize DMA stream for transfer
	DMA1_REG_RESET();
	DMA1_Init();
	enable_dma_stream(&DMA1Handle);
	dma_interrupt_configuration(&DMA1Handle);

	//Enable DMAT bit on USART2 (Start transfer to TX buffer)
	USART2Handle.pUSARTx->CR3 |= (1 << 7);

	GPIO_IRQHandling(GPIO_PIN_NO_0); //Clear pending event from EXTI line
}

void HT_Complete_calbback(void) {

}

void FT_Complete_calbback(void) {
	//Reset number of items to send to
	DMA1Handle.pDMAStream->NDTR = sizeof(dataStream);
	//Stop USART DMA requests
	USART2Handle.pUSARTx->CR3 &= ~(1 << 7);
	//Enable DMA stream
	enable_dma_stream(&DMA1Handle);
}

void TE_Complete_calbback(void) {
	while(1);
}

void FE_Complete_calbback(void) {
	while(1);
}

void DME_Complete_calbback(void) {
	while(1);
}

//IRQ Handler for DMA1 stream6 global interrupt
void DMA1_Stream6_IRQHandler(void) {
	if(is_it_HT()) {
		//clear interrupt flag
		DMA1->HIFCR |= (1 << DMA_HIFCR_CHTIF6);
		HT_Complete_calbback();
	} else if(is_it_FT()) {
		DMA1->HIFCR |= (1 << DMA_HIFCR_CTCIF6);
		FT_Complete_calbback();
	} else if(is_it_TE()) {
		DMA1->HIFCR |= (1 << DMA_HIFCR_CTEIF6);
		TE_Complete_calbback();
	} else if(is_it_FE()) {
		DMA1->HIFCR |= (1 << DMA_HIFCR_CFEIF4);
		FE_Complete_calbback();
	} else if(is_it_DME()) {
		DMA1->HIFCR |= (1 << DMA_HIFCR_CDMEIF6);
		DME_Complete_calbback();
	}
}
