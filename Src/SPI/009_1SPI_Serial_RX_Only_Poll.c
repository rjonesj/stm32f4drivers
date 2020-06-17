/*
 * 009SPI_Serial_RX_Only_Poll.c
 *
 *  Created on: Jun 9, 2020
 *      Author: rjonesj
 */

/*
 * When button on master is pressed, master will wait for incoming message sent from Arduino serial input.
 * Message can be sent using the Serial monitor using Carriage return line ending.
 * Message will be stored in the RcvBuff.
 * Upload 003SPISlaveUartReadOverSPI to Arduino to be connected as slave SPI device.
 *
 * Enable semihosting to view printf debugging statements
 * 	1. Exclude syscalls.c and sysmem.c from build
 * 	2. Goto Project > Properties. Select C/C++ Build > Settings. Select MCU GCC Linker > Libraries and add rdimon.
 * 	3. Select MCU GCC Linker > Miscellaneous. Add -specs=rdimon.specs to Other flags.
 * 	4. Open run configuration for debug mode. Select OpenOCD Debug probe.
 * 	5. Select Startup tab and add monitor arm semihosting enable to Initialization commands.
 *
 * 1. Use SPI RXONLY mode
 * 2. STM32F4DISCOVERY board will be in master mode and Arduino Uno will be configured for SPI slave mode
 * 3. DFF = 0
 * 4. Use Hardware slave management (SSM = 0)
 * 5. SCLK speed = 1MHz, fclk = 16MHz
 *
 * STM32F4DISCOVERY SPI2 pins
 * PB15 --> MOSI
 * PB13 --> SCLK
 * PB14 --> MISO
 * PB12 --> NSS
 *
 * ALT function mode: 5
 *
 *
 * Arduino UNO SPI pins
 * 11 --> MOSI
 * 13 --> SCLK
 * 12 --> MISO
 * 10 --> NSS
 */

# include "stm32f407xx.h"
# include <string.h>
# include <stdio.h>

extern void initialise_monitor_handles();

#define RX_START			0xF1
#define BUFF_LEN			100

SPI_Handle_t SPI2Handle;
uint8_t RcvBuff[BUFF_LEN];
uint8_t dummy_write = 0;
uint8_t readByte;
uint8_t rxContFlag = RESET;
uint8_t rcv_start;
uint8_t i;

void delay(void) {
	for(int i = 0; i < 500000/500; i++);
}

void SPI2_GPIOInit(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //Enable PU to transmit to slave device
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void) {
	SPI2Handle.pSPIx = SPI2;
	//The AVR implementation of SPI has no buffering, so slave is not fast enough to write to SPDR before master starts to shift it out
	//Full Duplex configuration will be used to transfer the data for this example
//	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_SIMPLEX_RXONLY;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16; //1MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DS;

	SPI_Init(&SPI2Handle);
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

int main(void) {
	initialise_monitor_handles();

	printf("Application is running...\n");

	//Initialize button
	GPIO_ButtonInit();

	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	//Configure SPI2 peripheral
	SPI2_Init();

	/*
	* Enable SSOE (slave select output enable)
	* The NSS pin will be automatically managed by the hardware.
	* When SPE=1, NSS will be pulled to low
	* When SPE=0, NSS will be pulled to high
	*/
	SPI_SSOEConfig(SPI2Handle.pSPIx, ENABLE);

	/**
	 * Enable SPI2 IRQ number on NVIC
	 */
	SPI_IRQConfig(IRQ_NO_SPI2, ENABLE);

	printf("SPI Initialized...\n");

	/**
	 * Main loop
	 */
	while(1) {
//		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)) {
//			delay(); //debounce button
//			while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)); //wait until button has been released

			//Reset buffer
			RcvBuff[0] = '\0';

			//Read data until reception is complete
			i = 0;
			rcv_start = 0;
			rxContFlag = SET;
			SPI_PeripheralControl(SPI2Handle.pSPIx, ENABLE);

			while(rxContFlag == SET) {
				SPI_SendData(SPI2Handle.pSPIx, &dummy_write, 1);
				delay();	//Allow time for slave to process
				SPI_ReceiveData(SPI2Handle.pSPIx, &readByte, 1);
				delay();	//Allow time for slave to process

				if(readByte == RX_START) {
					rcv_start = 1;
				} else {
					if(rcv_start) {
						if(readByte == '\r' || i == BUFF_LEN) {
							rxContFlag = RESET;
							rcv_start = 0;
							//Terminate String
							if(i != BUFF_LEN) {
								RcvBuff[i] = '\0';
							} else {
								RcvBuff[BUFF_LEN-1] = '\0';
							}
						} else {
							RcvBuff[i++] = readByte;
						}
					}
				}
			}

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2Handle.pSPIx, DISABLE);

			printf("Message Received: %s\n", RcvBuff);
		}
//	}

	return 0;
}
