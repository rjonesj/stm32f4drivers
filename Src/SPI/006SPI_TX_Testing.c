/*
 * 006SPI_TX_Testing.c
 *
 *  Created on: Jun 3, 2020
 *      Author: rjonesj
 */

/*
 * Test the SPI_SendData API to send the string "Hello world!" with the following configurations
 * 1. SPI-2 Master mode
 * 2. SCLK = max possible
 * 3. DFF = 0 and DFF = 1
 *
 * SPI2 pins
 * PB14 --> MISO
 * PB15 --> MOSI
 * PB13 --> SCLK
 * PB12 --> NSS
 *
 * ALT function mode: 5
 */

# include "stm32f407xx.h"
# include <string.h>

void SPI2_GPIOInit(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //Generates SCLK of 8 Mhz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_16BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}

int main(void) {
	char user_data[] = "Hello World!";

	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	//Configure SPI2 peripheral
	SPI2_Init();

	//Enable the Internal slave select (Sets NSS to high)
	SPI_SSIConfig(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//Send the data over MOSI line
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
