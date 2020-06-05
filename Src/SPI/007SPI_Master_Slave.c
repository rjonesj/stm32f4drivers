/*
 * 007SPI_Master_Slave.c
 *
 *  Created on: Jun 4, 2020
 *      Author: rjonesj
 */

/*
 * When button on master is pressed, master should send string of data to connected slave.
 * The data received by slave will be displayed on the serial port
 * 1. Use SPI Full duplex mode
 * 2. STM32F4DISCOVERY board will be in master mode and STM32F3DISCOVERY will be configured for SPI slave mode
 * 3. DFF = 0
 * 4. Use Hardware slave management (SSM = 0)
 * 5. SCLK speed = 2MHz, fclk = 16MHz
 *
 * Master will not receive anything from slave, so MISO does not need to be configured.
 * Unzip resources/stm32f3SPISlaveRx.zip to a directory and compile and upload to a STM32F3Discovery board to be used as a slave device.
 *
 * STM32F4DISCOVERY SPI2 pins
 * PB14 --> MISO
 * PB15 --> MOSI
 * PB13 --> SCLK
 * PB12 --> NSS
 *
 * STM32F3DISCOVERY SPI1 pins
 * PA4 --> NSS
 * PA5 --> SCLK
 * PA6 --> MISO
 * PA7 --> MOSI
 *
 * ALT function mode: 5
 */

# include "stm32f407xx.h"
# include <string.h>

void delay(void) {
	for(int i = 0; i < 500000/2; i++);
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

	//Not required since transmitting only
//	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void) {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //Generates SCLK of 8 Mhz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
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
	char user_data[] = "Hello World!";

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
	 SPI_SSOEConfig(SPI2, ENABLE);

	while(1) {
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay(); //debounce button

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send the data over MOSI line
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}
