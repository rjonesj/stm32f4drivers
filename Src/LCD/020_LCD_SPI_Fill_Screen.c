/*
 * 020_LCD_SPI_Fill_Screen.c
 *
 *  Created on: Jul 15, 2020
 *      Author: rjonesj
 */

/*
 * Program turns on a 240x320 pixel TFT LCD screen using ILI9341 driver with 4 wire SPI and tests full screen refreshes using alternate colors.
 * This a sample application to test different methods and clock speeds to achieve fast refresh rates.
 *
 * SPI1 pins	(42 MHz)
 * PA6 --> MISO
 * PA7 --> MOSI
 * PA5 --> SCLK
 * PA4 --> NSS
 * ALT function mode: 5
 *
 * LCD pins
 * PA1 --> DC
 * PA2 --> CS
 * PA0 --> RESET
 * Output Mode
 *
 * Connections
 * LCD					STM32
 * SDO(MISO)	-->		PA6
 * LED			-->		3V
 * SCK			-->		PA5
 * SDI(MOSI)	-->		PA7
 * DC			-->		PA1
 * RESET		-->		PA0
 * CS			-->		PA2
 * GND			-->		GND
 * VCC			-->		3V
 */

# include "stm32f407xx.h"
# include "ili9341_driver.h"
# include <string.h>

#define LCD_DC		GPIO_PIN_NO_1
#define LCD_CS		GPIO_PIN_NO_2
#define LCD_RESET 	GPIO_PIN_NO_0
#define TEST_PIN 	GPIO_PIN_NO_3
#define X_PIXELS	240
#define Y_PIXELS	320
#define DMA_MAX_NDTR	0xFFFF-1

GPIO_Handle_t LCDPins;
SPI_Handle_t SPI1Handle;
DMA_Handle_t DMA2Handle;
ILI9341_Handle_t ILIHandle;

void LCD_GPIOInit(void) {
	LCDPins.pGPIOx = GPIOA;
	LCDPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LCDPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LCDPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LCDPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//DC
	LCDPins.GPIO_PinConfig.GPIO_PinNumber = LCD_DC;
	GPIO_Init(&LCDPins);

	//CS
	LCDPins.GPIO_PinConfig.GPIO_PinNumber = LCD_CS;
	GPIO_Init(&LCDPins);

	//RESET
	LCDPins.GPIO_PinConfig.GPIO_PinNumber = LCD_RESET;
	GPIO_Init(&LCDPins);

	//TEST
	LCDPins.GPIO_PinConfig.GPIO_PinNumber = TEST_PIN;
	GPIO_Init(&LCDPins);
}

void SPI1_GPIOInit(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI1_Init(void) {
	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI1Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_DS;

	SPI_Init(&SPI1Handle);

	//Enable Tx Buffer DMA requests
	SPI1Handle.pSPIx->CR2 |= (1 << SPI_CR2_TXDMAEN);
}

void DMA2_Init(void) {
	//Configure DMA stream
	DMA2Handle.DMA_Config.destAddress = (uint32_t *)&SPI1Handle.pSPIx->DR;
	DMA2Handle.DMA_Config.transferDirection = DMA_DIRECTION_M2P;
	DMA2Handle.DMA_Config.memDataSize = DMA_DATA_SIZE_HALF_WORD;
	DMA2Handle.DMA_Config.memIncrementMode = DISABLE;
	DMA2Handle.DMA_Config.periphDataSize = DMA_DATA_SIZE_BYTE;
	DMA2Handle.DMA_Config.periphIncrementMode = DISABLE;
	DMA2Handle.DMA_Config.fifoMode = ENABLE;
	DMA2Handle.DMA_Config.fifoThreshold = DMA_FIFO_THLD_FULL;
	DMA2Handle.DMA_Config.circularMode = DISABLE;
	DMA2Handle.DMA_Config.priority = DMA_PRIORITY_LOW;
	DMA2Handle.DMA_Config.channel = DMA_CHANNEL_3;
	DMA2Handle.DMA_Config.stream = DMA_CHANNEL_3;

	//Configure DMA2 for M2P transfer over SPI1 TX (Stream 3, Channel 3)
	DMA2Handle.pDMAx = DMA2;
	DMA2Handle.pDMAStream = DMA2_Stream3;
	DMA_Init(&DMA2Handle);
}

void ILI9341_Driver_Init(void) {
	ILIHandle.intfMode = ILI9341_MODE_4WIRE_8BIT_SERIAL;
	ILIHandle.pLCDPins = &LCDPins;
	ILIHandle.xPixels = X_PIXELS;
	ILIHandle.yPixels = Y_PIXELS;

	ILIHandle.ILI9341_SPI_Config.lcdCSPin = LCD_CS;
	ILIHandle.ILI9341_SPI_Config.lcdDCPin = LCD_DC;
	ILIHandle.ILI9341_SPI_Config.lcdResetPin = LCD_RESET;
	ILIHandle.ILI9341_SPI_Config.enableDMA = ENABLE;
	ILIHandle.ILI9341_SPI_Config.pDMAHandle = &DMA2Handle;
	ILIHandle.ILI9341_SPI_Config.dmaMaxTransfer = DMA_MAX_NDTR;

	ILI9341_Init(&ILIHandle);
}

int main(void) {
	//Set System and Bus Clocks to max frequency
	ErrorStatus status = setClocksToMaxFrequency();
	if(status == ERROR) {
        // Do something to indicate error clock configuration
		while(1);
	}

	//Initialize LCD pins
	LCD_GPIOInit();

	//This function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInit();

	//Configure SPI1 peripheral
	SPI1_Init();

	/*
	* Enable SSOE (slave select output enable)
	* The NSS pin will be automatically managed by the hardware.
	* When SPE=1, NSS will be pulled to low
	* When SPE=0, NSS will be pulled to high
	*/
	SPI_SSOEConfig(SPI1, ENABLE);

	//Enable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, ENABLE);

	//Configure DMA Stream
	DMA2_Init();

	//Initialize ILI9341 driver
	ILI9341_Driver_Init();

	/* Set rotation to landscape */
	ILI9341_Set_Rotation(3);
	ILI9341_Set_Address(0, 0, Y_PIXELS-1, X_PIXELS-1);

	//Perform 60 alternating color screen refreshes
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_RESET);
	for(int i = 0; i < 30; i++) {
		ILI9341_Fill_Screen(ILI9341_COLOR_BLUE);
		ILI9341_Fill_Screen(ILI9341_COLOR_RED);
	}
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_RESET);

	//Disable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);

	return 0;
}
