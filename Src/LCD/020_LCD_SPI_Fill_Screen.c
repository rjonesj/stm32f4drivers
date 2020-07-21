/*
 * 020_LCD_SPI_Fill_Screen.c
 *
 *  Created on: Jul 15, 2020
 *      Author: rjonesj
 */

/*
 * Program turns on a 320X240 pixel TFT LCD screen using ILI9341 driver with 4 wire SPI and tests full screen refreshes using alternate colors.
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
# include <string.h>

#define LCD_DC		GPIO_PIN_NO_1
#define LCD_CS		GPIO_PIN_NO_2
#define LCD_RESET	GPIO_PIN_NO_0
#define TEST_PIN	GPIO_PIN_NO_3

#define BLUE        	0x001F
#define RED         	0xF800

static unsigned int X_SIZE = 240;
static unsigned int Y_SIZE = 320;
uint16_t color = 0x001F;
uint16_t numOfItemsToTransfer = 0xFFFF;

GPIO_Handle_t LCDPins;
SPI_Handle_t SPI1Handle;
DMA_Handle_t DMA2Handle;

void LCD_GPIOInit(void) {
	LCDPins.pGPIOx = GPIOA;
	LCDPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LCDPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LCDPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LCDPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

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
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

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
	SPI1Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //Generates SCLK of 8 Mhz
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
	DMA2Handle.DMA_Config.sourceAddress = (uint32_t *)&color;
	DMA2Handle.DMA_Config.destAddress = (uint32_t *)&SPI1Handle.pSPIx->DR;
	DMA2Handle.DMA_Config.len = numOfItemsToTransfer;
	DMA2Handle.DMA_Config.transferDirection = DMA_DIRECTION_M2P;
	DMA2Handle.DMA_Config.memDataSize = DMA_DATA_SIZE_HALF_WORD;
	DMA2Handle.DMA_Config.memIncrementMode = DISABLE;
	DMA2Handle.DMA_Config.periphDataSize = DMA_DATA_SIZE_BYTE;
	DMA2Handle.DMA_Config.periphIncrementMode = DISABLE;
	DMA2Handle.DMA_Config.fifoMode = ENABLE;
	DMA2Handle.DMA_Config.fifoThreshold = DMA_FIFO_THLD_1_4_FULL;
	DMA2Handle.DMA_Config.circularMode = DISABLE;
	DMA2Handle.DMA_Config.priority = DMA_PRIORITY_LOW;
	DMA2Handle.DMA_Config.channel = DMA_CHANNEL_3;

	//Configure DMA2 for M2P transfer over SPI1 TX (Stream 3, Channel 3)
	DMA2Handle.pDMAx = DMA2;
	DMA2Handle.pDMAStream = DMA2_Stream3;
	DMA_Init(&DMA2Handle);
}

static ErrorStatus setClocksToMaxFrequency() {
	ErrorStatus clockStatus = ERROR;

	// Resets the clock configuration to the default reset state
	RCC_DeInit();

	uint32_t *pRccCfgrReg = (uint32_t*) (RCC_BASEADDR + 0x08);

	//1. Set the RCC clock configuration register MCO1 to PLL
	*pRccCfgrReg &= ~(0x3 << RCC_CFGR_MCO1); // clear bit 21 - 22 positions
	*pRccCfgrReg |= (0x3 << RCC_CFGR_MCO1); // set bit 21 - 22 positions to PLL

	// Enable external crystal (HSE)
	RCC_HSEConfig(RCC_HSE_ON);
	// Wait until HSE ready to use or not
	ErrorStatus hseStatus = RCC_WaitForHSEStartUp();

	if (hseStatus == SUCCESS)
	{
		// Configure the PLL for 168MHz SysClk and 48MHz for USB OTG, SDIO
		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
		// Enable PLL
		RCC_PLLCmd(ENABLE);
		// Wait until main PLL clock ready
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == FS_RESET);

		// Set flash latency
		FLASH_SetLatency(FLASH_Latency_5);

		// AHB 168MHz
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		// APB1 42MHz
		RCC_PCLK1Config(RCC_HCLK_Div4);
		// APB2 84 MHz
		RCC_PCLK2Config(RCC_HCLK_Div2);

		// Set SysClk using PLL
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		//Clocks configured successfully
		clockStatus = SUCCESS;
	}

	return clockStatus;
}

/**
 * ILI9341 Driver functions
 */

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

static void Before_Sending_Data() {
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_DC, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_RESET);
}

static void Before_Sending_Command() {
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_DC, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_RESET);
}

void ILI9341_SPI_Send(unsigned char data)
{
	SPI_SendData(SPI1Handle.pSPIx, &data, 1);
}

static void ILI9341_Send_Command(unsigned char command) {
	Before_Sending_Command();
	ILI9341_SPI_Send(command);
}

static void ILI9341_Send_Data(unsigned char data) {
	Before_Sending_Data();
	ILI9341_SPI_Send(data);
}

void ILI9341_SPI_Send_32(unsigned char command, unsigned long data) {
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_DC, GPIO_PIN_RESET);
	ILI9341_SPI_Send(command);

	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_DC, GPIO_PIN_SET);
	ILI9341_SPI_Send(data >> 24);
	ILI9341_SPI_Send(data >> 16);
	ILI9341_SPI_Send(data >> 8);
	ILI9341_SPI_Send(data);

}

void ILI9341_Reset() {
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_RESET, GPIO_PIN_RESET);
	delay();
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_RESET, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_RESET);
	delay();
	ILI9341_Send_Command(0x01);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_SET);
}

void ILI9341_Init() {
	/* Reset The Screen */
	ILI9341_Reset();
	ILI9341_Send_Command(0x01);

	/* Power Control A */
	ILI9341_Send_Command(0xCB);
	ILI9341_Send_Data(0x39);
	ILI9341_Send_Data(0x2C);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x34);
	ILI9341_Send_Data(0x02);

	/* Power Control B */
	ILI9341_Send_Command(0xCF);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0xC1);
	ILI9341_Send_Data(0x30);

	/* Driver timing control A */
	ILI9341_Send_Command(0xE8);
	ILI9341_Send_Data(0x85);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x78);

	/* Driver timing control B */
	ILI9341_Send_Command(0xEA);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x00);

	/* Power on Sequence control */
	ILI9341_Send_Command(0xED);
	ILI9341_Send_Data(0x64);
	ILI9341_Send_Data(0x03);
	ILI9341_Send_Data(0x12);
	ILI9341_Send_Data(0x81);

	/* Pump ratio control */
	ILI9341_Send_Command(0xF7);
	ILI9341_Send_Data(0x20);

	/* Power Control 1 */
	ILI9341_Send_Command(0xC0);
	ILI9341_Send_Data(0x10);

	/* Power Control 2 */
	ILI9341_Send_Command(0xC1);
	ILI9341_Send_Data(0x10);

	/* VCOM Control 1 */
	ILI9341_Send_Command(0xC5);
	ILI9341_Send_Data(0x3E);
	ILI9341_Send_Data(0x28);

	/* VCOM Control 2 */
	ILI9341_Send_Command(0xC7);
	ILI9341_Send_Data(0x86);

	/* VCOM Control 2 */
	ILI9341_Send_Command(0x36);
	ILI9341_Send_Data(0x48);

	/* Pixel Format Set */
	ILI9341_Send_Command(0x3A);
	ILI9341_Send_Data(0x55);    //16bit

	ILI9341_Send_Command(0xB1);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x18);

	/* Display Function Control */
	ILI9341_Send_Command(0xB6);
	ILI9341_Send_Data(0x08);
	ILI9341_Send_Data(0x82);
	ILI9341_Send_Data(0x27);

	/* 3GAMMA FUNCTION DISABLE */
	ILI9341_Send_Command(0xF2);
	ILI9341_Send_Data(0x00);

	/* GAMMA CURVE SELECTED */
	ILI9341_Send_Command(0x26); //Gamma set
	ILI9341_Send_Data(0x01); 	//Gamma Curve (G2.2)

	//Positive Gamma  Correction
	ILI9341_Send_Command(0xE0);
	ILI9341_Send_Data(0x0F);
	ILI9341_Send_Data(0x31);
	ILI9341_Send_Data(0x2B);
	ILI9341_Send_Data(0x0C);
	ILI9341_Send_Data(0x0E);
	ILI9341_Send_Data(0x08);
	ILI9341_Send_Data(0x4E);
	ILI9341_Send_Data(0xF1);
	ILI9341_Send_Data(0x37);
	ILI9341_Send_Data(0x07);
	ILI9341_Send_Data(0x10);
	ILI9341_Send_Data(0x03);
	ILI9341_Send_Data(0x0E);
	ILI9341_Send_Data(0x09);
	ILI9341_Send_Data(0x00);

	//Negative Gamma  Correction
	ILI9341_Send_Command(0xE1);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x0E);
	ILI9341_Send_Data(0x14);
	ILI9341_Send_Data(0x03);
	ILI9341_Send_Data(0x11);
	ILI9341_Send_Data(0x07);
	ILI9341_Send_Data(0x31);
	ILI9341_Send_Data(0xC1);
	ILI9341_Send_Data(0x48);
	ILI9341_Send_Data(0x08);
	ILI9341_Send_Data(0x0F);
	ILI9341_Send_Data(0x0C);
	ILI9341_Send_Data(0x31);
	ILI9341_Send_Data(0x36);
	ILI9341_Send_Data(0x0F);

	//EXIT SLEEP
	ILI9341_Send_Command(0x11);

	//TURN ON DISPLAY
	ILI9341_Send_Command(0x29);
	ILI9341_Send_Data(0x2C);
}

void ILI9341_Set_Rotation(unsigned char rotation) {
	ILI9341_Send_Command(0x36);
	switch (rotation) {
	case 0:
		ILI9341_Send_Data(0x48);
		X_SIZE = 240;
		Y_SIZE = 320;
		break;
	case 1:
		ILI9341_Send_Data(0x28);
		X_SIZE = 320;
		Y_SIZE = 240;
		break;
	case 2:
		ILI9341_Send_Data(0x88);
		X_SIZE = 240;
		Y_SIZE = 320;
		break;
	case 3:
		ILI9341_Send_Data(0xE8);
		X_SIZE = 320;
		Y_SIZE = 240;
		break;
	}
}

void ILI9341_Set_Address(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
	unsigned long t;
	t = x1;
	t <<= 16;
	t |= x2;
	ILI9341_SPI_Send_32(0x2A, t); //Column Address Set
	t = y1;
	t <<= 16;
	t |= y2;
	ILI9341_SPI_Send_32(0x2B, t); //Page Address Set
}

void ILI9341_Send_Burst(unsigned short ucolor, unsigned long len) {
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_DC, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2C);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_DC, GPIO_PIN_SET);

//	//Slow SPI single byte method
//	unsigned char high_bit = ucolor >> 8, low_bit = ucolor;
//	for(int i = 0; i < len; i++) {
//		ILI9341_SPI_Send(high_bit);
//		ILI9341_SPI_Send(low_bit);
//	}

	//Faster DMA method
	//Flip the bytes for the little-endian ARM core.
	ucolor = (((ucolor & 0x00FF) << 8) | ((ucolor & 0xFF00) >> 8));
	color = ucolor;
	numOfItemsToTransfer = 0xFFFF-1;
	len = len*2;
	while(len > 0) {
		DMA2_REG_RESET();
		DMA2_Init();
		while(DMA2Handle.pDMAStream->NDTR);
		len -= numOfItemsToTransfer;
		if(len < numOfItemsToTransfer) {
			numOfItemsToTransfer = len;
		}
	}

	GPIO_WriteToOutputPin(LCDPins.pGPIOx, LCD_CS, GPIO_PIN_SET);
}

void ILI9341_Fill_Screen(unsigned int color)
{
	ILI9341_Set_Address(0, 0, X_SIZE-1, Y_SIZE-1);
	ILI9341_Send_Burst(color, (long)X_SIZE * (long)Y_SIZE);
}

/**
 * End ILI9341 Functions
 */

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

	//Turn on the LCD
	ILI9341_Init();

	/* Set rotation to landscape */
	ILI9341_Set_Rotation(3);

	delay();

	//Perform 60 alternating color screen refreshes
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_RESET);
	for(int i = 0; i < 30; i++) {
		ILI9341_Fill_Screen(BLUE);
		ILI9341_Fill_Screen(RED);
	}
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, TEST_PIN, GPIO_PIN_RESET);

	//Disable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);

	return 0;
}
