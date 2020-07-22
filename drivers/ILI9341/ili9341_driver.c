/*
 * ili9341_driver.c
 *
 *  Created on: Jul 21, 2020
 *      Author: rjonesj
 */

#include "ili9341_driver.h"

GPIO_Handle_t LCDPins;
SPI_Handle_t SPIHandle;
DMA_Handle_t DMAHandle;
uint8_t lcdReset, lcdCS, lcdDC;
uint16_t dmaMaxItemTransfer, xLen, yLen, oxLen, oyLen;

static void delay(void) {
	for(int i = 0; i < 500000; i++);
}

/**
 * Initialization and De-Initialization
 */

/*********************************************************************
 * @fn      		  - ILI9341_Init
 * @brief             - Function initializes library and powers on the ILI9341 device using a configured ILI9341_Handle_t
 *
 * @param[in]         - Address to ILI9341_Handle_t struct
 *
 * @return            - none
 * @Note              - Function must be called first before any others in this library

 */
void ILI9341_Init(ILI9341_Handle_t *pHandle) {
	LCDPins = *pHandle->pLCDPins;
	SPIHandle = *pHandle->pSPIHandle;
	DMAHandle = *pHandle->pDMAHandle;

	lcdReset = pHandle->ILI9341_Config.lcdResetPin;
	lcdCS = pHandle->ILI9341_Config.lcdCSPin;
	lcdDC = pHandle->ILI9341_Config.lcdDCPin;
	xLen = pHandle->ILI9341_Config.xPixels;
	yLen = pHandle->ILI9341_Config.yPixels;
	oxLen = xLen;
	oyLen = yLen;
	dmaMaxItemTransfer = pHandle->ILI9341_Config.dmaMaxTransfer;

	ILI9341_Reset();
	ILI9341_PowerOn();
}

/*********************************************************************
 * @fn      		  - ILI9341_Reset
 * @brief             - Function resets the device to initialize the chip
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Reset() {
	//Set RESET pin from LOW to HIGH
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdReset, GPIO_PIN_RESET);
	delay();
	GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdReset, GPIO_PIN_SET);
	delay();
}


/*********************************************************************
 * @fn      		  - ILI9341_PowerOn
 * @brief             - Function performs software initialization commands and turns on the screen
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_PowerOn() {
	//Enable ILI9341 chip
	ILI9341_ChipSelect(ENABLE);

	/* Reset The Screen */
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

/**
 * High Level user functions
 */

/*********************************************************************
 * @fn      		  - ILI9341_Fill_Screen
 * @brief             - Function fills screen with a given 16 bit color
 *
 * @param[in]         - Pixel color value given in RGB565 format
 * @param[in]         - ENABLE or DISABLE macro to use DMA for data transfer
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Fill_Screen(unsigned int color, uint8_t enableDMA)
{
	ILI9341_Set_Address(0, 0, xLen-1, yLen-1);
	if(enableDMA == ENABLE) {
		ILI9341_Send_Burst_DMA(color, (long)xLen * (long)yLen);
	} else {
		ILI9341_Send_Burst_SPI(color, (long)xLen * (long)yLen);
	}
}

/*
 * ILI9341 Commands
 */

/*********************************************************************
 * @fn      		  - ILI9341_Set_Rotation
 * @brief             - Function defines the read/write scanning direction of frame memory
 *
 * @param[in]         - value from 0 to 3 to set screen orientation writing
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Set_Rotation(unsigned char rotation) {
	ILI9341_Send_Command(0x36);
	switch (rotation) {
	case 0:
		ILI9341_Send_Data(0x48);
		xLen = oxLen;
		yLen = oyLen;
		break;
	case 1:
		ILI9341_Send_Data(0x28);
		xLen = oyLen;
		yLen = oxLen;
		break;
	case 2:
		ILI9341_Send_Data(0x88);
		xLen = oxLen;
		yLen = oyLen;
		break;
	case 3:
		ILI9341_Send_Data(0xE8);
		xLen = oyLen;
		yLen = oxLen;
		break;
	}
}

/*********************************************************************
 * @fn      		  - ILI9341_Set_Address
 * @brief             - Function configures the area of frame memory MCU can access by performing column and page address set commands
 *
 * @param[in]         - X coordinate of starting pixel
 * @param[in]         - y coordinate of starting pixel
 * @param[in]         - X coordinate of ending pixel
 * @param[in]         - y coordinate of ending pixel
 *
 * @return            - none
 * @Note              - none
 */
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

/*********************************************************************
 * @fn      		  - ILI9341_Send_Burst_SPI
 * @brief             - Function sends a burst of a given 16 bit color to screen using configured SPI
 *
 * @param[in]         - Pixel color value given in RGB565 format
 * @param[in]         - Number of pixels to be sent to screen
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Send_Burst_SPI(unsigned short ucolor, unsigned long len) {
	//Send Memory Write command
	ILI9341_Send_Command(0x2C);

	//Set DC for data bytes
	ILI9341_DataSelect(ENABLE);

	//SPI single byte method
	unsigned char high_bit = ucolor >> 8, low_bit = ucolor;
	for(int i = 0; i < len; i++) {
		ILI9341_SPI_Send(high_bit);
		ILI9341_SPI_Send(low_bit);
	}
}

/*********************************************************************
 * @fn      		  - ILI9341_Send_Burst_DMA
 * @brief             - Function sends a burst of a given 16 bit color to screen using configured DMA stream
 * @Note  				DMA should be configured using as M2P with a FIFO buffer having byte data width size for the peripheral and half word for memory.
 * 						sourceAddress will be set by function to the given color and does not need to be configured.
 *
 * @param[in]         - Pixel color value given in RGB565 format
 * @param[in]         - Number of pixels to be sent to screen
 *
 * @return            - none
 */
void ILI9341_Send_Burst_DMA(unsigned short ucolor, unsigned long len) {
	//Send Memory Write command
	ILI9341_Send_Command(0x2C);

	//Set DC for data bytes
	ILI9341_DataSelect(ENABLE);
	//Flip the bytes for the little-endian ARM core.
	ucolor = (((ucolor & 0x00FF) << 8) | ((ucolor & 0xFF00) >> 8));
	DMAHandle.DMA_Config.sourceAddress = (uint32_t *)&ucolor;
	//Program color source address
	DMAHandle.pDMAStream->M0AR = (uint32_t) DMAHandle.DMA_Config.sourceAddress;

	len = len*2;
	while(len > 0) {
		//Set Transfer length
		if(len < dmaMaxItemTransfer) {
			//Send transfer length if len is less than max
			DMAHandle.pDMAStream->NDTR = len;
		} else {
			//Else, Send max transfer length
			DMAHandle.pDMAStream->NDTR = dmaMaxItemTransfer;
		}
		//Clear any event flags
		clearAllInterrupts(&DMAHandle);
		//Enable DMA stream
		enable_dma_stream(&DMAHandle);
		//Wait for transfer to complete
		while(DMAHandle.pDMAStream->NDTR);
		//Decrement TX length
		if( len >= dmaMaxItemTransfer) {
			len -= dmaMaxItemTransfer;
		} else {
			len = 0;
		}
	}
}

/**
 * Low Level Data TX functions
 */

/*********************************************************************
 * @fn      		  - ILI9341_Send_Command
 * @brief             - Function pulls the DC and CS lines LOW then sends command to device
 *
 * @param[in]         - Command to send to device
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Send_Command(unsigned char command) {
	ILI9341_DataSelect(DISABLE);
	ILI9341_SPI_Send(command);
}

/*********************************************************************
 * @fn      		  - ILI9341_Send_Command
 * @brief             - Function pulls the CS line LOW and DC HIGH then sends 1 byte of data to device
 *
 * @param[in]         - Data to send to device
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Send_Data(unsigned char data) {
	ILI9341_DataSelect(ENABLE);
	ILI9341_SPI_Send(data);
}

/*********************************************************************
 * @fn      		  - ILI9341_SPI_Send_32
 * @brief             - Function sends a command and 4 bytes of data to device
 *
 * @param[in]         - Command to send to device
 * @param[in]         - 32 bit data to be sent after command
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_SPI_Send_32(unsigned char command, unsigned long data) {
	ILI9341_Send_Command(command);

	ILI9341_DataSelect(ENABLE);
	ILI9341_SPI_Send(data >> 24);
	ILI9341_SPI_Send(data >> 16);
	ILI9341_SPI_Send(data >> 8);
	ILI9341_SPI_Send(data);
}

/*********************************************************************
 * @fn      		  - ILI9341_SPI_Send
 * @brief             - Function sends 1 byte of data over configured SPI peripheral
 *
 * @param[in]         - 1 byte of data to be sent over SPI
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_SPI_Send(unsigned char data)
{
	SPI_SendData(SPIHandle.pSPIx, &data, 1);
}

/**
 * Line Controls
 */

/*********************************************************************
 * @fn      		  - ILI9341_ChipSelect
 * @brief             - Function sets CS line to LOW if ENABLE to enable chip, HIGH if DISABLE to disable chip
 * @Note              - Chip will remain selected after power on.  May be disabled and enabled as needed when not in use.
 *
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_ChipSelect(uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdCS, GPIO_PIN_RESET);
	} else {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdCS, GPIO_PIN_SET);
	}
}

/*********************************************************************
 * @fn      		  - ILI9341_DataSelect
 * @brief             - Function sets DC line to LOW if DISABLE for commands,
 * 												 HIGH if ENABLE for data
 *
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_DataSelect(uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdDC, GPIO_PIN_SET);
	} else {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdDC, GPIO_PIN_RESET);
	}
}
