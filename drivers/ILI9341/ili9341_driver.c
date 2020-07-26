/*
 * ili9341_driver.c
 *
 *  Created on: Jul 21, 2020
 *      Author: rjonesj
 */

#include "ili9341_driver.h"

ILI9341_Handle_t *pILI9341Handle;
GPIO_Handle_t LCDPins;
GPIO_Pin_Handle_t dataPins[16];
uint8_t mode, lcdReset, lcdCS, lcdDC, lcdWR, lcdRD;
uint16_t xLen, yLen, oxLen, oyLen;
void (*pSendDataFunction)(uint16_t);		/* Address to function which sends data */
void (*pBurstFunction)(unsigned short ucolor, unsigned long len); /* Address to function which sends data burst*/

static void delay(void) {
	for(int i = 0; i < 500000; i++);
}

/**
 * Initialization and De-Initialization
 */
/*********************************************************************
 * @fn      		  - ILI9341_DataPin_Init
 * @brief             - Function initializes the GPIO pin and sets it in the dataPins array.
 * 						Only needs to be called to initialize parallel data pins when using multi port mode before ILI9341_Init.
 *
 * @param[in]         - Address to ILI9341_Handle_t struct
 *
 * @return            - none

 */
void ILI9341_DataPin_Init(GPIO_Handle_t *gpioHandle, uint8_t pinNo, uint8_t dataNo) {
	//Initialize GPIO pin
	GPIO_Pin_Init(gpioHandle, pinNo);

	//Create a pinHandle for the data pin and add it to the dataPins array
	GPIO_Pin_Handle_t pinHandle;
	pinHandle.pGPIOx = gpioHandle->pGPIOx;
	pinHandle.pinNo = pinNo;
	dataPins[dataNo] = pinHandle;
}

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
	pILI9341Handle = pHandle;
	mode = pHandle->intfMode;
	LCDPins = *pHandle->pLCDPins;
	xLen = pHandle->xPixels;
	yLen = pHandle->yPixels;
	if(mode == ILI9341_MODE_4WIRE_8BIT_SERIAL) {
		lcdReset = pHandle->ILI9341_SPI_Config.lcdResetPin;
		lcdCS = pHandle->ILI9341_SPI_Config.lcdCSPin;
		lcdDC = pHandle->ILI9341_SPI_Config.lcdDCPin;

		//Initialize function addresses
		pSendDataFunction = ILI9341_SPI_Send;
		if(pILI9341Handle->ILI9341_SPI_Config.enableDMA == ENABLE) {
			pBurstFunction = ILI9341_Send_Burst_DMA;
		} else {
			pBurstFunction = ILI9341_Send_Burst_SPI;
		}
	} else if(mode == ILI9341_MODE_8080_I_16BIT_PARALLEL) {
		lcdReset = pILI9341Handle->ILI9341_Parallel_Config.lcdResetPin;
		lcdCS = pILI9341Handle->ILI9341_Parallel_Config.lcdCSPin;
		lcdWR = pILI9341Handle->ILI9341_Parallel_Config.lcdWRPin;
		lcdRD = pILI9341Handle->ILI9341_Parallel_Config.lcdRDPin;
		lcdDC = pILI9341Handle->ILI9341_Parallel_Config.lcdDCPin;

		//Initialize function addresses
		if(pILI9341Handle->ILI9341_Parallel_Config.dataPortMode == ILI9341_PARALLEL_PORTMODE_SINGLE) {
			pSendDataFunction = ILI9341_Parallel_Send_SinglePort;
			pBurstFunction = ILI9341_Send_Burst_Parallel_SinglePort;
		} else {
			pSendDataFunction = ILI9341_Parallel_Send;
			pBurstFunction = ILI9341_Send_Burst_Parallel;
		}
	}
	oxLen = xLen;
	oyLen = yLen;

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

	//Set default signal state for 16 bit mode
	if(mode == ILI9341_MODE_8080_I_16BIT_PARALLEL) {
		ILI9341_ReadSelect(DISABLE);
		ILI9341_WriteSelect(DISABLE);
	}

	/* Reset The Screen */
	ILI9341_Send_Command(0x01);
	delay();

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
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Fill_Screen(unsigned int color)
{
	pBurstFunction(color, (long)xLen * (long)yLen);
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
	 //Column Address Set
	ILI9341_Send_32(0x2A, t);

	t = y1;
	t <<= 16;
	t |= y2;
	//Page Address Set
	ILI9341_Send_32(0x2B, t);
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
		pSendDataFunction(high_bit);
		pSendDataFunction(low_bit);
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
	//Configure DMA Handle
	DMA_Handle_t DMAHandle = *pILI9341Handle->ILI9341_SPI_Config.pDMAHandle;
	uint16_t dmaMaxItemTransfer = pILI9341Handle->ILI9341_SPI_Config.dmaMaxTransfer;
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
	DMAHandle.DMA_Config.sourceAddress = NULL;
}

/*********************************************************************
 * @fn      		  - ILI9341_Send_Burst_Parallel
 * @brief             - Function sends a burst of a given 16 bit color to screen on parallel 16 bit data port
 *
 * @param[in]         - Pixel color value given in RGB565 format
 * @param[in]         - Number of pixels to be sent to screen
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Send_Burst_Parallel(unsigned short ucolor, unsigned long len) {
	//Send Memory Write command
	ILI9341_Send_Command(0x2C);

	//Set DC for data bytes
	ILI9341_DataSelect(ENABLE);

	//Parallel 2 byte method
	for(int i = 0; i < len; i++) {
		ILI9341_Parallel_Send(ucolor);
	}
}

/*********************************************************************
 * @fn      		  - ILI9341_Send_Burst_Parallel
 * @brief             - Function sends a burst of a given 16 bit color to screen on single 16 bit data port
 *
 * @param[in]         - Pixel color value given in RGB565 format
 * @param[in]         - Number of pixels to be sent to screen
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Send_Burst_Parallel_SinglePort(unsigned short ucolor, unsigned long len) {
	//Send Memory Write command
	ILI9341_Send_Command(0x2C);

	//Set DC for data bytes
	ILI9341_DataSelect(ENABLE);

	//Parallel 2 byte method over single port
	GPIO_RegDef_t *pLCDGPIOx = pILI9341Handle->pLCDPins->pGPIOx;
	GPIO_RegDef_t *pDataPortGPIOx = pILI9341Handle->ILI9341_Parallel_Config.singleDataPort;
	for(int i = 0; i < len; i++) {
		pLCDGPIOx->ODR &= ~(1 << lcdWR);
		pDataPortGPIOx->ODR = ucolor;
		pLCDGPIOx->ODR |= (1 << lcdWR);
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
	pSendDataFunction(command);
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
	pSendDataFunction(data);
}

/*********************************************************************
 * @fn      		  - ILI9341_Send_32
 * @brief             - Function sends a command and 4 bytes of data to device
 *
 * @param[in]         - Command to send to device
 * @param[in]         - 32 bit data to be sent after command
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Send_32(unsigned char command, unsigned long data) {
	ILI9341_Send_Command(command);

	ILI9341_DataSelect(ENABLE);
	pSendDataFunction(data >> 24);
	pSendDataFunction(data >> 16);
	pSendDataFunction(data >> 8);
	pSendDataFunction(data);
}

/*********************************************************************
 * @fn      		  - ILI9341_SPI_Send
 * @brief             - Function sends 1 byte of data over configured SPI peripheral
 *
 * @param[in]         - 1 byte of data to be sent over SPI (top 8 bits will be ignored)
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_SPI_Send(uint16_t data)
{
	SPI_SendData(pILI9341Handle->ILI9341_SPI_Config.pSPIHandle->pSPIx, (uint8_t *)&data, 1);
}

/*********************************************************************
 * @fn      		  - ILI9341_Parallel_Send
 * @brief             - Function sets 16 bits of data on configured data pins
 *
 * @param[in]         - 2 bytes of data to be sent over parallel interface
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Parallel_Send(uint16_t data)
{
	ILI9341_WriteSelect(ENABLE);
	//Set pins to data
	for(int i = 0; i < 16; i++) {
		GPIO_Pin_Handle_t *pinHandle = &dataPins[i];
		GPIO_WriteToOutputPin(pinHandle->pGPIOx, pinHandle->pinNo, (data & (1 << i)) >> i);
	}
	ILI9341_WriteSelect(DISABLE);

	//clear pins
//	for(int i = 0; i < 16; i++) {
//		GPIO_Pin_Handle_t *pinHandle = pILI9341Handle->ILI9341_Parallel_Config.dataPins[i];
//		GPIO_WriteToOutputPin(pinHandle->pGPIOx, pinHandle->pinNo, 0);
//	}
}

/*********************************************************************
 * @fn      		  - ILI9341_Parallel_Send_SinglePort
 * @brief             - Function sets 16 bits of data on configured data pins on single GPIO port
 *
 * @param[in]         - 2 bytes of data to be sent over parallel interface
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_Parallel_Send_SinglePort(uint16_t data)
{
	GPIO_RegDef_t *pLCDGPIOx = pILI9341Handle->pLCDPins->pGPIOx;
	GPIO_RegDef_t *pGPIOx = pILI9341Handle->ILI9341_Parallel_Config.singleDataPort;
	pLCDGPIOx->ODR &= ~(1 << lcdWR);
	pGPIOx->ODR = data;
	pLCDGPIOx->ODR |= (1 << lcdWR);

	//clear pins
//	GPIO_WriteToOutputPort(pILI9341Handle->ILI9341_Parallel_Config.singleDataPort, 0);
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

/*********************************************************************
 * @fn      		  - ILI9341_WriteSelect
 * @brief             - Function sets WR line to LOW if ENABLE to set data to be sent,
 * 												 HIGH if DISABLE to send data to display module on rising edge.
 *
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_WriteSelect(uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdWR, GPIO_PIN_RESET);
	} else {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdWR, GPIO_PIN_SET);
	}
}

/*********************************************************************
 * @fn      		  - ILI9341_ReadSelect
 * @brief             - Function sets RD line to LOW if ENABLE to allow display module to set data to be read,
 * 												 HIGH if DISABLE to read data on rising edge.
 *
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 * @Note              - none
 */
void ILI9341_ReadSelect(uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdRD, GPIO_PIN_RESET);
	} else {
		GPIO_WriteToOutputPin(LCDPins.pGPIOx, lcdRD, GPIO_PIN_SET);
	}
}
