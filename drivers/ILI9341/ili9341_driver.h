/*
 * ili9341_driver.h
 *
 *  Created on: Jul 21, 2020
 *      Author: rjonesj
 */

#ifndef ILI9341_DRIVER_H_
#define ILI9341_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

/**
 * This is a 4 wire SPI Configuration structure for an ILI9341 device
 */
typedef struct {
	uint8_t lcdResetPin;				/* GPIO Pin Configured for RESET */
	uint8_t lcdCSPin;					/* GPIO Pin Configured for CS */
	uint8_t lcdDCPin;					/* GPIO Pin Configured for DC */
	uint16_t xPixels;					/* Number of pixels on x-axis in default vertical mode */
	uint16_t yPixels;					/* Number of pixels on y-axis in default vertical mode */
	uint16_t dmaMaxTransfer;			/* Max number of items to send when using DMA transfers */
} ILI9341_Config_t;

/**
 * This is an 8080 16-bit Parallel Configuration structure for an ILI9341 device
 */
typedef struct {
	uint8_t lcdResetPin;				/* GPIO Pin Configured for RESET */
	uint8_t lcdWRPin;					/* GPIO Pin Configured for RESET */
	uint8_t lcdRDPin;					/* GPIO Pin Configured for CS */
	uint8_t lcdCSPin;					/* GPIO Pin Configured for DC */
	uint8_t lcdDCPin;					/* GPIO Pin Configured for DC */
	GPIO_Pin_Handle_t *dataPins[16];
	uint16_t xPixels;					/* Number of pixels on x-axis in default vertical mode */
	uint16_t yPixels;					/* Number of pixels on y-axis in default vertical mode */
} ILI9341_Parallel_Config_t;

/**
 * This is a Handle structure for an ILI9341 device
 */
typedef struct {
	GPIO_Handle_t *pLCDPins;					/* This holds the base address of the LCD GPIO Pins */
	SPI_Handle_t *pSPIHandle;					/* This holds the base address of the configured SPI peripheral */
	DMA_Handle_t *pDMAHandle;					/* This holds the base address of the configured DMA stream to send data to SPI Tx Buffer */
	ILI9341_Config_t ILI9341_Config;			/* This holds the ILI9341 configuration settings */
	ILI9341_Parallel_Config_t ILI9341_Parallel_Config;			/* This holds the ILI9341 configuration settings */
	uint8_t intfMode;					/* MCU Interface Selection - possible values from @ILI9341_MODE */
} ILI9341_Handle_t;

/**
 * @ILI9341_MODE
 * ILI9341 MCU Interface Selection
 */
#define ILI9341_MODE_4WIRE_8BIT_SERIAL				0
#define ILI9341_MODE_8080_I_16BIT_PARALLEL			1

/**
 * @ILI9341_COLOR
 * ILI9341 Color definitions
 */
#define ILI9341_COLOR_BLUE        0x001F
#define ILI9341_COLOR_RED         0xF800

/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Initialization and De-Initialization
 */
void ILI9341_Init(ILI9341_Handle_t *pHandle);
void ILI9341_Reset();
void ILI9341_PowerOn();

/**
 * High Level user functions
 */
void ILI9341_Fill_Screen(unsigned int color, uint8_t enableDMA);

/*
 * ILI9341 Commands
 */
void ILI9341_Set_Rotation(unsigned char rotation) ;
void ILI9341_Set_Address(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void ILI9341_Send_Burst_SPI(unsigned short ucolor, unsigned long len);
void ILI9341_Send_Burst_DMA(unsigned short ucolor, unsigned long len);
void ILI9341_Send_Burst_Parallel(unsigned short ucolor, unsigned long len);

/**
 * Low Level Data TX functions
 */
void ILI9341_Send_Command(unsigned char command);
void ILI9341_Send_Data(unsigned char data);
void ILI9341_SPI_Send(unsigned char data);
void ILI9341_SPI_Send_32(unsigned char command, unsigned long data);
void ILI9341_Parallel_Send(uint16_t data);
void ILI9341_Parallel_Send_32(unsigned char command, unsigned long data);
uint16_t ILI9341_DataPort_Read(void);

/**
 * Line Controls
 */
void ILI9341_ChipSelect(uint8_t enOrDis);
void ILI9341_DataSelect(uint8_t enOrDis);
void ILI9341_WriteSelect(uint8_t enOrDis);
void ILI9341_ReadSelect(uint8_t enOrDis);

#endif /* ILI9341_DRIVER_H_ */
