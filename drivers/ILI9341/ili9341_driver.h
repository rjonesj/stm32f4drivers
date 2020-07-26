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
	SPI_Handle_t *pSPIHandle;			/* This holds the base address of the configured SPI peripheral */
	uint8_t lcdResetPin;				/* GPIO Pin Configured for RESET */
	uint8_t lcdCSPin;					/* GPIO Pin Configured for CS */
	uint8_t lcdDCPin;					/* GPIO Pin Configured for DC */
	uint16_t xPixels;					/* Number of pixels on x-axis in default vertical mode */
	uint16_t yPixels;					/* Number of pixels on y-axis in default vertical mode */
	uint8_t enableDMA;					/* ENABLE or DISABLE macro to use DMA for data transfer */
	DMA_Handle_t *pDMAHandle;			/* This holds the base address of the configured DMA stream to send data to SPI Tx Buffer */
	uint16_t dmaMaxTransfer;			/* Max number of items to send when using DMA transfers */
} ILI9341_SPI_Config_t;

/**
 * This is an 8080 16-bit Parallel Configuration structure for an ILI9341 device
 */
typedef struct {
	uint8_t lcdResetPin;				/* GPIO Pin Configured for RESET */
	uint8_t lcdWRPin;					/* GPIO Pin Configured for RESET */
	uint8_t lcdRDPin;					/* GPIO Pin Configured for CS */
	uint8_t lcdCSPin;					/* GPIO Pin Configured for DC */
	uint8_t lcdDCPin;					/* GPIO Pin Configured for DC */
	uint16_t xPixels;					/* Number of pixels on x-axis in default vertical mode */
	uint16_t yPixels;					/* Number of pixels on y-axis in default vertical mode */
	uint8_t dataPortMode;				/* Possible values from ILI9341_PARALLEL_PORTMODE */
	GPIO_RegDef_t *singleDataPort;		/* Set this element when all data pins are configured on the same GPIO port */
	GPIO_Pin_Handle_t *dataPins[16];	/* Set this element when data pins are on multiple GPIO ports */
} ILI9341_Parallel_Config_t;

/**
 * This is a Handle structure for an ILI9341 device
 */
typedef struct {
	GPIO_Handle_t *pLCDPins;					/* This holds the base address of the LCD GPIO Pins */
	ILI9341_SPI_Config_t ILI9341_SPI_Config;			/* This holds the ILI9341 configuration settings for SPI mode */
	ILI9341_Parallel_Config_t ILI9341_Parallel_Config;			/* This holds the ILI9341 configuration settings for parallel mode*/
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

/**
 * @ILI9341_PARALLEL_PORTMODE
 * ILI9341 Parallel port modes
 */
#define ILI9341_PARALLEL_PORTMODE_SINGLE        0 	/* Configure this mode when all data pins [15:0] are on the same GPIO port (ie GPIOB). Faster performance */
#define ILI9341_PARALLEL_PORTMODE_MULTI         1	/* Configure this mode when data pins are located on different ports. */


/**
 * @ILI9341_DATAPIN
 * ILI9341 parallel data pins
 */
#define ILI9341_DATAPIN_0	 	0
#define ILI9341_DATAPIN_1	 	1
#define ILI9341_DATAPIN_2	 	2
#define ILI9341_DATAPIN_3	 	3
#define ILI9341_DATAPIN_4	 	4
#define ILI9341_DATAPIN_5	 	5
#define ILI9341_DATAPIN_6	 	6
#define ILI9341_DATAPIN_7	 	7
#define ILI9341_DATAPIN_8	 	8
#define ILI9341_DATAPIN_9	 	9
#define ILI9341_DATAPIN_10	 	10
#define ILI9341_DATAPIN_11	 	11
#define ILI9341_DATAPIN_12	 	12
#define ILI9341_DATAPIN_13	 	13
#define ILI9341_DATAPIN_14	 	14
#define ILI9341_DATAPIN_15	 	15

/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Initialization and De-Initialization
 */
void ILI9341_DataPin_Init(GPIO_Handle_t *gpioHandle, uint8_t pinNo, uint8_t dataNo);
void ILI9341_Init(ILI9341_Handle_t *pHandle);
void ILI9341_Reset();
void ILI9341_PowerOn();

/**
 * High Level user functions
 */
void ILI9341_Fill_Screen(unsigned int color);

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
void ILI9341_Send_32(unsigned char command, unsigned long data);

void ILI9341_SPI_Send(uint16_t data);
void ILI9341_Parallel_Send(uint16_t data);
void ILI9341_Parallel_Send_SinglePort(uint16_t data);

/**
 * Line Controls
 */
void ILI9341_ChipSelect(uint8_t enOrDis);
void ILI9341_DataSelect(uint8_t enOrDis);
void ILI9341_WriteSelect(uint8_t enOrDis);
void ILI9341_ReadSelect(uint8_t enOrDis);

#endif /* ILI9341_DRIVER_H_ */
