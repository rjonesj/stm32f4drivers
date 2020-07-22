/*
 * stm32f407xx_dma_driver.h
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_DMA_DRIVER_H_
#define INC_STM32F407XX_DMA_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a DMA Stream
 */
typedef struct {
	__vo uint32_t *sourceAddress;					/* Address of source data buffer */
	__vo uint32_t *destAddress;						/* Address of destination data buffer */
	uint32_t len;									/* length of data to send */
	uint8_t transferDirection;						/* possible values from @DMA_DIRECTION */
	uint8_t memDataSize;							/* memory data width size: possible values from @DMA_DATA_SIZE */
	uint8_t memIncrementMode;						/* memory increment mode: ENABLE or DISABLE macros */
	uint8_t periphDataSize;							/* peripheral data width size: possible values from @DMA_DATA_SIZE */
	uint8_t periphIncrementMode;					/* peripheral increment mode: ENABLE or DISABLE macros */
	uint8_t fifoMode;								/* ENABLE or DISABLE macros */
	uint8_t fifoThreshold;							/* possible values from @DMA_FIFO_THLD */
	uint8_t circularMode;							/* ENABLE or DISABLE macros */
	uint8_t priority;								/* possible values from @DMA_PRIORITY */
	uint8_t channel;								/* possible values from @DMA_CHANNEL */
	uint8_t stream;									/* possible values from @DMA_STREAM */
} DMA_Stream_Config_t;

/**
 * This is a Handle structure for a DMA controller
 */
typedef struct {
	DMA_RegDef_t *pDMAx; 						/* This holds the base address of the DMA controller */
	DMA_Stream_RegDef_t *pDMAStream;			/* This holds the base address of the DMA stream */
	DMA_Stream_Config_t DMA_Config;				/* This holds DMA configuration settings */
} DMA_Handle_t;

/**
 * @DMA_DIRECTION
 * DMA possible transfer directions
 */
#define DMA_DIRECTION_P2M		0
#define DMA_DIRECTION_M2P		1
#define DMA_DIRECTION_M2M		2

/**
 * @DMA_DATA_SIZE
 * DMA possible memory and peripheral data sizes
 */
#define DMA_DATA_SIZE_BYTE			0
#define DMA_DATA_SIZE_HALF_WORD		1
#define DMA_DATA_SIZE_WORD			2

/**
 * @DMA_FIFO_THLD
 * DMA possible fifo threshold selection
 */
#define DMA_FIFO_THLD_1_4_FULL			0
#define DMA_FIFO_THLD_1_2_FULL			1
#define DMA_FIFO_THLD_3_4_FULL			2
#define DMA_FIFO_THLD_FULL				3

/**
 * @DMA_PRIORITY
 * DMA priority levels
 */
#define DMA_PRIORITY_LOW		0
#define DMA_PRIORITY_MEDIUM		1
#define DMA_PRIORITY_HIGH		2
#define DMA_PRIORITY_VERY_HIGH	3

/**
 * @DMA_CHANNEL
 * DMA channel selection
 */
#define DMA_CHANNEL_0			0
#define DMA_CHANNEL_1			1
#define DMA_CHANNEL_2			2
#define DMA_CHANNEL_3			3
#define DMA_CHANNEL_4			4
#define DMA_CHANNEL_5			5
#define DMA_CHANNEL_6			6
#define DMA_CHANNEL_7			7

/**
 * @DMA_STREAM
 * DMA stream selection
 */
#define DMA_STREAM_0			0
#define DMA_STREAM_1			1
#define DMA_STREAM_2			2
#define DMA_STREAM_3			3
#define DMA_STREAM_4			4
#define DMA_STREAM_5			5
#define DMA_STREAM_6			6
#define DMA_STREAM_7			7

/**
 * @DMA_EVENT
 * DMA interrupt event
 */
#define DMA_EVENT_TC			0
#define DMA_EVENT_HT			1
#define DMA_EVENT_TE			2
#define DMA_EVENT_DME			3
#define DMA_EVENT_FE			4


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void DMA_Init(DMA_Handle_t *pDMAHandle);
void DMA_DeInit(DMA_RegDef_t *pDMAx);

/**
 * Other peripheral controls
 */
void enable_dma_stream(DMA_Handle_t *pDMAHandle);
void dma_interrupt_configuration(DMA_Handle_t *pDMAHandle);
uint8_t getInterruptStatus(DMA_Handle_t *pDMAHandle, uint8_t interruptEvent);
void clearInterruptStatus(DMA_Handle_t *pDMAHandle, uint8_t interruptEvent);

/**
 * IRQ Configuration and ISR handling
 */
void DMA_IRQHandling(DMA_Handle_t *pDMAHandle);

/**
 * Application callback
 */
void HT_Complete_callback(void);
void TC_Complete_callback(void);
void TE_Complete_callback(void);
void DME_Complete_callback(void);
void FE_Complete_callback(void);


#endif /* INC_STM32F407XX_DMA_DRIVER_H_ */
