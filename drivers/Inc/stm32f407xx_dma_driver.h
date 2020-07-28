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

/*******************************************************************************************************************
 * Bit position definitions of DMA peripheral
 *******************************************************************************************************************/

/*
 * Bit position definitions for DMA low interrupt status register
 */
#define DMA_LISR_FEIF0			0
#define DMA_LISR_DMEIF0			2
#define DMA_LISR_TEIF0			3
#define DMA_LISR_HTIF0			4
#define DMA_LISR_TCIF0			5
#define DMA_LISR_FEIF1			6
#define DMA_LISR_DMEIF1			8
#define DMA_LISR_TEIF1			9
#define DMA_LISR_HTIF1			10
#define DMA_LISR_TCIF1			11
#define DMA_LISR_FEIF2			16
#define DMA_LISR_DMEIF2			18
#define DMA_LISR_TEIF2			19
#define DMA_LISR_HTIF2			20
#define DMA_LISR_TCIF2			21
#define DMA_LISR_FEIF3			22
#define DMA_LISR_DMEIF3			24
#define DMA_LISR_TEIF3			25
#define DMA_LISR_HTIF3			26
#define DMA_LISR_TCIF3			27

/*
 * Bit position definitions for DMA high interrupt status register
 */
#define DMA_HISR_FEIF4			0
#define DMA_HISR_DMEIF4			2
#define DMA_HISR_TEIF4			3
#define DMA_HISR_HTIF4			4
#define DMA_HISR_TCIF4			5
#define DMA_HISR_FEIF5			6
#define DMA_HISR_DMEIF5			8
#define DMA_HISR_TEIF5			9
#define DMA_HISR_HTIF5			10
#define DMA_HISR_TCIF5			11
#define DMA_HISR_FEIF6			16
#define DMA_HISR_DMEIF6			18
#define DMA_HISR_TEIF6			19
#define DMA_HISR_HTIF6			20
#define DMA_HISR_TCIF6			21
#define DMA_HISR_FEIF7			22
#define DMA_HISR_DMEIF7			24
#define DMA_HISR_TEIF7			25
#define DMA_HISR_HTIF7			26
#define DMA_HISR_TCIF7			27

/*
 * Bit position definitions for DMA low interrupt flag clear register
 */
#define DMA_LIFCR_CFEIF0		0
#define DMA_LIFCR_CDMEIF0		2
#define DMA_LIFCR_CTEIF0		3
#define DMA_LIFCR_CHTIF0		4
#define DMA_LIFCR_CTCIF0		5
#define DMA_LIFCR_CFEIF1		6
#define DMA_LIFCR_CDMEIF1		8
#define DMA_LIFCR_CTEIF1		9
#define DMA_LIFCR_CHTIF1		10
#define DMA_LIFCR_CTCIF1		11
#define DMA_LIFCR_CFEIF2		16
#define DMA_LIFCR_CDMEIF2		18
#define DMA_LIFCR_CTEIF2		19
#define DMA_LIFCR_CHTIF2		20
#define DMA_LIFCR_CTCIF2		21
#define DMA_LIFCR_CFEIF3		22
#define DMA_LIFCR_CDMEIF3		24
#define DMA_LIFCR_CTEIF3		25
#define DMA_LIFCR_CHTIF3		26
#define DMA_LIFCR_CTCIF3		27

/*
 * Bit position definitions for DMA high interrupt flag clear register
 */
#define DMA_HIFCR_CFEIF4		0
#define DMA_HIFCR_CDMEIF4		2
#define DMA_HIFCR_CTEIF4		3
#define DMA_HIFCR_CHTIF4		4
#define DMA_HIFCR_CTCIF4		5
#define DMA_HIFCR_CFEIF5		6
#define DMA_HIFCR_CDMEIF5		8
#define DMA_HIFCR_CTEIF5		9
#define DMA_HIFCR_CHTIF5		10
#define DMA_HIFCR_CTCIF5		11
#define DMA_HIFCR_CFEIF6		16
#define DMA_HIFCR_CDMEIF6		18
#define DMA_HIFCR_CTEIF6		19
#define DMA_HIFCR_CHTIF6		20
#define DMA_HIFCR_CTCIF6		21
#define DMA_HIFCR_CFEIF7		22
#define DMA_HIFCR_CDMEIF7		24
#define DMA_HIFCR_CTEIF7		25
#define DMA_HIFCR_CHTIF7		26
#define DMA_HIFCR_CTCIF7		27

/*
 * Bit position definitions for DMA stream x configuration register
 */
#define DMA_SxCR_EN			0
#define DMA_SxCR_DMEIE		1
#define DMA_SxCR_TEIE		2
#define DMA_SxCR_HTIE		3
#define DMA_SxCR_TCIE		4
#define DMA_SxCR_PFCTRL		5
#define DMA_SxCR_DIR		6
#define DMA_SxCR_CIRC		8
#define DMA_SxCR_PINC		9
#define DMA_SxCR_MINC		10
#define DMA_SxCR_PSIZE		11
#define DMA_SxCR_MSIZE		13
#define DMA_SxCR_PINCOS		15
#define DMA_SxCR_PL			16
#define DMA_SxCR_DBM		18
#define DMA_SxCR_CT			19
#define DMA_SxCR_PBURST		21
#define DMA_SxCR_MBURST		23
#define DMA_SxCR_CHSEL		25

/*
 * Bit position definitions for DMA stream x FIFO control register
 */
#define DMA_SxFCR_FTH			0
#define DMA_SxFCR_DMDIS			2
#define DMA_SxFCR_FS			3
#define DMA_SxFCR_FEIE			7


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
void clearAllInterrupts(DMA_Handle_t *pDMAHandle);

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
