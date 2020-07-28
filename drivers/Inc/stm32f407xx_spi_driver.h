/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: June 2, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a SPIx peripheral
 */
typedef struct {
	uint8_t SPI_DeviceMode;			/* possible values from @SPI_DeviceMode */
	uint8_t SPI_BusConfig;			/* possible values from @SPI_BusConfig */
	uint8_t SPI_SclkSpeed;			/* possible values from @SPI_SclkSpeed */
	uint8_t SPI_DFF;				/* possible values from @SPI_DFF */
	uint8_t SPI_CPOL;				/* possible values from @SPI_CPOL */
	uint8_t SPI_CPHA;				/* possible values from @SPI_CPHA */
	uint8_t SPI_SSM;				/* possible values from @SPI_SSM */
} SPI_Config_t;

/**
 * This is a Handle structure for a SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t *pSPIx; 			/* This holds the base address of the SPIx(1,2,3) peripheral */
	SPI_Config_t SPI_Config;		/* This holds SPI configuration settings */
	uint8_t		*pTxBuffer;			/* Holds the application's Tx buffer address */
	uint8_t		*pRxBuffer;			/* Holds the application's Rx buffer address */
	uint32_t	txLen;				/* Tx Length */
	uint32_t	rxLen;				/* Rx Length */
	uint8_t		txState;			/* Holds the Tx state defined in @SPI_STATE */
	uint8_t		rxState;			/* Holds the Rx state defined in @SPI_STATE */
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE				0	/* Slave Mode */
#define SPI_DEVICE_MODE_MASTER				1	/* Master mode */

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1	/* Full Duplex */
#define SPI_BUS_CONFIG_HD					2	/* Half Duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3	/* Simplex Receive Only */

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0	/* fPCLK / 2 */
#define SPI_SCLK_SPEED_DIV4					1	/* fPCLK / 4 */
#define SPI_SCLK_SPEED_DIV8					2	/* fPCLK / 8 */
#define SPI_SCLK_SPEED_DIV16				3	/* fPCLK / 16 */
#define SPI_SCLK_SPEED_DIV32				4	/* fPCLK / 32 */
#define SPI_SCLK_SPEED_DIV64				5	/* fPCLK / 64 */
#define SPI_SCLK_SPEED_DIV128				6	/* fPCLK / 128 */
#define SPI_SCLK_SPEED_DIV256				7	/* fPCLK / 256 */

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS						0	/* 8 bit data frame format */
#define SPI_DFF_16BITS						1	/* 16 bit data frame format */

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW						0	/* SCLK to 0 when idle */
#define SPI_CPOL_HIGH						1	/* SCLK to 1 when idle */

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST						0	/* data captured on first clock transition */
#define SPI_CPHA_SECOND						1	/* data captured on second clock transition */

/*
 * @SPI_SSM
 */
#define SPI_SSM_DS							0	/* Software slave management disabled */
#define SPI_SSM_EN							1	/* Software slave management enabled */

/*
 * SPI related status flag definitions
 * @SPI_FLAGS
 */
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)
#define SPI_OVR_FLAG						(1 << SPI_SR_OVR)

/**
 * SPI application states
 * @SPI_STATE
 */
#define SPI_READY							0
#define SPI_BUSY_RX							1
#define SPI_BUSY_TX							2

/**
 * SPI application events
 * @SPI_EVENT
 */
#define SPI_EVENT_TX_COMPLETE				1
#define SPI_EVENT_RX_COMPLETE				2
#define SPI_EVENT_OVR_ERR					3


/*******************************************************************************************************************
 * Bit position definitions of SPI peripheral
 *******************************************************************************************************************/

/*
 * Bit position definitions for SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions for SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit position definitions for SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * Other Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDis);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDis);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDis);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
uint8_t SPI_VerifyResponse(uint8_t ackByte);

/**
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent);

/**
 * IRQ Configuration and ISR handling
 */
void SPI_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
