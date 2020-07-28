/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: June 23, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a USARTx peripheral
 */
typedef struct {
	uint8_t USART_Mode;				/* possible values from @USART_Mode */
	uint32_t USART_Baud;			/* possible values from @USART_Baud */
	uint8_t USART_NoOfStopBits;		/* possible values from @USART_NoOfStopBits */
	uint8_t USART_WordLength;		/* possible values from @USART_WordLength */
	uint8_t USART_ParityControl;	/* possible values from @USART_ParityControl */
	uint8_t USART_HWFlowControl;	/* possible values from @USART_HWFlowControl */
} USART_Config_t;

/**
 * This is a Handle structure for a USARTx peripheral
 */
typedef struct {
	USART_RegDef_t *pUSARTx; 			/* This holds the base address of the USART(1-6) peripheral */
	USART_Config_t USART_Config;		/* This holds USART configuration settings */
	uint8_t		*pTxBuffer;			/* Holds the application's Tx buffer address */
	uint8_t		*pRxBuffer;			/* Holds the application's Rx buffer address */
	uint32_t	txLen;				/* Tx Length */
	uint32_t	rxLen;				/* Rx Length */
	uint8_t		txState;			/* Holds the Tx state defined in @USART_STATE */
	uint8_t		rxState;			/* Holds the Rx state defined in @USART_STATE */
} USART_Handle_t;



/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART related status flag definitions
 * @USART_FLAGS
 */
#define USART_FLAG_PE						(1 << USART_SR_PE)
#define USART_FLAG_FE						(1 << USART_SR_FE)
#define USART_FLAG_NF						(1 << USART_SR_NF)
#define USART_FLAG_ORE						(1 << USART_SR_ORE)
#define USART_FLAG_IDLE						(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE						(1 << USART_SR_RXNE)
#define USART_FLAG_TC						(1 << USART_SR_TC)
#define USART_FLAG_TXE						(1 << USART_SR_TXE)
#define USART_FLAG_LBD						(1 << USART_SR_LBD)
#define USART_FLAG_CTS						(1 << USART_SR_CTS)

/**
 * Communication state of peripheral
 * @USART_STATE
 */
#define USART_READY							0
#define USART_BUSY_IN_RX					1
#define USART_BUSY_IN_TX					2

/**
 * Application event of peripheral
 * @USART_EVENT
 */
#define USART_EVENT_TX_CMPLT				0
#define USART_EVENT_RX_CMPLT				1
#define USART_EVENT_CTS						2
#define USART_EVENT_IDLE					3
#define USART_EVENT_ORE						4
#define USART_ERREVENT_FE					5
#define USART_ERREVENT_NF					6
#define USART_ERREVENT_ORE					7

/*******************************************************************************************************************
 * Bit position definitions of USART peripheral
 *******************************************************************************************************************/

/*
 * Bit position definitions for USART_SR
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*
 * Bit position definitions for USART_CR1
 */

#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

/*
 * Bit position definitions for USART_CR2
 */

#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

/*
 * Bit position definitions for USART_CR3
 */

#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

/*
 * Bit position definitions for USART_GTPR
 */

#define USART_GTPR_PSC			0
#define USART_GTPR_GT			8


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/**
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * IRQ Configuration and ISR handling
 */
void USART_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void USART_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/**
 * Other Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t enOrDis);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t flagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/**
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEvent);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
