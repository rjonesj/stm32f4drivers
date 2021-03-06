/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: June 12, 2020
 *      Author: rjonesj
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/**
 * This is a Configuration structure for a I2Cx peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;			/* possible values from @I2C_SCLSpeed */
	uint8_t I2C_DeviceAddress;		/* device address configured by user */
	uint8_t I2C_ACKControl;			/* possible values from  @I2C_ACKControl */
	uint8_t I2C_FMDutyCycle;		/* possible values from @I2C_FMDutyCycle */
} I2C_Config_t;

/**
 * This is a Handle structure for a I2Cx peripheral
 */
typedef struct {
	I2C_RegDef_t *pI2Cx; 			/* This holds the base address of the I2Cx(1,2,3) peripheral */
	I2C_Config_t I2C_Config;		/* This holds I2C configuration settings */
	uint8_t *pTxBuffer;				/* Application Tx Buffer address */
	uint8_t *pRxBuffer;				/* Application Rx Buffer address */
	uint32_t txLen;					/* Application Tx Len */
	uint32_t rxLen;					/* Application Rx Len */
	uint8_t txRxState;				/* Communication state from @I2C_STATES */
	uint8_t devAddr;				/* Slave / device address */
	uint32_t rxSize;				/* Rx Size */
	uint8_t sr;						/* Repeated start value */
} I2C_Handle_t;

/**
 * I2C serial clock speed
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM					100000 		/* Standard mode - 100 KHz */
#define I2C_SCL_SPEED_FM4K					400000		/* Fast mode - 400 KHz */
#define I2C_SCL_SPEED_FM2K					200000		/* Fast mode - 200 KHz */

/**
 * I2C Ack Control
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE						1 		/* Enable hardware Acking */
#define I2C_ACK_DISABLE						0 		/* Disable hardware Acking */

/**
 * I2C Duty Cycle
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2						0 		/* Fm mode tlow/thigh = 2 */
#define I2C_FM_DUTY_16_9					1 		/* Fm mode tlow/thigh = 16/9 */

/*
 * I2C related status flag definitions
 * @I2C_FLAGS
 */
#define I2C_FLAG_SB							(1 << I2C_SR1_SB)
#define I2C_FLAG_TXE						(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE						(1 << I2C_SR1_RXNE)
#define I2C_FLAG_ADDR						(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF						(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF						(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR						(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO						(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF							(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR						(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT					(1 << I2C_SR1_TIMEOUT)

/*
 * I2C Repeated Start macros
 */
#define I2C_ENABLE_SR								0
#define I2C_DISABLE_SR								1

/*
 * I2C application states
 * @I2C_STATES
 */
#define I2C_READY									0
#define I2C_BUSY_RX									1
#define I2C_BUSY_TX									2

/*
 * I2C application events
 * @I2C_STATES
 */
#define I2C_EV_TX_COMPLETE							0
#define I2C_EV_RX_COMPLETE							1
#define I2C_EV_STOP									2
#define I2C_ERROR_BERR								3
#define I2C_ERROR_ARLO								4
#define I2C_ERROR_AF								5
#define I2C_ERROR_OVR								6
#define I2C_ERROR_TIMEOUT							7
#define I2C_EV_DATA_REQ								8
#define I2C_EV_DATA_RCV								9


/******************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs, check the function definitions
 ******************************************************************************************************/

/**
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);

/**
 * Initialization and De-Initialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/**
 * IRQ Configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * Other Control APIs
 */
void I2C_PeripheralControl(I2C_Handle_t *pI2CHandle, uint8_t enOrDis);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDis);
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);
void I2C_CloseReception(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveConfigureCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t enOrDis);

/**
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
