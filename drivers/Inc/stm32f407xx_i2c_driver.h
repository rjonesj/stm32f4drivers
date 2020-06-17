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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress);

/**
 * IRQ Configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t irqNumber, uint8_t enOrDis);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);

/**
 * Other Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDis);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);

/**
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
