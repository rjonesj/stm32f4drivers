/*
 * 013I2C_Slave_TX_String_IT.c
 *
 *  Created on: Jun 21, 2020
 *      Author: rjonesj
 *
 * When s character is sent over serial monitor on master Arduino, slave should read and send data to connected master over I2C using interrupt based methods.
 * Master must first send the length of data to read the subsequent data from the slave.
 * 	-Master sends command code 0x51 to read the length (1 byte) of the data from the slave
 * 	-Master sends command code 0x52 to read the complete data from the slave
 * Upload 003I2CMasterRxString to Arduino to be connected as master I2C device. Serial monitor should add no line ending to Strings for repeated communication.
 *
 * 1. Use I2C SCL - 100 KHz (Standard mode)
 * 2. Use internal PU resistors for SDA and SCL lines
 *
 * STM32F4DISCOVERY I2C1 pins
 * PB6 --> SCL
 * PB7 --> SDA
 *
 * Arduino I2C pins
 * A5 --> SCL
 * A4 --> SDA
 */

# include "stm32f407xx.h"
# include <string.h>
# include <stdio.h>

#define SLAVE_ADDRESS	 0x68
#define MY_ADDRESS		 SLAVE_ADDRESS
#define LEN_CMD			 0x51
#define DATA_CMD		 0x52

I2C_Handle_t I2C1Handle;
uint8_t tx_buf[32] = "Hello! From STM32F4DISCOVERY.";

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

void I2C1_GPIOInit(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Init(void) {
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GpioButton;

	//Configure Button
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //No internal resistor required as one is built into the button
	GPIO_Init(&GpioButton);
}

int main(void) {
	//Initialize GPIO Pins for I2C
	I2C1_GPIOInit();

	//Initialize I2C peripheral
	I2C1_Init();

	//I2C IRQ Configuration
	I2C_IRQConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER,ENABLE);
	I2C_SlaveConfigureCallbackEvents(I2C1Handle.pI2Cx, ENABLE);

	//Initialize button
	GPIO_ButtonInit();

	//Enable the I2C peripheral
	I2C_PeripheralControl(&I2C1Handle, ENABLE);

	while(1);
}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent) {
	static uint8_t commandCode = 0;
	static uint8_t i = 0;

	if(appEvent == I2C_EV_DATA_REQ) {
		//Slave needs to send data to master
		if(commandCode == LEN_CMD) {
			//Send the length information to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char *)tx_buf));
		} else if(commandCode == DATA_CMD) {
			//Send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buf[i++]);
		}
	} else if(appEvent == I2C_EV_DATA_RCV) {
		//Slave needs to receive data from master
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	} else if(appEvent == I2C_ERROR_AF) {
		//Only happens during Tx.  Slave should not send any more data to master.
		commandCode = 0xFF;
		i = 0;
	} else if(appEvent == I2C_EV_STOP) {
		//Only happens during Rx.  Master has ended I2C communication with slave and should not receive any more data.
	}
}
