/*
 * 014I2C_Slave_TX_String_Large_Buf_IT.c
 *
 *  Created on: Jun 21, 2020
 *      Author: rjonesj
 *
 * When s character is sent over serial monitor on master Arduino, slave should read and send data to connected master over I2C using interrupt based methods.
 * This version supports larger tx buffers of 32 bit length.
 * Master must first send the length of data to read the subsequent data from the slave.
 * 	-Master sends command code 0x51 to read the length (4 bytes) of the data from the slave
 * 	-Master sends command code 0x52 to read the complete data from the slave
 * Upload 003I2CMasterRxStringLen to Arduino to be connected as master I2C device.
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
#define BUF_LEN			 512

I2C_Handle_t I2C1Handle;
uint32_t dataLen = 0;
uint8_t tx_buf[BUF_LEN] = "Hello! From STM32F4DISCOVERY. This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :)";
//uint8_t tx_buf[BUF_LEN] = "Hello! From STM32F4DISCOVERY. This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) This is an extra looong message :) THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...THIS IS A SUPER LONG MESSAGE...";

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
	//Store length of txBuffer
	tx_buf[BUF_LEN-1] = '\0';
	dataLen = strlen((char *)tx_buf);

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
	static uint32_t i = 0;
	static uint32_t w_ptr = 0;

	 if(appEvent == I2C_EV_DATA_REQ) {
		//Slave needs to send data to master
		if(commandCode == LEN_CMD) {
			//Send 4 bytes of length information to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, ((dataLen >> ((i++%4) * 8)) & 0xFF));
		} else if(commandCode == DATA_CMD) {
			I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buf[w_ptr++]);
		}
	} else if(appEvent == I2C_EV_DATA_RCV) {
		//Slave needs to receive data from master
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	} else if(appEvent == I2C_ERROR_AF) {
		//Only happens during Tx.  Slave should not send any more data to master.
		//If the current active code is DATA_CMD, then don't validate
		if(!(commandCode == DATA_CMD)) {
			commandCode = 0xFF;
		}
		//Reset the counter at end of Tx
		i = 0;
		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(w_ptr >= dataLen) {
			w_ptr = 0;
			commandCode = 0xFF;
		}
	} else if(appEvent == I2C_EV_STOP) {
		//Only happens during Rx.  Master has ended I2C communication with slave and should not receive any more data.
		i = 0;
	}
}
