/*
 * 011I2C_Master_RX_Testing.c
 *
 *  Created on: Jun 18, 2020
 *      Author: rjonesj
 *
 * When button on master is pressed, master should read and display data from connected Arduino slave over I2C.
 * Master must first receive the length of data to read the subsequent data from the slave.
 * 	-Master sends command code 0x51 to read the length (1 byte) of the data from the slave
 * 	-Master sends command code 0x52 to read the complete data from the slave
 * Upload 002I2CSlaveTxString to Arduino to be connected as slave SPI device.
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

#define MY_ADDRESS		 0x61
#define SLAVE_ADDRESS	 0x68
#define LEN_CMD			 0x51
#define DATA_CMD		 0x52

I2C_Handle_t I2C1Handle;

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

	//Initialize button
	GPIO_ButtonInit();

	//Enable the I2C peripheral
	I2C_PeripheralControl(&I2C1Handle, ENABLE);

	uint8_t dataLen;
	uint8_t cmdBuffer;
	uint8_t data[32];
	while(1) {
		//Wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay(); // debounce button

		//Send command to get length from slave
		cmdBuffer = LEN_CMD;
		I2C_MasterSendData(&I2C1Handle, &cmdBuffer, 1, SLAVE_ADDRESS, I2C_ENABLE_SR);
		//Read length information from slave
		I2C_MasterReceiveData(&I2C1Handle, &dataLen, 1, SLAVE_ADDRESS, I2C_ENABLE_SR);

		//Send command to get String from slave
		cmdBuffer = DATA_CMD;
		I2C_MasterSendData(&I2C1Handle, &cmdBuffer, 1, SLAVE_ADDRESS, I2C_ENABLE_SR);
		//Read data from slave
		I2C_MasterReceiveData(&I2C1Handle, data, dataLen, SLAVE_ADDRESS, I2C_DISABLE_SR);

		//display data
		printf("Slave returned: %s", data);
	}
}
