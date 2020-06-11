/*
 * 008SPI_CMD_Handling.c
 *
 *  Created on: Jun 7, 2020
 *      Author: rjonesj
 */

/*
 * When button on master is pressed, master should send command to connected Arduino slave.
 * Slave will return an ACK or NACK response, then master will send 1 or more arguments to slave to be processed.
 * Slave will return response if applicable for command.  Master will then send the next command on the next button press.
 * Upload 002SPISlaveCmdHandling to Arduino to be connected as slave SPI device.
 *
 * Enable semihosting to view printf debugging statements
 * 	1. Exclude syscalls.c and sysmem.c from build
 * 	2. Goto Project > Properties. Select C/C++ Build > Settings. Select MCU GCC Linker > Libraries and add rdimon.
 * 	3. Select MCU GCC Linker > Miscellaneous. Add -specs=rdimon.specs to Other flags.
 * 	4. Open run configuration for debug mode. Select OpenOCD Debug probe.
 * 	5. Select Startup tab and add monitor arm semihosting enable to Initialization commands.
 *
 * 1. Use SPI Full duplex mode
 * 2. STM32F4DISCOVERY board will be in master mode and STM32F3DISCOVERY will be configured for SPI slave mode
 * 3. DFF = 0
 * 4. Use Hardware slave management (SSM = 0)
 * 5. SCLK speed = 2MHz, fclk = 16MHz
 *
 * STM32F4DISCOVERY SPI2 pins
 * PB15 --> MOSI
 * PB13 --> SCLK
 * PB14 --> MISO
 * PB12 --> NSS
 *
 * ALT function mode: 5
 *
 *
 * Arduino UNO SPI pins
 * 11 --> MOSI
 * 13 --> SCLK
 * 12 --> MISO
 * 10 --> NSS
 */

# include "stm32f407xx.h"
# include <string.h>
# include <stdio.h>

extern void initialise_monitor_handles();

/*
 * Command codes
 */
#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_ON				1
#define LED_OFF				0

#define LED_CTRL_FUN		0
#define SENSOR_READ_FUN		1
#define LED_READ_FUN		2
#define PRINT_FUN			3
#define ID_FUN				4

/*
 * Arduino analog pins
 */
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4
#define ANALOG_PIN5			5

/*
 * Arduino LED
 */
#define LED_PIN				9

uint8_t dummy_write = 0xff;
uint8_t dummy_read = 0xff;

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

void SPI2_GPIOInit(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //Enable PU to transmit to slave device
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void) {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //Generates SCLK of 2 Mhz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DS;

	SPI_Init(&SPI2Handle);
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

void ledControl(uint8_t pinNo, uint8_t pinValue) {
	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//1. CMD_LED_CTRL <pin no(1)> <value(1)>
	uint8_t commandCode = CMD_LED_CTRL;
	uint8_t ackByte;
	uint8_t args[2];

	SPI_SendData(SPI2, &commandCode, 1);

	//Do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	//Send dummy byte to fetch the response from the slave
	SPI_SendData(SPI2, &dummy_write, 1);
	//Read the ack byte received
	SPI_ReceiveData(SPI2, &ackByte, 1);
	if(SPI_VerifyResponse(ackByte)) {
		//send arguments
		args[0] = pinNo;
		args[1] = pinValue;
		SPI_SendData(SPI2, args, 2);
	}

	//Wait until SPI is not busy before returning
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
}

uint8_t sensorRead(uint8_t pinNo) {
	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//2. CMD_SENSOR_READ <analog pin number(1)>
	uint8_t commandCode = CMD_SENSOR_READ;
	uint8_t ackByte;
	uint8_t analog_read=0;
	uint8_t args[2];

	//Send command
	SPI_SendData(SPI2, &commandCode, 1);

	//Do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	//Send dummy byte to fetch the response from the slave
	SPI_SendData(SPI2, &dummy_write, 1);
	//Read the ack byte received
	SPI_ReceiveData(SPI2, &ackByte, 1);
	if(SPI_VerifyResponse(ackByte)) {
		//send arguments
		args[0] = pinNo;
		SPI_SendData(SPI2, args, 1);
		//Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//Add some delay so slave can process ADC conversion
		delay();
		//Send dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &analog_read, 1);
	}

	//Wait until SPI is not busy before returning
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	return analog_read;
}

uint8_t ledRead(uint8_t pinNo) {
	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//3. CMD_LED_READ <led pin number(1)>
	uint8_t commandCode = CMD_LED_READ;
	uint8_t ackByte;
	uint8_t led_read=0;
	uint8_t args[2];

	//Send command
	SPI_SendData(SPI2, &commandCode, 1);

	//Do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	//Send dummy byte to fetch the response from the slave
	SPI_SendData(SPI2, &dummy_write, 1);
	//Read the ack byte received
	SPI_ReceiveData(SPI2, &ackByte, 1);
	if(SPI_VerifyResponse(ackByte)) {
		//send arguments
		args[0] = pinNo;
		SPI_SendData(SPI2, args, 1);
		//Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//Send dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &led_read, 1);
	}

	//Wait until SPI is not busy before returning
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	return led_read;
}

void print(char *data, uint8_t len) {
	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//4. CMD_PRINT <len> <message>
	uint8_t commandCode = CMD_PRINT;
	uint8_t ackByte;

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//Send command
	SPI_SendData(SPI2, &commandCode, 1);

	//Do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	//Send dummy byte to fetch the response from the slave
	SPI_SendData(SPI2, &dummy_write, 1);
	//Read the ack byte received
	SPI_ReceiveData(SPI2, &ackByte, 1);
	if(SPI_VerifyResponse(ackByte)) {
		//First send length information
		SPI_SendData(SPI2, &len, 1);

		//Then, send the data over MOSI line
		SPI_SendData(SPI2, (uint8_t*)data, len);
	}

	//Wait until SPI is not busy before returning
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
}

void idRead(char *idBuffer, uint8_t idLen) {
	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//5. CMD_ID_READ
	uint8_t commandCode = CMD_ID_READ;
	uint8_t ackByte;

	//Send command
	SPI_SendData(SPI2, &commandCode, 1);

	//Do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	//Send dummy byte to fetch the response from the slave
	SPI_SendData(SPI2, &dummy_write, 1);
	//Read the ack byte received
	SPI_ReceiveData(SPI2, &ackByte, 1);
	if(SPI_VerifyResponse(ackByte)) {
		for(int i = 0; i < idLen; i++) {
			//Send dummy byte to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);
			//Write ID byte to idBuffer
			SPI_ReceiveData(SPI2, (uint8_t*)idBuffer++, 1);
		}
		//add null termination
		idBuffer[idLen] = '\0';
	}

	//Wait until SPI is not busy before returning
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
}

int main(void) {
	initialise_monitor_handles();

	printf("Application is running...\n");

	uint8_t cmdCounter = 0;
	uint8_t totalCmds = 5;
	uint8_t currentCmd;

	//Initialize button
	GPIO_ButtonInit();

	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	//Configure SPI2 peripheral
	SPI2_Init();

	/*
	* Enable SSOE (slave select output enable)
	* The NSS pin will be automatically managed by the hardware.
	* When SPE=1, NSS will be pulled to low
	* When SPE=0, NSS will be pulled to high
	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	printf("SPI Initialized...\n");

	/**
	 * Main loop
	 */
	uint8_t cmdReturn;
	while(1) {
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)) {
			delay(); //debounce button
			while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)); //wait until button has been released
			cmdReturn = 0;
			currentCmd = cmdCounter++ % totalCmds;

			if(currentCmd == LED_CTRL_FUN) {
				ledControl(LED_PIN, LED_ON);
			} else if(currentCmd == SENSOR_READ_FUN) {
				cmdReturn = sensorRead(ANALOG_PIN0);
			} else if(currentCmd == LED_READ_FUN) {
				cmdReturn = ledRead(LED_PIN);
			} else if(currentCmd == PRINT_FUN) {
				char user_data[] = "Hello World!";
				print(user_data, strlen(user_data));
			} else if(currentCmd == ID_FUN) {
				uint8_t idLen = 10;
				char idBuffer[idLen+1];
				idRead(idBuffer, idLen);
				printf("ID returned: %s\n", idBuffer);
			}

			printf("Command returned: %d\n", cmdReturn);
		}
	}


	return 0;
}
