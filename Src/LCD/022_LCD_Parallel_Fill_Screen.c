/*
 * 022_LCD_Parallel_Fill_Screen.c
 *
 *  Created on: Jul 22, 2020
 *      Author: rjonesj
 */

/*
 * Program turns on a 240x320 pixel TFT LCD screen using ILI9341 driver with 8080 16 bit parallel interface and tests full screen refreshes using alternate colors.
 * This a sample application to test different methods and clock speeds to achieve fast refresh rates.
 *
 * Data pins can be configured on any available GPIO pin when using multi port mode. Else, must all be on same port with matching DB pin in single mode.
 * All other LCD pins must be configured on the same GPIO port.
 *
 *
 * Connections
 * LCD					STM32
 * GND			-->		GND
 * VCC			-->		3V
 * LED			-->		3V
 * RS (DC)		-->		PA1
 * WR			-->		PA6
 * RD			-->		PA5
 * CS			-->		PA2
 * REST (RESET) -->		PA0
 * TEST			-->		PA3
 *
 * DB0			-->		PB0
 * DB1			-->		PB1
 * DB2			-->		PB2
 * DB3			-->		PB3
 * DB4			-->		PB4
 * DB5			-->		PB5
 * DB6			-->		PB6
 * DB7			-->		PB7
 * DB8			-->		PB8
 * DB9			-->		PB9
 * DB10			-->		PB10
 * DB11			-->		PB11
 * DB12			-->		PB12
 * DB13			-->		PB13
 * DB14			-->		PB14
 * DB15			-->		PB15
 */

# include "stm32f407xx.h"
# include "ili9341_driver.h"
# include <string.h>

#define LCD_A_REST		GPIO_PIN_NO_0
#define LCD_A_RS		GPIO_PIN_NO_1
#define LCD_A_CS	 	GPIO_PIN_NO_2
#define LCD_A_TEST	 	GPIO_PIN_NO_3
#define LCD_A_RD	 	GPIO_PIN_NO_5
#define LCD_A_WR	 	GPIO_PIN_NO_6

#define LCD_B_DB0	 	GPIO_PIN_NO_0
#define LCD_B_DB1	 	GPIO_PIN_NO_1
#define LCD_B_DB2	 	GPIO_PIN_NO_2
#define LCD_B_DB3	 	GPIO_PIN_NO_3
#define LCD_B_DB4	 	GPIO_PIN_NO_4
#define LCD_B_DB5	 	GPIO_PIN_NO_5
#define LCD_B_DB6	 	GPIO_PIN_NO_6
#define LCD_B_DB7	 	GPIO_PIN_NO_7
#define LCD_B_DB8	 	GPIO_PIN_NO_8
#define LCD_B_DB9	 	GPIO_PIN_NO_9
#define LCD_B_DB10	 	GPIO_PIN_NO_10
#define LCD_B_DB11	 	GPIO_PIN_NO_11
#define LCD_B_DB12	 	GPIO_PIN_NO_12
#define LCD_B_DB13	 	GPIO_PIN_NO_13
#define LCD_B_DB14	 	GPIO_PIN_NO_14
#define LCD_B_DB15	 	GPIO_PIN_NO_15

#define X_PIXELS	240
#define Y_PIXELS	320

ILI9341_Handle_t ILIHandle;
GPIO_Handle_t A_Pins, B_Pins;

static void initOutputPinConfig(GPIO_Handle_t *gpioHandle);

static void initOutputPinConfig(GPIO_Handle_t *gpioHandle) {
	gpioHandle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioHandle->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioHandle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioHandle->GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
}

void LCD_GPIOInit(void) {
	//Initialize pin configuration
	initOutputPinConfig(&A_Pins);
	initOutputPinConfig(&B_Pins);

	//Configure port A pins
	A_Pins.pGPIOx = GPIOA;
	GPIO_Pin_Init(&A_Pins, LCD_A_REST);
	GPIO_Pin_Init(&A_Pins, LCD_A_RS);
	GPIO_Pin_Init(&A_Pins, LCD_A_CS);
	GPIO_Pin_Init(&A_Pins, LCD_A_RD);
	GPIO_Pin_Init(&A_Pins, LCD_A_WR);
	GPIO_Pin_Init(&A_Pins, LCD_A_TEST);

	//Configure port B pins (Parallel Data)
	B_Pins.pGPIOx = GPIOB;
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB0, ILI9341_DATAPIN_0);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB1, ILI9341_DATAPIN_1);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB2, ILI9341_DATAPIN_2);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB3, ILI9341_DATAPIN_3);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB4, ILI9341_DATAPIN_4);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB5, ILI9341_DATAPIN_5);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB6, ILI9341_DATAPIN_6);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB7, ILI9341_DATAPIN_7);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB8, ILI9341_DATAPIN_8);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB9, ILI9341_DATAPIN_9);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB10, ILI9341_DATAPIN_10);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB11, ILI9341_DATAPIN_11);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB12, ILI9341_DATAPIN_12);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB13, ILI9341_DATAPIN_13);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB14, ILI9341_DATAPIN_14);
	ILI9341_DataPin_Init(&B_Pins, LCD_B_DB15, ILI9341_DATAPIN_15);
}

void ILI9341_Driver_Init(void) {
	ILIHandle.intfMode = ILI9341_MODE_8080_I_16BIT_PARALLEL;
	ILIHandle.pLCDPins = &A_Pins;
	ILIHandle.xPixels = X_PIXELS;
	ILIHandle.yPixels = Y_PIXELS;

	ILIHandle.ILI9341_Parallel_Config.lcdResetPin = LCD_A_REST;
	ILIHandle.ILI9341_Parallel_Config.lcdWRPin = LCD_A_WR;
	ILIHandle.ILI9341_Parallel_Config.lcdRDPin = LCD_A_RD;
	ILIHandle.ILI9341_Parallel_Config.lcdCSPin = LCD_A_CS;
	ILIHandle.ILI9341_Parallel_Config.lcdDCPin = LCD_A_RS;

	//Single port initialization
	ILIHandle.ILI9341_Parallel_Config.dataPortMode = ILI9341_PARALLEL_PORTMODE_SINGLE;
	ILIHandle.ILI9341_Parallel_Config.singleDataPort = GPIOB;

	ILI9341_Init(&ILIHandle);
}

int main(void) {
	//Set System and Bus Clocks to max frequency
	ErrorStatus status = setClocksToMaxFrequency();
	if(status == ERROR) {
        // Do something to indicate error clock configuration
		while(1);
	}

	//Initialize LCD pins
	LCD_GPIOInit();

	//Initialize ILI9341 driver
	ILI9341_Driver_Init();

	/* Set rotation to landscape */
	ILI9341_Set_Rotation(1);
	ILI9341_Set_Address(0, 0, Y_PIXELS-1, X_PIXELS-1);

	//Perform 60 alternating color screen refreshes
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_RESET);
	for(int i = 0; i < 30; i++) {
		ILI9341_Fill_Screen(ILI9341_COLOR_BLUE);
		ILI9341_Fill_Screen(ILI9341_COLOR_RED);
	}
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_RESET);

	while(1);

	return 0;
}
