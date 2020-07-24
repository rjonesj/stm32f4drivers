/*
 * 022_LCD_Parallel_Fill_Screen.c
 *
 *  Created on: Jul 22, 2020
 *      Author: rjonesj
 */

/*
 * Program turns on a 320X240 pixel TFT LCD screen using ILI9341 driver with 8080 16 bit parallel interface and tests full screen refreshes using alternate colors.
 * This a sample application to test different methods and clock speeds to achieve fast refresh rates.
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
 * F_CS			-->		PA7
 * REST (RESET)-->		PA0
 * TEST			-->		PA3
 *
 * DB0			-->		PC6
 * DB1			-->		PC8
 * DB2			-->		PA8
 * DB3			-->		PA10
 * DB4			-->		PD14
 * DB5			-->		PD11
 * DB6			-->		PD9
 * DB7			-->		PD1
 * DB8			-->		PC7
 * DB9			-->		PD12
 * DB10			-->		PA9
 * DB11			-->		PD15
 * DB12			-->		PD13
 * DB13			-->		PD10
 * DB14			-->		PD0
 * DB15			-->		PD2
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
#define LCD_A_F_CS	 	GPIO_PIN_NO_7

#define LCD_C_DB0	 	GPIO_PIN_NO_6
#define LCD_C_DB1	 	GPIO_PIN_NO_8
#define LCD_A_DB2	 	GPIO_PIN_NO_8
#define LCD_A_DB3	 	GPIO_PIN_NO_10
#define LCD_D_DB4	 	GPIO_PIN_NO_14
#define LCD_D_DB5	 	GPIO_PIN_NO_11
#define LCD_D_DB6	 	GPIO_PIN_NO_9
#define LCD_D_DB7	 	GPIO_PIN_NO_1
#define LCD_C_DB8	 	GPIO_PIN_NO_7
#define LCD_D_DB9	 	GPIO_PIN_NO_12
#define LCD_A_DB10	 	GPIO_PIN_NO_9
#define LCD_D_DB11	 	GPIO_PIN_NO_15
#define LCD_D_DB12	 	GPIO_PIN_NO_13
#define LCD_D_DB13	 	GPIO_PIN_NO_10
#define LCD_D_DB14	 	GPIO_PIN_NO_0
#define LCD_D_DB15	 	GPIO_PIN_NO_2

#define X_PIXELS	240
#define Y_PIXELS	320

#define ILI9341_DATAPIN_0	 	0
#define ILI9341_DATAPIN_1	 	1
#define ILI9341_DATAPIN_2	 	2
#define ILI9341_DATAPIN_3	 	3
#define ILI9341_DATAPIN_4	 	4
#define ILI9341_DATAPIN_5	 	5
#define ILI9341_DATAPIN_6	 	6
#define ILI9341_DATAPIN_7	 	7
#define ILI9341_DATAPIN_8	 	8
#define ILI9341_DATAPIN_9	 	9
#define ILI9341_DATAPIN_10	 	10
#define ILI9341_DATAPIN_11	 	11
#define ILI9341_DATAPIN_12	 	12
#define ILI9341_DATAPIN_13	 	13
#define ILI9341_DATAPIN_14	 	14
#define ILI9341_DATAPIN_15	 	15

GPIO_Pin_Handle_t dataPins[16];
GPIO_Handle_t A_Pins, C_Pins, D_Pins;
ILI9341_Handle_t ILIHandle;

static void initGPIOPinConfig(GPIO_Handle_t *gpioHandle);
static void initDataPin(GPIO_Handle_t *gpioHandle, uint8_t pinNo, uint8_t dataNo);

static void initGPIOPinConfig(GPIO_Handle_t *gpioHandle) {
	gpioHandle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioHandle->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioHandle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioHandle->GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
}

static void initDataPin(GPIO_Handle_t *gpioHandle, uint8_t pinNo, uint8_t dataNo) {
	//Initialize GPIO pin
	GPIO_Pin_Init(gpioHandle, pinNo);

	//Create a pinHandle for the data pin and add it to the dataPins array
	GPIO_Pin_Handle_t pinHandle;
	pinHandle.pGPIOx = gpioHandle->pGPIOx;
	pinHandle.pinNo = pinNo;
	dataPins[dataNo] = pinHandle;
}

void LCD_GPIOInit(void) {
	//Initialize pin configuration
	initGPIOPinConfig(&A_Pins);
	initGPIOPinConfig(&C_Pins);
	initGPIOPinConfig(&D_Pins);

	//Configure port A pins
	A_Pins.pGPIOx = GPIOA;
	GPIO_Pin_Init(&A_Pins, LCD_A_REST);
	GPIO_Pin_Init(&A_Pins, LCD_A_RS);
	GPIO_Pin_Init(&A_Pins, LCD_A_CS);
	GPIO_Pin_Init(&A_Pins, LCD_A_RD);
	GPIO_Pin_Init(&A_Pins, LCD_A_WR);
	GPIO_Pin_Init(&A_Pins, LCD_A_F_CS);
	GPIO_Pin_Init(&A_Pins, LCD_A_TEST);
	initDataPin(&A_Pins, LCD_A_DB2, ILI9341_DATAPIN_2);
	initDataPin(&A_Pins, LCD_A_DB3, ILI9341_DATAPIN_3);
	initDataPin(&A_Pins, LCD_A_DB10, ILI9341_DATAPIN_10);

	//Configure port C pins
	C_Pins.pGPIOx = GPIOC;
	initDataPin(&C_Pins, LCD_C_DB0, ILI9341_DATAPIN_0);
	initDataPin(&C_Pins, LCD_C_DB1, ILI9341_DATAPIN_1);
	initDataPin(&C_Pins, LCD_C_DB8, ILI9341_DATAPIN_8);

	//Configure port D pins
	D_Pins.pGPIOx = GPIOD;
	initDataPin(&D_Pins, LCD_D_DB4, ILI9341_DATAPIN_4);
	initDataPin(&D_Pins, LCD_D_DB5, ILI9341_DATAPIN_5);
	initDataPin(&D_Pins, LCD_D_DB6, ILI9341_DATAPIN_6);
	initDataPin(&D_Pins, LCD_D_DB7, ILI9341_DATAPIN_7);
	initDataPin(&D_Pins, LCD_D_DB9, ILI9341_DATAPIN_9);
	initDataPin(&D_Pins, LCD_D_DB11, ILI9341_DATAPIN_11);
	initDataPin(&D_Pins, LCD_D_DB12, ILI9341_DATAPIN_12);
	initDataPin(&D_Pins, LCD_D_DB13, ILI9341_DATAPIN_13);
	initDataPin(&D_Pins, LCD_D_DB14, ILI9341_DATAPIN_14);
	initDataPin(&D_Pins, LCD_D_DB15, ILI9341_DATAPIN_15);
}

void ILI9341_Handle_Init(void) {
	ILIHandle.intfMode = ILI9341_MODE_8080_I_16BIT_PARALLEL;

	ILIHandle.pLCDPins = &A_Pins;
	ILIHandle.ILI9341_Parallel_Config.lcdResetPin = LCD_A_REST;
	ILIHandle.ILI9341_Parallel_Config.lcdWRPin = LCD_A_WR;
	ILIHandle.ILI9341_Parallel_Config.lcdRDPin = LCD_A_RD;
	ILIHandle.ILI9341_Parallel_Config.lcdCSPin = LCD_A_CS;
	ILIHandle.ILI9341_Parallel_Config.lcdDCPin = LCD_A_RS;

	ILIHandle.ILI9341_Parallel_Config.xPixels = X_PIXELS;
	ILIHandle.ILI9341_Parallel_Config.yPixels = Y_PIXELS;

	for(int i = 0; i < sizeof(dataPins)/sizeof(GPIO_Pin_Handle_t); i++) {
		ILIHandle.ILI9341_Parallel_Config.dataPins[i] = &dataPins[i];
	}

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
	ILI9341_Handle_Init();

	/* Set rotation to landscape */
	ILI9341_Set_Rotation(1);

	//Perform 60 alternating color screen refreshes
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_RESET);
	for(int i = 0; i < 30; i++) {
		ILI9341_Fill_Screen(ILI9341_COLOR_BLUE, ENABLE);
		ILI9341_Fill_Screen(ILI9341_COLOR_RED, ENABLE);
	}
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(A_Pins.pGPIOx, LCD_A_TEST, GPIO_PIN_RESET);

	while(1);

	return 0;
}
