/*
 * 017ADC_Sensor_Read.c
 *
 *  Created on: Jul 1, 2020
 *      Author: rjonesj
 *
 * When button on master is pressed, STM32 should read analog data from PA1 and print the value when using semi-hosting debug.
 * Analog data can come from a sensor such as a capacitive soil moisture sensor.
 * The in min and max values should be set to the values read at minimum and maximum values for the sensor (in air and submerged),
 * and the out min and max should be the displayed values output to the user (0 to 100).
 *
 * Connections
 * Sensor			STM32
 * GND    <------>  GND
 * VCC    <------>  3V
 * AUOT   <------>  PA1
 */

# include "stm32f407xx.h"
# include <string.h>
# include <stdio.h>

ADC_Handle_t ADC1Handle;

uint16_t in_min_value = 3450;
uint16_t in_max_value = 1700;
uint8_t out_min_value = 0;
uint8_t out_max_value = 100;

extern void initialise_monitor_handles();

void ADC1_GPIOInit(void) {
	GPIO_Handle_t ADCPins;

	ADCPins.pGPIOx = GPIOA;
	ADCPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	ADCPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ADCPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ADCPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//ADC1_IN1
	ADCPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&ADCPins);
}

void ADC1_Init(void) {
	ADC1Handle.pADCx = ADC1;
	ADC1Handle.ADC_Config.ADC_Channel = ADC_CH_IN1;
	ADC1Handle.ADC_Config.ADC_Mode = ADC_MODE_CONTINUOUS;
	ADC1Handle.ADC_Config.ADC_Align = ADC_ALIGN_RIGHT;
	ADC1Handle.ADC_Config.ADC_Resolution = ADC_RESOLUTION_12;

	ADC_Init(&ADC1Handle);
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

void delay(void) {
	for(int i = 0; i < 500000; i++);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) {
	uint16_t data;

	initialise_monitor_handles();

	//Initialize GPIO Pins for ADC
	ADC1_GPIOInit();

	//Initialize ADC1 peripheral
	ADC1_Init();

	//Initialize button
	GPIO_ButtonInit();

	//Enable the ADC1 peripheral
	ADC_PeripheralControl(ADC1Handle.pADCx, ENABLE);

	//Start Conversion
	ADC_StartConversion(ADC1Handle.pADCx);

	while(1) {
		//Wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay(); // debounce button

		//Read data from ADC
		data = ADC_ReadData(ADC1Handle.pADCx);
		long output = map(data, in_min_value, in_max_value, out_min_value, out_max_value);

		printf("Read value: %d Mapped value: %ld\n", data, output);
	}
}
