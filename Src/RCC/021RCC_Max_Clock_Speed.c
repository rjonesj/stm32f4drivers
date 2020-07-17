/*
 * 021RCC_Max_Clock_Speed.c
 *
 *  Created on: Jul 17, 2020
 *      Author: rjonesj
 *
 *  Application configures the system and bus clocks to maximum speed using the main PLL (Phase-locked loop)
 *
 *  AHB 168MHz
 *  APB1 42MHz
 *  APB2 84 MHz
 *
 *  System clock signal (PLL) will be sent over PA8 to read using a logic analyzer or oscilloscope.
 */


#include "stm32f407xx.h"
#include <string.h>

int main(void)
{
    // Resets the clock configuration to the default reset state
    RCC_DeInit();

	uint32_t *pRccCfgrReg = (uint32_t*) (RCC_BASEADDR + 0x08);

	//1. Set the RCC clock configuration register MCO1 to PLL
	*pRccCfgrReg &= ~(0x3 << RCC_CFGR_MCO1); // clear bit 21 - 22 positions
	*pRccCfgrReg |= (0x3 << RCC_CFGR_MCO1); // set bit 21 - 22 positions to PLL

    // Enable external crystal (HSE)
    RCC_HSEConfig(RCC_HSE_ON);
    // Wait until HSE ready to use or not
    ErrorStatus errorStatus = RCC_WaitForHSEStartUp();

    if (errorStatus == SUCCESS)
    {
        // Configure the PLL for 168MHz SysClk and 48MHz for USB OTG, SDIO
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
        // Enable PLL
        RCC_PLLCmd(ENABLE);
        // Wait until main PLL clock ready
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == FS_RESET);

        // Set flash latency
        FLASH_SetLatency(FLASH_Latency_5);

        // AHB 168MHz
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        // APB1 42MHz
        RCC_PCLK1Config(RCC_HCLK_Div4);
        // APB2 84 MHz
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // Set SysClk using PLL
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
    else
    {
        // Do something to indicate that error clock configuration
        while (1);
    }

	//2. Configure PA8 to AF0 mode to behave as MCO signal
    GPIO_Handle_t MCO1Pin;

	//Set all member elements to 0
	memset(&MCO1Pin,0,sizeof(MCO1Pin));

    MCO1Pin.pGPIOx = GPIOA;
    MCO1Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    MCO1Pin.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
    MCO1Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    MCO1Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;

    GPIO_Init(&MCO1Pin);

    while (1);
}
