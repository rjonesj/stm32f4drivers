/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: June 2, 2020
 *      Author: jone1
 */

#include "stm32f407xx_spi_driver.h"

/**
 * Peripheral clock setup
 */

/**
 * @fn			- SPI PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis) {
	if(EnOrDis == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if(pSPIx == SPI1) {
			SPI1_PCLK_DS();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_DS();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */

/**
 * @fn			- SPI_Init
 * @brief		- This function configures the settings for a SPI peripheral
 *
 * @param[in]	- Address to SPI_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	//First configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//bidi mode should be cleared
		//RXONLY bit should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |=  (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the clock speed
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the DFF
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 |= tempreg;
}


/**
 * @fn			- SPI_DeInit
 * @brief		- This function resets all the registers for a SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 *
 * @return		- none
 * @note		- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if(pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if(pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}
