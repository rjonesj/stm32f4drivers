/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: June 2, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx_spi_driver.h"
#include <string.h>

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
	//Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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

	//7. Configure the SSM
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

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

/**
 * Data Send and Receive
 */

/**
 * @fn			- SPI_GetFlagStatus
 * @brief		- This functions returns the Flag status for an SPI SR bit
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Flag macro defined in @SPI_FLAGS in spi header file
 *
 * @return		- FLAG_SET or FLAG_RESET
 * @note		- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName) {
	if(pSPIx->SR & flagName) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}


/**
 * @fn			- SPI_SendData
 * @brief		- This function sends data on SPI peripheral using blocking method (polling).
 * 				  Will wait for all bytes to be transmitted
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Address of data to be sent
 * @param[in]	- Number of bytes to transmit
 *
 * @return		- none
 * @note		- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while(len > 0) {
		//Wait until TXE Buffer is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)) && len > 1) {
			//If DFF is enabled and len > 1, load 2 bytes of data to DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len -= 2;
//			((uint16_t*) pTxBuffer)++; //Incorrect
//			pTxBuffer = (uint16_t*)pTxBuffer + 1; //Correct, with warning
			pTxBuffer += 2; //Correct
		} else {
			//else, load 1 byte of data to DR
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

/**
 * @fn			- SPI_PeripheralControl
 * @brief		- This function enables or disables a SPI peripheral by setting the CR1 SPE bit
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/**
 * @fn			- SPI_SSIConfig
 * @brief		- This function sets the CR1 SSI bit to force the value onto the NSS pin when SSM is enabled
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}
