/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: June 2, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx_spi_driver.h"
#include <string.h>

/**
 * private handler functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
			pTxBuffer += 2;
		} else {
			//else, load 1 byte of data to DR
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

/**
 * @fn			- SPI_ReceiveData
 * @brief		- This function receives data on SPI peripheral using blocking method (polling).
 * 				  Will wait for all bytes to be received
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Address of buffer to receive data
 * @param[in]	- Number of bytes to receive
 *
 * @return		- none
 * @note		- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while(len > 0) {
		//Wait until RX Buffer is not empty
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)) && len > 1) {
			//If DFF is enabled and len > 1, load 2 bytes of data to RxBuffer from DR
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len -= 2;
			pRxBuffer += 2;
		} else {
			//else, load 1 byte of data  to RxBuffer from DR
			*(pRxBuffer) = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

/**
 * @fn			- SPI_SendDataIT
 * @brief		- This function sends data on SPI peripheral using interrupt method.
 *
 * @param[in]	- Address of the SPI Handle struct
 * @param[in]	- Address of data to be sent
 * @param[in]	- Number of bytes to transmit
 *
 * @return		- none
 * @note		- none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->txState;

	if(state != SPI_BUSY_TX) {
		//1. Save the TX buffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is complete
		state = SPI_BUSY_TX;
		pSPIHandle->txState = state;
		
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}

/**
 * @fn			- SPI_ReceiveDataIT
 * @brief		- This function receives data on SPI peripheral using interrupt method.
 *
 * @param[in]	- Address of the SPI Handle struct
 * @param[in]	- Address of buffer to receive data
 * @param[in]	- Number of bytes to receive
 *
 * @return		- none
 * @note		- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->rxState;

	if(state != SPI_BUSY_RX) {
		//1. Save the RX buffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = len;

		//2. Mark the SPI state as busy in receiving so that no other code can take over same SPI peripheral until receive is complete
		state = SPI_BUSY_RX;
		pSPIHandle->rxState = state;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Enable the ERRIE control bit to get interrupt whenever OVR flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_ERRIE);
	}

	return state;
}


/**
 * Other Control APIs
 */

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

/**
 * @fn			- SPI_SSOEConfig
 * @brief		- This function sets the CR2 SSOE bit to enable slave select output enable
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/**
 * @fn			- SPI_ClearOVRFlag
 * @brief		- This function clears the OVR SR flag by reading the DR, then the SR
 *
 * @param[in]	- Base address of the SPI peripheral
 *
 * @return		- none
 * @note		- none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/**
 * @fn			- SPI_CloseTransmission
 * @brief		- This function terminates transmission if state is in SPI_BUSY_TX
 *
 * @param[in]	- Address of the SPI Handle struct
 *
 * @return		- none
 * @note		- none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	//Close the SPI transmission and inform application that TX is over when txLen is 0
	//Prevent interrupts from setting of TXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = SPI_READY;
}

/**
 * @fn			- SPI_CloseReception
 * @brief		- This function terminates reception if state is in SPI_BUSY_RX
 *
 * @param[in]	- Address of the SPI Handle struct
 *
 * @return		- none
 * @note		- none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	//Close the SPI reception and inform application that RX is over when rxLen is 0
	//Prevent interrupts from setting of RXNE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
}


/**
 * @fn			- SPI_VerifyResponse
 * @brief		- This function returns 1 if an ack byte is given (0xF5), else will return 0
 *
 * @param[in]	- Byte containing response to verify
 *
 * @return		- none
 * @note		- none
 */
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}


/**
 * Application Callback
 */

/**
 * @fn			- SPI_ApplicationEventCallback
 * @brief		- This function is to be overridden by the application to implement handling after SPI events
 *
 * @param[in]	- Address of the SPI Handle struct
 * @param[in]	- Possible appEvent defined in @SPI_EVENT
 *
 * @return		- none
 * @note		- none
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent) {

}

/**
 * IRQ Configuration and ISR handling
 */

/**
 * @fn			- SPI_IRQConfig
 * @brief		- This function enables or disables interrupts in the NVIC ISER registers
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void SPI_IRQConfig(uint8_t irqNumber, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		if(irqNumber <= 31) {
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber < 64) {
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber >= 64 && irqNumber < 96) {
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (irqNumber % 64));
		}
	} else {
		if(irqNumber <= 31) {
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber < 64) {
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber >= 64 && irqNumber < 96) {
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (irqNumber % 64));
		}
	}
}

/**
 * @fn			- SPI_IRQPriorityConfig
 * @brief		- This function sets the priority of an IRQ number
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- priority to be assigned to irqNumber
 *
 * @return		- none
 * @note		- none
 */
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
	//First, find the IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber% 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Then set priority at register
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);
}


/**
 * @fn			- SPI_IRQHandling
 * @brief		- This clears the EXTI PR register corresponding to a pinNumber
 *
 * @param[in]	- Address to SPI Handle struct
 *
 * @return		- none
 * @note		- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	uint8_t temp1, temp2;
	//First check for TXE flag
	temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2) {
		//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2) {
		//Handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//Check for overrun error
	temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2) {
		//Handle OVR
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/**
 * private handler functions
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) && pSPIHandle->txLen > 1) {
		//If DFF is enabled and len > 1, load 2 bytes of data to DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->txLen -= 2;
		pSPIHandle->pTxBuffer += 2;
	} else {
		//else, load 1 byte of data to DR
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->txLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->txLen == 0) {
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) &&  pSPIHandle->rxLen > 1) {
		//If DFF is enabled and len > 1, load 2 bytes of data to RxBuffer from DR
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen -= 2;
		pSPIHandle->pRxBuffer += 2;
	} else {
		//else, load 1 byte of data  to RxBuffer from DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->rxLen == 0) {
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	//1. Clear the OVR flag by reading the DR, then the SR
	if(pSPIHandle->txState != SPI_BUSY_TX) {
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
