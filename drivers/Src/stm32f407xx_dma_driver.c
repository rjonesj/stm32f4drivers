/*
 * stm32f407xx_dma_driver.c
 *
 *  Created on: Jul 13, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx_dma_driver.h"

/**
 * Peripheral clock setup
 */

/**
 * @fn			- DMA_PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given DMA Controller
 *
 * @param[in]	- Base address of the DMA Controller
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @note		- none
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDis){
	if(EnOrDis == ENABLE) {
		if(pDMAx == DMA1) {
			DMA1_PCLK_EN();
		} else if(pDMAx == DMA2) {
			DMA2_PCLK_EN();
		}
	} else {
		if(pDMAx == DMA1) {
			DMA1_PCLK_DS();
		} else if(pDMAx == DMA2) {
			DMA2_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */

/**
 * @fn			- DMA_Init
 * @brief		- This function configures the settings for a DMA controller
 *
 * @param[in]	- Address to DMA_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void DMA_Init(DMA_Handle_t *pDMAHandle) {
	//1. Enable the peripheral clock for the DMA
	DMA_PeriClockControl(pDMAHandle->pDMAx, ENABLE);

	//Stream configuration registers

	//2. Identify the stream which is suitable for the peripheral
	DMA_Stream_RegDef_t *pStream;
	pStream = pDMAHandle->pDMAStream;

	//3. Program the source address (memory)
	pStream->M0AR = (uint32_t) pDMAHandle->DMA_Config.sourceAddress;

	//4. Program the destination address (peripheral)
	pStream->PAR = (uint32_t) pDMAHandle->DMA_Config.destAddress;

	//5. Program the number of data items to send
	pStream->NDTR = pDMAHandle->DMA_Config.len;

	//6. The direction of data transfer. M2P, P2M, or M2M
	pStream->CR |= (pDMAHandle->DMA_Config.transferDirection << DMA_SxCR_DIR);

	//7. Program the source and destination data width
	pStream->CR &= ~(0x3 << DMA_SxCR_PSIZE);
	pStream->CR |= (pDMAHandle->DMA_Config.periphDataSize << DMA_SxCR_PSIZE);
	pStream->CR &= ~(0x3 << DMA_SxCR_MSIZE);
	pStream->CR |= (pDMAHandle->DMA_Config.memDataSize << DMA_SxCR_MSIZE);

	//7a. Program the memory / peripheral auto increment mode
	pStream->CR |= (pDMAHandle->DMA_Config.memIncrementMode << DMA_SxCR_MINC);
	pStream->CR |= (pDMAHandle->DMA_Config.periphIncrementMode << DMA_SxCR_PINC);

	//8. Select Direct or FIFO mode
	pStream->FCR |= (pDMAHandle->DMA_Config.fifoMode << DMA_SxFCR_DMDIS);

	//9. Select the FIFO threshold if enabled.
	if(pDMAHandle->DMA_Config.fifoMode) {
		pStream->FCR &= ~(0x3 << DMA_SxFCR_FTH);
		pStream->FCR |= (pDMAHandle->DMA_Config.fifoThreshold << DMA_SxFCR_FTH);
	}

	//10. Enable the circular mode if required
	pStream->CR |= (pDMAHandle->DMA_Config.circularMode << DMA_SxCR_CIRC);

	//11. Single transfer or burst transfer

	//12. Configure the stream priority
	pStream->CR |= (pDMAHandle->DMA_Config.priority << DMA_SxCR_PL);

	//13. Program the channel selection
	pStream->CR &= ~(0x7 << DMA_SxCR_CHSEL);
	pStream->CR |= (pDMAHandle->DMA_Config.channel << DMA_SxCR_CHSEL);
}

/**
 * @fn			- enable_dma_stream
 * @brief		- This function enables the configured DMA stream
 *
 * @param[in]	- Address to DMA_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void enable_dma_stream(DMA_Handle_t *pDMAHandle) {
	//Enable the stream
	pDMAHandle->pDMAStream->CR |= (1 << DMA_SxCR_EN);
}

/**
 * @fn			- dma_interrupt_configuration
 * @brief		- This function enables the global interrupts for all DMA events
 *
 * @param[in]	- Address to DMA_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void dma_interrupt_configuration(DMA_Handle_t *pDMAHandle) {
	DMA_Stream_RegDef_t *pStream = pDMAHandle->pDMAStream;

	//1. Half-transfer IE (HTIE)
	pStream->CR  |= (1 << DMA_SxCR_HTIE);

	//2. Transfer complete IE (TCIE)
	pStream->CR |= (1 << DMA_SxCR_TCIE);

	//3. Transfer error IE (TEIE)
	pStream->CR |= (1 << DMA_SxCR_TEIE);

	//4. FIFO overrun/underrun IE (FEIE)
	pStream->FCR |= (1 << DMA_SxFCR_FEIE);

	//5. Direct mode error (DMEIE)
	pStream->CR |= (1 << DMA_SxCR_DMEIE);

	//6. Enable the IRQ for DMA1 stream6 global interrupt in NVIC
	NVIC_IRQConfig(IRQ_NO_DMA1_STREAM6, ENABLE);
}


/**
 * @fn			- DMA_DeInit
 * @brief		- This function resets all the registers for a DMA controller
 *
 * @param[in]	- Base address of the DMA controller
 *
 * @return		- none
 * @note		- none
 */
void DMA_DeInit(DMA_RegDef_t *pDMAx) {
	if(pDMAx == DMA1) {
		DMA1_REG_RESET();
	} else if(pDMAx == DMA2) {
		DMA2_REG_RESET();
	}
}

/**
 * Data read and write
 */

/**
 * IRQ Configuration and ISR handling
 */
void DMA_IRQHandling(uint8_t pinNumber);
