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

	//2. Identify the stream which is suitable for the peripheral

	//3. Identify the channel number on which the peripheral sends request

	//4. Program the source address

	//5. Program the destination address

	//6. Program the number of data items to send

	//7. The direction of data transfer. M2p, P2M, M2M

	//8. Program the source and destination data width

	//9. Select Direct or FIFO mode

	//10. Select the FIFO threshold if enabled.

	//11. Enable the circular mode if required

	//12. Single transfer or burst transfer

	//13. Configure the stream priority

	//14. Enable the stream
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
