/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jun 23, 2020
 *      Author: rjonesj
 */
#include "stm32f407xx_usart_driver.h"
#include <string.h>

/**
 * Peripheral clock setup
 */

/**
 * @fn			- USART PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given USART peripheral
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @note		- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis) {
	if(EnOrDis == ENABLE) {
		if(pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if(pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if(pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if(pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if(pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if(pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if(pUSARTx == USART1) {
			USART1_PCLK_DS();
		} else if(pUSARTx == USART2) {
			USART2_PCLK_DS();
		} else if(pUSARTx == USART3) {
			USART3_PCLK_DS();
		} else if(pUSARTx == UART4) {
			UART4_PCLK_DS();
		} else if(pUSARTx == UART5) {
			UART5_PCLK_DS();
		} else if(pUSARTx == USART6) {
			USART6_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */

/**
 * @fn			- USART_Init
 * @brief		- This function configures the settings for a USART peripheral
 *
 * @param[in]	- Address to USART_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	//Temporary variable
	uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/

		//Enable the Clock for given USART peripheral
		USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);

		//Enable USART Tx and Rx engines according to the USART_Mode configuration item
		if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
		{
			//Enable the Receiver bit field
			tempreg|= (1 << USART_CR1_RE);
		}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
		{
			//Enable the Transmitter bit field
			tempreg |= ( 1 << USART_CR1_TE );

		}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
		{
			//Enable the both Transmitter and Receiver bit fields
			tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
		}

	    //Configure the Word length configuration item
		tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M) ;

	    //Configuration of parity control bit fields
		if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
		{
			//Enable the parity control
			tempreg |= ( 1 << USART_CR1_PCE);

			//Enable EVEN parity
			//Not required because by default EVEN parity will be selected once you enable the parity control

		}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
		{
			//Enable the parity control
			tempreg |= ( 1 << USART_CR1_PCE);

		    //Implement the code to enable ODD parity
		    tempreg |= ( 1 << USART_CR1_PS);

		}

	   //Program the CR1 register
		pUSARTHandle->pUSARTx->CR1 |= tempreg;

	/******************************** Configuration of CR2******************************************/

		tempreg=0;

		//Implement the code to configure the number of stop bits inserted during USART frame transmission
		tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

		//Program the CR2 register
		pUSARTHandle->pUSARTx->CR2 |= tempreg;

	/******************************** Configuration of CR3******************************************/

		tempreg=0;

		//Configuration of USART hardware flow control
		if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			//Implement the code to enable CTS flow control
			tempreg |= ( 1 << USART_CR3_CTSE);
		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			//Implement the code to enable RTS flow control
			tempreg |= ( 1 << USART_CR3_RTSE);
		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			//Implement the code to enable both CTS and RTS Flow control
			tempreg |= ( 1 << USART_CR3_CTSE);
			tempreg |= ( 1 << USART_CR3_RTSE);
		}

		pUSARTHandle->pUSARTx->CR3 |= tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

		//Implement the code to configure the baud rate
		//We will cover this in the lecture. No action required here
}

/**
 * @fn			- USART_DeInit
 * @brief		- This function resets all the registers for a USART peripheral
 *
 * @param[in]	- Base address of the USART peripheral
 *
 * @return		- none
 * @note		- none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx) {
	if(pUSARTx == USART1) {
		USART1_REG_RESET();
	} else if(pUSARTx == USART2) {
		USART2_REG_RESET();
	} else if(pUSARTx == USART3) {
		USART3_REG_RESET();
	} else if(pUSARTx == UART4) {
		UART4_REG_RESET();
	} else if(pUSARTx == UART5) {
		UART5_REG_RESET();
	} else if(pUSARTx == USART6) {
		USART6_REG_RESET();
	}
}

/**
 * Data Send and Receive
 */

/**
 * IRQ Configuration and ISR handling
 */


/**
 * @fn			- USART_IRQConfig
 * @brief		- This function enables or disables interrupts in the NVIC ISER registers
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void USART_IRQConfig(uint8_t irqNumber, uint8_t enOrDis) {
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
 * @fn			- USART_IRQPriorityConfig
 * @brief		- This function sets the priority of an IRQ number
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- priority to be assigned to irqNumber
 *
 * @return		- none
 * @note		- none
 */
void USART_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
	//First, find the IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber% 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Then set priority at register
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);
}

/**
 * Other Control APIs
 */

/**
 * @fn			- USART_PeripheralControl
 * @brief		- This function enables or disables a USART peripheral by setting the CR1 UE bit
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSART, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pUSART->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSART->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/**
 * @fn			- USART_GetFlagStatus
 * @brief		- This functions returns the Flag status for an USART SR bit
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Flag macro defined in @USART_FLAGS in usart header file
 *
 * @return		- FLAG_SET or FLAG_RESET
 * @note		- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flagName) {
	if(pUSARTx->SR & flagName) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}

/**
 * @fn			- USART_ClearFlag
 * @brief		- This functions clears the Flag status for an USART SR bit
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Flag macro defined in @USART_FLAGS in usart header file
 *
 * @return		- FLAG_SET or FLAG_RESET
 * @note		- none
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t flagName) {
	pUSARTx->SR &= ~(flagName);
}

/**
 * Application callback
 */
//void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEvent);
