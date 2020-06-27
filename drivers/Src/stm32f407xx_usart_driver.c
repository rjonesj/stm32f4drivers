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
		USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

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

		    //Enable ODD parity
		    tempreg |= ( 1 << USART_CR1_PS);

		}

	   //Program the CR1 register
		pUSARTHandle->pUSARTx->CR1 |= tempreg;

	/******************************** Configuration of CR2******************************************/

		tempreg=0;

		//Configure the number of stop bits inserted during USART frame transmission
		tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

		//Program the CR2 register
		pUSARTHandle->pUSARTx->CR2 |= tempreg;

	/******************************** Configuration of CR3******************************************/

		tempreg=0;

		//Configuration of USART hardware flow control
		if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			//Enable CTS flow control
			tempreg |= ( 1 << USART_CR3_CTSE);
		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			//Enable RTS flow control
			tempreg |= ( 1 << USART_CR3_RTSE);
		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			//Enable both CTS and RTS Flow control
			tempreg |= ( 1 << USART_CR3_CTSE);
			tempreg |= ( 1 << USART_CR3_RTSE);
		}

		pUSARTHandle->pUSARTx->CR3 |= tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

		//Configure the baud rate
		USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
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
 * @fn			- USART_SendData
 * @brief		- This function sends data on USART peripheral using blocking method (polling).
 * 				  Will wait for all bytes to be transmitted
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Address of data to be sent
 * @param[in]	- Number of bytes to transmit
 *
 * @return		- none
 * @note		- none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint16_t *pdata;
   //Loop over until "len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			//Increment the buffer address
			pTxBuffer++;
		}
	}

	//Wait until TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/**
 * @fn			- USART_ReceiveData
 * @brief		- This function receives data on USART peripheral using blocking method (polling).
 * 				  Will wait for all bytes to be received
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Address of buffer to receive data
 * @param[in]	- Number of bytes to receive
 *
 * @return		- none
 * @note		- none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
   //Loop over until "len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = pUSARTHandle->pUSARTx->DR;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/**
 * @fn			- USART_SendDataIT
 * @brief		- This function sends data on USART peripheral using interrupt method.
 *
 * @param[in]	- Address of the USART Handle struct
 * @param[in]	- Address of data to be sent
 * @param[in]	- Number of bytes to transmit
 *
 * @return		- none
 * @note		- none
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len) {
	uint8_t txstate = pUSARTHandle->txState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->txLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);

	}

	return txstate;
}

/**
 * @fn			- USART_ReceiveDataIT
 * @brief		- This function receives data on USART peripheral using interrupt method.
 *
 * @param[in]	- Address of the USART Handle struct
 * @param[in]	- Address of buffer to receive data
 * @param[in]	- Number of bytes to receive
 *
 * @return		- none
 * @note		- none
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t rxstate = pUSARTHandle->rxState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->rxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxState = USART_BUSY_IN_RX;

		//Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

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
 * @fn			- USART_SetBaudRate
 * @brief		- This function configures the baud rate in the BRR register
 *
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Baud rate value to be programmed
 *
 * @return		- none
 * @note		- none
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
	//Variable to hold the APB clock
	uint32_t PCLKx;
	//Variable to hold USART div
	uint32_t usartdiv;
	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB2 bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2_Value();
	}else
	{
	   PCLKx = RCC_GetPCLK1_Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else
	{
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}else
	{
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/**
 * Application callback
 */
//void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEvent);
