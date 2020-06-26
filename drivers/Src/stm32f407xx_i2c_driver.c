/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: June 12, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include <string.h>

/**
 * Private functions
 */
#define READ		1
#define WRITE		0

static void delay(void);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress, uint8_t readOrWrite);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void delay(void) {
	for(int i = 0; i < 250; i++);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress, uint8_t readOrWrite) {
	slaveAddress = (slaveAddress << 1);
	if(readOrWrite == WRITE) {
		slaveAddress &= ~(0x1);
	} else if(readOrWrite == READ) {
		slaveAddress |= (0x1);
	}
	pI2Cx->DR = slaveAddress;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummyRead;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		//device is in master mode
		if(pI2CHandle->txRxState == I2C_BUSY_RX) {
			if(pI2CHandle->rxSize ==1) {
				//First, disable the ACK
				I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		} else {
			//Clear the ADDR flag
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	} else {
		//device is in slave mode
		//Clear the ADDR flag
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->rxSize == 1) {
		// Load the data into RX buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// Decrement the rxLen
		pI2CHandle->rxLen--;
	} else if(pI2CHandle->rxSize > 1) {
		if(pI2CHandle->rxLen == 2) {
			//Clear the ACK bit
			I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
		}
		// Load the data into RX buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// Decrement the rxLen
		pI2CHandle->rxLen--;
		// Increment the buffer address
		pI2CHandle->pRxBuffer++;
	}
	if(pI2CHandle->rxLen == 0) {
		//Close the I2C data reception and notify the application
		//1. Generate STOP condition
		if(pI2CHandle->sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Reset all the member elements of the handle structure
		I2C_CloseReception(pI2CHandle);

		//3. Notify the application transmission is complete
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
	}
}


/**
 * Peripheral clock setup
 */

/**
 * @fn			- I2C PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given I2C peripheral
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @note		- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis) {
	if(EnOrDis == ENABLE) {
		if(pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if(pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if(pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if(pI2Cx == I2C1) {
			I2C1_PCLK_DS();
		} else if(pI2Cx == I2C2) {
			I2C2_PCLK_DS();
		} else if(pI2Cx == I2C3) {
			I2C3_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */

/**
 * @fn			- I2C_Init
 * @brief		- This function configures the settings for a I2C peripheral
 *
 * @param[in]	- Address to I2C_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0;

	//Enable the clock for the I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Configure CR2 register (FREQ field)
	tempreg = 0;
	tempreg = RCC_GetPCLK1_Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	//Configure own address register 1
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD1;
	tempreg |= (1 << I2C_OAR1_BIT14ON);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//CCR Register configuration
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//standard mode
		ccr_value = RCC_GetPCLK1_Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	} else {
		//fast mode
		//Enable FM mode
		tempreg |= (1 << I2C_CCR_FS);
		//Set Duty Cycle
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		//Calculate CCR value
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = RCC_GetPCLK1_Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			ccr_value = RCC_GetPCLK1_Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;

	//Configure TRISE register
	uint8_t trise;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//standard mode
		trise = (RCC_GetPCLK1_Value() / 1000000U) + 1;
	} else {
		//fast mode
		trise = (RCC_GetPCLK1_Value() / 300000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);
}


/**
 * @fn			- I2C_DeInit
 * @brief		- This function resets all the registers for a I2C peripheral
 *
 * @param[in]	- Base address of the I2C peripheral
 *
 * @return		- none
 * @note		- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if(pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/**
 * Data Send and Receive
 */

/**
 * @fn			- I2C_MasterSendData
 * @brief		- This function sends data to slave device over I2C peripheral
 *
 * @param[in]	- Address of I2C_Handle_t struct
 * @param[in]	- Address to data buffer to transmit
 * @param[in]	- Length of data in bytes
 * @param[in]	- Address of slave device to receive data
 * @param[in]	- I2C_ENABLE_SR or I2C_DISABLE_SR macro
 *
 * @return		- none
 * @note		- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR) {
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag has been set in SR1
	//   Note: SCL will be stretched (pulled to LOW) until SB bit is cleared
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w (0). (Total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddress, WRITE);

	//4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//   Note: SCL will be stretched until ADDR flag is cleared.
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while(len > 0) {
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. When Len becomes zero, wait for TXE=1 and BTF=1 (Byte transfer finished) before generating the STOP condition
	//   Note: TXE=1, BTF=1 means that both SR and DR are empty and next transmission should begin
	//         When BTF=1 SCL will be stretched.
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition.  Master does not need to wait for completion of the STOP condition
	//   Note: Generating STOP automatically clears the BTF
	if(enSR == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/**
 * @fn			- I2C_MasterReceiveData
 * @brief		- This function receives data from slave device over I2C peripheral
 *
 * @param[in]	- Address of I2C_Handle_t struct
 * @param[in]	- Address of data buffer to store data
 * @param[in]	- Length of data in bytes
 * @param[in]	- Address of slave device to receive data
 * @param[in]	- I2C_ENABLE_SR or I2C_DISABLE_SR macro
 *
 * @return		- none
 * @note		- none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR) {
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag has been set in SR1
	//   Note: SCL will be stretched (pulled to LOW) until SB bit is cleared
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to r (1). (Total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddress, READ);

	//4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//Procedure to read only 1 byte from slave
	if(len == 1) {
		//Disable ACKing
		I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition
		if(enSR == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data in to buffer
		*pRxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
	} else if(len > 1) {
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read the data until len becomes zero
		for(uint32_t i = len; i > 0; i--) {
			//Wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			//if last 2 bytes are remaining
			if(i == 2) {
				//Clear the ack bit
				I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);

				//Generate STOP condition
				if(enSR == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//Read the data in to buffer
			*pRxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;

			//Increment the buffer address
			pRxBuffer++;
		}
	}

	//Restore ACKing
	I2C_ACKControl(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_ACKControl);
}


/**
 * @fn			- I2C_MasterSendDataIT
 * @brief		- This function sends data to slave device over I2C peripheral
 *
 * @param[in]	- Address of I2C_Handle_t struct
 * @param[in]	- Address to data buffer to transmit
 * @param[in]	- Length of data in bytes
 * @param[in]	- Address of slave device to receive data
 * @param[in]	- I2C_ENABLE_SR or I2C_DISABLE_SR macro
 *
 * @return		- state of I2C peripheral
 * @note		- none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR) {
		uint8_t busystate = pI2CHandle->txRxState;

		if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
		{
			//Store TX values
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->txLen = len;
			pI2CHandle->txRxState = I2C_BUSY_TX;
			pI2CHandle->devAddr = slaveAddress;
			pI2CHandle->sr = enSR;

			//Generate the START condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		}

		return busystate;

}

/**
 * @fn			- I2C_MasterReceiveDataIT
 * @brief		- This function receives data from slave device over I2C peripheral
 *
 * @param[in]	- Address of I2C_Handle_t struct
 * @param[in]	- Address of data buffer to store data
 * @param[in]	- Length of data in bytes
 * @param[in]	- Address of slave device to receive data
 * @param[in]	- I2C_ENABLE_SR or I2C_DISABLE_SR macro
 *
 * @return		- state of I2C peripheral
 * @note		- none
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t enSR) {
		uint8_t busystate = pI2CHandle->txRxState;

		if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->rxLen = len;
			pI2CHandle->txRxState = I2C_BUSY_RX;
			pI2CHandle->rxSize = len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->devAddr = slaveAddress;
			pI2CHandle->sr = enSR;

			//Generate the START condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		}

		return busystate;
}

/**
 * @fn			- I2C_SlaveSendData
 * @brief		- This function sends one byte of data when device is in slave mode
 *
 * @param[in]	- Address of I2C_RegDef_t struct
 * @param[in]	- Data to be sent
 *
 * @return		- none
 * @note		- none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
	pI2Cx->DR = data;
}

/**
 * @fn			- I2C_SlaveReceiveData
 * @brief		- This function receives one byte of data when device is in slave mode
 *
 * @param[in]	- Address of I2C_RegDef_t struct
 *
 * @return		- received data byte
 * @note		- none
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
	return (uint8_t) pI2Cx->DR;
}

/**
 * IRQ Configuration and ISR handling
 */

/**
 * @fn			- I2C_IRQConfig
 * @brief		- This function enables or disables interrupts in the NVIC ISER registers
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void I2C_IRQConfig(uint8_t irqNumber, uint8_t enOrDis) {
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
 * @fn			- I2C_IRQPriorityConfig
 * @brief		- This function sets the priority of an IRQ number
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- priority to be assigned to irqNumber
 *
 * @return		- none
 * @note		- none
 */
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
	//First, find the IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber% 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Then set priority at register
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);
}

/**
 * @fn			- I2C_EV_IRQHandling
 * @brief		- This function handles the event interrupts for I2C peripheral in both master and slave modes
 *
 * @param[in]	- Address of I2C handle struct
 *
 * @return		- none
 * @note		- none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle interrupt generated by SB event
	//   Note: SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3) {
		//Interrupt is generated because of SB event, will not execute in slave mode
		//Execute address phase to clear SB bit
		if(pI2CHandle->txRxState == I2C_BUSY_TX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddr, WRITE);
		} else if(pI2CHandle->txRxState == I2C_BUSY_RX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddr, READ);
		}
	}

	//2. Handle interrupt generated by ADDR event
	//   Note: When in master mode: Address is sent
	//         When in slave mode: Address is matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3) {
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle interrupt generated by BTF (Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3) {
		if(pI2CHandle->txRxState == I2C_BUSY_TX) {
			//Make sure TXE is also set (all transmission has completed)
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
				//Verify len is 0
				if(pI2CHandle->txLen == 0) {
					//1. Generate the STOP condition if SR is disabled
					if(pI2CHandle->sr == I2C_DISABLE_SR) {
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset all the member elements of the handle structure
					I2C_CloseTransmission(pI2CHandle);

					//3. Notify the application transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
				}
			}
		} else if(pI2CHandle->txRxState == I2C_BUSY_RX) {
			//Nothing to do
		}
	}

	//4. Handle interrupt generated by STOPF event
	//   Note: Stop detection flag is applicable only in slave mode. For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3) {
		// Clear the STOPF (Read SR1, then write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify the application transmission is complete
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3) {
		//Verify device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			if(pI2CHandle->txRxState == I2C_BUSY_TX) {
				if(pI2CHandle->txLen > 0) {
					// Load the data into DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
					// Decrement the txLen
					pI2CHandle->txLen--;
					// Increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
		} else {
			//device in slave mode
			//Verify slave is in transmitter mode and AF flag is not set
			delay(); // allow time for AF flag to be set
			if((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) && (!((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_AF) & 0x1))) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3) {
		//Verify device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			if(pI2CHandle->txRxState == I2C_BUSY_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else {
			//device in slave mode
			//Verify slave is in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}


/**
 * @fn			- I2C_ER_IRQHandling
 * @brief		- This function handles the error interrupts for I2C peripheral in both master and slave modes
 *
 * @param[in]	- Address of I2C handle struct
 *
 * @return		- none
 * @note		- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1,temp2;

	//Get the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//Clear the bus error flag (Write 0 to BERR bit)
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//Clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

	/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//Clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//Clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//Clear the Timeout error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/**
 * Other Control APIs
 */

/**
 * @fn			- I2C_PeripheralControl
 * @brief		- This function enables or disables a I2C peripheral by setting the CR1 PE bit
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void I2C_PeripheralControl(I2C_Handle_t *pI2CHandle, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_PE);

		//Enable Ack bit after PE=1
		I2C_ACKControl(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_ACKControl);
	} else {
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/**
 * @fn			- I2C_GetFlagStatus
 * @brief		- This functions returns the Flag status for an I2C SR bit
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Flag macro defined in @I2C_FLAGS in i2c header file
 *
 * @return		- FLAG_SET or FLAG_RESET
 * @note		- none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName) {
	if(pI2Cx->SR1 & flagName) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}

/**
 * @fn			- I2C_ACKControl
 * @brief		- This function enables or disables hw ACKing for an I2C peripheral
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/**
 * @fn			- I2C_CloseTransmission
 * @brief		- This function stops interrupt based transmission and resets the handle structure
 *
 * @param[in]	- Base address of the I2C handle struct
 *
 * @return		- none
 * @note		- none
 */
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle) {
	//Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLen = 0;
	pI2CHandle->txRxState = I2C_READY;
	pI2CHandle->devAddr = 0;
	pI2CHandle->sr = I2C_DISABLE_SR;
}

/**
 * @fn			- I2C_CloseTransmission
 * @brief		- This function stops interrupt based reception and resets the handle structure
 *
 * @param[in]	- Base address of the I2C handle struct
 *
 * @return		- none
 * @note		- none
 */
void I2C_CloseReception(I2C_Handle_t *pI2CHandle) {
	//Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->rxLen = 0;
	pI2CHandle->txRxState = I2C_READY;
	pI2CHandle->rxSize = 0;
	pI2CHandle->devAddr = 0;
	pI2CHandle->sr = I2C_DISABLE_SR;
	I2C_ACKControl(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_ACKControl);
}

/**
 * @fn			- I2C_GenerateStopCondition
 * @brief		- This function generates the STOP condition by setting the STOP bit in the CR1 register
 *
 * @param[in]	- Base address of the I2C_RegDef_t struct
 *
 * @return		- none
 * @note		- none
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/**
 * @fn			- I2C_SlaveConfigureCallbackEvents
 * @brief		- This enables or disables interrupt events when device is configured in slave mode
 *
 * @param[in]	- Base address of the I2C_RegDef_t struct
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void I2C_SlaveConfigureCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

/**
 * Application Callback
 */

/**
 * @fn			- I2C_ApplicationEventCallback
 * @brief		- This function is to be overridden by the application to implement handling after I2C events
 *
 * @param[in]	- Address of the I2C Handle struct
 * @param[in]	- Possible appEvent defined in @I2C_EVENT
 *
 * @return		- none
 * @note		- none
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent) {

}

