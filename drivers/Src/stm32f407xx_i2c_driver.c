/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: June 12, 2020
 *      Author: rjonesj
 */

#include "stm32f407xx_i2c_driver.h"
#include <string.h>

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

/**
 * Private functions
 */
static uint32_t RCC_GetPLLOutputClock();
static uint32_t RCC_GetPCLK1_Value();
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static uint32_t RCC_GetPLLOutputClock() {
	//TODO: Implement function
	return 0;
}

static uint32_t RCC_GetPCLK1_Value() {
	uint32_t pclk1, systemclk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0) {
		//HSI oscillator
		systemclk = 16000000;
	} else if(clksrc == 1) {
		//HSE oscillator
		systemclk = 8000000;
	} else if(clksrc == 2) {
		//PLL
		systemclk = RCC_GetPLLOutputClock();
	}

	//AHBP prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	//APB1 prescaler
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (systemclk / ahbp) / apb1p;

	return pclk1;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress) {
	slaveAddress = (slaveAddress << 1);
	slaveAddress &= ~(0x1);
	pI2Cx->DR = slaveAddress;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
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

	//Configure CR1 register (ACK bit)
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

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
 *
 * @return		- none
 * @note		- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress) {
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag has been set in SR1
	//   Note: SCL will be stretched (pulled to LOW) until SB bit is cleared
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w (0). (Total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddress);

	//4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//   Note: SCL will be stretched until ADDR flag is cleared.
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDis) {
	if(enOrDis == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
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

