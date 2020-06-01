/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 25, 2020
 *      Author: rjonesj
 */


#include "stm32f407xx_gpio_driver.h"

/**
 * Peripheral clock setup
 */

/**
 * @fn			- GPIO PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 * @note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis) {
	if(EnOrDis == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DS();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DS();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DS();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DS();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DS();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DS();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DS();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DS();
		} else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_DS();
		}
	}
}

/**
 * Initialization and De-Initialization
 */

/**
 * @fn			- GPIO GPIO_Init
 * @brief		- This function configures the settings for a GPIO port
 *
 * @param[in]	- Address to GPIO_Handle_t struct to be configured
 *
 * @return		- none
 * @note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp=0; //temporary register

	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//Non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //2 bit field, get position by multiplying field size by pin number
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear bits
		pGPIOHandle->pGPIOx->MODER |= temp;	// set bits
	} else {
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDRE_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp = 0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. Configure the PuPd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		//Calculate AFR register and starting bit position
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		//Configure the alt function register
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
		temp = 0;
	}

}


/**
 * @fn			- GPIO GPIO_DeInit
 * @brief		- This function resets all the registers for a GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 *
 * @return		- none
 * @note		- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if(pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/**
 * Data read and write
 */


/**
 * @fn			- GPIO_ReadFromInputPin
 * @brief		- This function returns the value of the IDR register for a GPIO port pin
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Pin number to read input value
 *
 * @return		- 0 or 1
 * @note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	uint8_t value;
	// shift the bit of the pin to be read to 0 position and bitmask all other positions to get value
	value = (uint8_t)(pGPIOx->IDR >> pinNumber) & 0x1;
	return value;
}


/**
 * @fn			- GPIO_ReadFromInputPort
 * @brief		- This function returns all values from the IDR register for a GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 *
 * @return		- 16 bit IDR register value
 * @note		- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/**
 * @fn			- GPIO_WriteToOutputPin
 * @brief		- This function writes a value to the ODR register for a GPIO port pin
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Pin number to read input value
 * @param[in]	- Value to write to pin, 0 or 1
 *
 * @return		- none
 * @note		- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
	if(value == GPIO_PIN_SET) {
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << pinNumber);
	} else {
		//Write 0, clear the bit at the pin number
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}


/**
 * @fn			- GPIO_WriteToOutputPort
 * @brief		- This function writes a value to the ODR register for a GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- 16 bit value to write to register
 *
 * @return		- none
 * @note		- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	//write value to the output data register for the GPIO port
	pGPIOx->ODR = value;
}

/**
 * @fn			- GPIO_WriteToOutputPort
 * @brief		- This function toggles the value at the ODR register for a GPIO port pin
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Pin number to toggle value
 *
 * @return		- none
 * @note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= (1 << pinNumber);
}

/**
 * IRQ Configuration and ISR handling
 */

/**
 * @fn			- GPIO_IRQConfig
 * @brief		- This function enables or disables interrupts in the NVIC ISER registers
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 * @note		- none
 */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t enOrDis) {
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
 * @fn			- GPIO_IRQConfig
 * @brief		- This function sets the priority of an IRQ number
 *
 * @param[in]	- irqNumber in NVIC vector table
 * @param[in]	- priority to be assigned to irqNumber
 *
 * @return		- none
 * @note		- none
 */
void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
	//First, find the IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber% 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Then set priority at register
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);
}


/**
 * @fn			- GPIO_IRQHandling
 * @brief		- This clears the EXTI PR register corresponding to a pinNumber
 *
 * @param[in]	- pinNumber to be cleared in EXTI PR register
 *
 * @return		- none
 * @note		- none
 */
void GPIO_IRQHandling(uint8_t pinNumber) {
	if(EXTI->PR & ( 1 << pinNumber)) {
		//Clear
		EXTI->PR |= ( 1 << pinNumber);
	}
}
