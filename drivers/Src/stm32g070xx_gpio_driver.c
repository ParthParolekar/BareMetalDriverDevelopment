/*
 * stm32g070xx_gpio_driver.c
 *
 *  Created on: 25-Jan-2026
 *      Author: PARTH
 */


#include "stm32g070xx_gpio_driver.h"

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
	}
	else if(EnOrDi == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
	}

	}
}

//Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; //temporary variable for holding the appropriate register value to be stored for each configuration

	//1.) Configuring the Pin Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_AF){

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear the bits before setting
		pGPIOHandle->pGPIOx->MODER |= temp;


	}else{
		//Write Interrupt Mode Code Here
	}

	temp = 0;

	//2.) Configuring the Pin Speed Register

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear the bits before setting
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3.) Configuring the PullUp PullDown Register

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear the bits before setting
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4.) Configuring the Output Type Register

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << 1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear the bits before setting
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5.) Configuring the Alternate Function Register

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF){
		//Configure the alt function register
		uint8_t LowOrHigh = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; //Check which AF Register to store into; 0=AFLR, 1=AFHR

		if(LowOrHigh == 0){
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8); //Pin % 8 to reduce pin number from 0 to 8
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8); //Clear the bits before setting
			pGPIOHandle->pGPIOx->AFRL |= temp;
			temp = 0;
		}
		else if(LowOrHigh == 1){
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8); //Pin % 8 to reduce pin number from 0 to 8
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8); //Clear the bits before setting
			pGPIOHandle->pGPIOx->AFRH |= temp;
			temp = 0;
		}
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){

		pGPIOx->ODR |= 0x1 << PinNumber;
	}else if(Value == GPIO_PIN_RESET){

		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	//XOR operator `^` is useful for toggling bits.
	//Masking with 0 does not change the state
	//Masking with 1 flips the current state
	/*

	____________________________________________
	|	Bit A		Mask Bit B		Results A^B	|
	|-------------------------------------------|
	|	0			0				0			|
	|	1			0				1			|
	|	0			1				1			|
	|	1			1				0			|
	|___________________________________________|

	*/
	pGPIOx->ODR ^= 1 << PinNumber;
}

//IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}

void GPIO_IRQHandling(uint8_t PinNumber){

}
