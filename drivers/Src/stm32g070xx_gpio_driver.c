/*
 * stm32g070xx_gpio_driver.c
 *
 *  Created on: 25-Jan-2026
 *      Author: PARTH
 */


#include "stm32g070xx_gpio_driver.h"

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){

}

//Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

}

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

//IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}

void GPIO_IRQHandling(uint8_t PinNumber){

}
