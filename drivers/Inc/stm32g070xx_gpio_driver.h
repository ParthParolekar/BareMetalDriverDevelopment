/*
 * stm32g070_gpio_driver.h
 *
 *  Created on: 25-Jan-2026
 *      Author: PARTH
 */

#ifndef INC_STM32G070XX_GPIO_DRIVER_H_
#define INC_STM32G070XX_GPIO_DRIVER_H_

#include "stm32g070xx.h"

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* Handle Structure for GPIO Pin*/

typedef struct{
	GPIO_RegDef_t *pGPIOx; // Holds the base address of the GPIO Port it belongs to
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

//Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);






#endif /* INC_STM32G070XX_GPIO_DRIVER_H_ */
