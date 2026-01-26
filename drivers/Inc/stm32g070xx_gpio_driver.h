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
	uint8_t GPIO_PinMode;					//<Possible Values from @GPIO_PIN_MODES>
	uint8_t GPIO_PinSpeed;					//<Possible Values from @GPIO_PIN_SPEED>
	uint8_t GPIO_PinPuPdControl;			//<Possible Values from @GPIO_PIN_PULLUP_PULLDN>
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* Handle Structure for GPIO Pin*/

typedef struct{
	GPIO_RegDef_t *pGPIOx; // Holds the base address of the GPIO Port it belongs to
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/**************************************** GPIO Specific Macros ****************************************/

//GPIO Pin Numbers
//@GPIO_PIN_NUMBERS
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15




//GPIO Pin Possible Mode MODER (Register)
//@GPIO_PIN_MODES
#define GPIO_MODE_IN 			0			//GPIO MODER Value = 00 (Input)
#define GPIO_MODE_OUT 			1			//GPIO MODER Value = 01 (Output)
#define GPIO_MODE_AF 			2			//GPIO MODER Value = 10	(Alt Function Mode)
#define GPIO_MODE_ANALOG 		3			//GPIO MODER Value = 11 (Analog)
#define GPIO_MODE_IT_FT			4			//GPIO INTERRUPT Falling Trigger Mode
#define GPIO_MODE_IT_RT			5			//GPIO INTERRUPT Rising Trigger Mode
#define GPIO_MODE_IT_RFT		6			//GPIO INTERRUPT Rising and Falling Trigger Mode

//GPIO Pin Possible Output Types OTYPER (Register)
//@GPIO_PIN_OUTPUT
#define GPIO_OP_TYPE_PP			0			//GPIO OTYPER Value = 00 (Push-Pull)
#define GPIO_OP_TYPE_OD			1			//GPIO OTYPER Value = 01 (Open Drain)

//GPIO Pin Possible Output Speeds OSPEEDR (Register)
//@GPIO_PIN_SPEED
#define GPIO_SPEED_VLOW			0			//GPIO OSPEEDR Value = 00 (Very Low)
#define GPIO_SPEED_LOW			1			//GPIO OSPEEDR Value = 01 (Low)
#define GPIO_SPEED_HIGH			2			//GPIO OSPEEDR Value = 10 (High)
#define GPIO_SPEED_VHIGH		3			//GPIO OSPEEDR Value = 11 (Very High)

//GPIO PullUp PullDown PUPDR (Register)
//@GPIO_PIN_PULLUP_PULLDN
#define GPIO_NO_PUPD			0			//GPIO PUPDR Value = 00 (No Pull Up or Pull Down)
#define GPIO_PIN_PU				1			//GPIO PUPDR Value = 01 (Pull Up)
#define GPIO_PIN_PD				2			//GPIO PUPDR Value = 10 (Pull Down)

/**************************************** APIs Supported By this driver ****************************************/
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
