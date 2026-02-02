/*
 * LEDUserBtn.c
 *
 *  Created on: 30-Jan-2026
 *      Author: Parth
 */

#include "stm32g070xx.h"
#include "stm32g070xx_gpio_driver.h"
#include <stdint.h>

void delay(void){
	for(uint32_t i = 0; i < 50000; i++);
}

int main(void){

	GPIO_Handle_t LEDPin;
	LEDPin.pGPIOx = GPIOA;
	LEDPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	LEDPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LEDPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	LEDPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LEDPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t  BtnPin;
	BtnPin.pGPIOx = GPIOC;
	BtnPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BtnPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	BtnPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//	BtnPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//	BtnPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&LEDPin);
	GPIO_Init(&BtnPin);

	int currentBtnState = 0;
	int lastBtnState = 0;

	while(1){

		currentBtnState = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13);

		if(currentBtnState != lastBtnState){
			delay();

			int BtnPressed;
			BtnPressed = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13);

			if(!BtnPressed){
				GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			}

			lastBtnState = currentBtnState;
		}

		delay();
	}
	return 0;
}

