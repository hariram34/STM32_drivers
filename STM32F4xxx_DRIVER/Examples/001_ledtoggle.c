/*
 * 001_ledtoggle.c
 *
 *  Created on: Mar 19, 2020
 *      Author: Hariram Asokan
 *
 *		Push Pull Config:
 *      This program is used to toggle an LED using Push pull configuration
 *
 *      Open drain :
 *      This program is used to toggle an LED using open drain
 *      configuration. Internal pull up resistor is enabled to
 *      provide high state.
 *
 */

#include "stm32f446xx.h"
void delay(){
	for(int i =0;i<50000;i++);
}
int main(){

	GPIO_Handle_t led;
	led.pGPIOx = GPIOA;
	/*
	 * Push Pull config
	 */
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN5;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPUPDcontrol = GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;


	/*/Open Drain Configuration
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN5;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OD;
	led.GPIO_PinConfig.GPIO_PinPUPDcontrol = GPIO_PU;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	*/

	//GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_init(&led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
		delay();
	}

	return 0;
}
