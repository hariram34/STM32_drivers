/*
 * 002_button.c
 *
 *  Created on: Apr 10, 2020
 *      Author: welcome
 */



#include "stm32f446xx.h"

int sampleTime = 1000;
void delay(){
	for(int i =0;i<60000;i++);
}
void button_delay(){
	for(int i =0;i<100;i++);
}
int buttonPress(GPIO_Reg *pGPIOx, uint8_t pinNumber)
{
	int pressedCount = 0, releasedCount = 0,time = 0;
	while(time<sampleTime){
	if(GPIO_ReadInputPin(pGPIOx, pinNumber)==0){
		pressedCount++;
	}else{
		releasedCount++;
	}
	button_delay();
	time++;
	}
	if(pressedCount>releasedCount){
		return 1;
	}else{
		return 0;
	}


}
int main(){

	GPIO_Handle_t led,button;
	led.pGPIOx = GPIOA;
	/*
	 * Push Pull config
	 */
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN5;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPUPDcontrol = GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	//button configuration
	button.pGPIOx = GPIOC;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinPUPDcontrol = GPIO_NO_PUPD;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//GPIO_PeriClockControl(GPIOA,ENABLE);
	//GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_init(&led);
	GPIO_init(&button);

	while(1)
	{
		/*
		//button is initially pulled high and goes low when pressed
		//debouncing has been taken care by adding a small delay
		if(GPIO_ReadInputPin(GPIOC, GPIO_PIN13) == 0)
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
			//GPIO_WriteOutputPin(GPIOA, GPIO_PIN5, 1);
			delay();
		}
		*/
		if(buttonPress(GPIOC, GPIO_PIN13) == 1)
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
			//GPIO_WriteOutputPin(GPIOA, GPIO_PIN5, 1);
			delay();
		}
	}

	return 0;
}
