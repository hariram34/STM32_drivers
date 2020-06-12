/*
 * stm32f446re_gpio_driver.h
 *
 *  Created on: Mar 15, 2020
 *      Author: Hariram Asokan
 */

#ifndef INC_STM32F446RE_GPIO_DRIVER_H_
#define INC_STM32F446RE_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>
/*GPIO Configuration Structure*/
typedef struct
{
	uint8_t GPIO_PinNumber;			/*Possible values from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;			/*Possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;			/*Possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPUPDcontrol;	/*Possible values from @GPIO_PUPD*/
	uint8_t GPIO_PinOPType;			/*Possible values from @GPIO_OPTYPE*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*GPIO Handle structure*/
typedef struct
{
	GPIO_Reg 	*pGPIOx;				//holds the base address of a GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//Configuration settings for a GPIO pin
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 *
 */
#define GPIO_PIN0					0
#define GPIO_PIN1					1
#define GPIO_PIN2					2
#define GPIO_PIN3					3
#define GPIO_PIN4					4
#define GPIO_PIN5					5
#define GPIO_PIN6					6
#define GPIO_PIN7					7
#define GPIO_PIN8					8
#define GPIO_PIN9					9
#define GPIO_PIN10					10
#define GPIO_PIN11					11
#define GPIO_PIN12					12
#define GPIO_PIN13					13
#define GPIO_PIN14					14
#define GPIO_PIN15					15


/*
 * @GPIO_PIN_MODES
 * GPIO Pin Modes
 */
#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALTFN 			2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_FT 			4
#define GPIO_MODE_IT_RT 			5
#define GPIO_MODE_IT_RFT			6

/*
 * @GPIO_OTYPE
 * GPIO Output Type
 */
#define GPIO_OTYPE_PP				0
#define GPIO_OTYPE_OD				1

/*
 * @GPIO_PIN_SPEED
 * GPIO Output Speed Modes
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM 		  	1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

/*
 * @GPIO_PUPD
 * GPIO Pull Up/Down
 */
#define GPIO_NO_PUPD				0
#define GPIO_PU 		  			1
#define GPIO_PD						2

/*******************************************************************
 * 				APIs for GPIO driver
 *
 * 		API information in function definitions
 ********************************************************************/

/*
 * GPIO init,deinit
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_deinit(GPIO_Reg *pGPIOx);

/*
 * GPIO peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_Reg *pGPIOx,uint8_t EnorDi);

/*
 * GPIO read,write
 */
uint8_t GPIO_ReadInputPin(GPIO_Reg *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_Reg *pGPIOx);
void GPIO_WriteOutputPin(GPIO_Reg *pGPIOx,uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_Reg *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Reg *pGPIOx,uint8_t PinNumber);

/*
 * GPIO Interrupt handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
