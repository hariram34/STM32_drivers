/*
 * stm32f446re_gpio_driver.c
 *
 *  Created on: Mar 15, 2020
 *      Author: Hariram Asokan
 */

#include "stm32f446re_gpio_driver.h"

/*
 * @fn			- GPIO_init
 *
 * @brief		- Initialize GPIO
 *
 * @param1		- GPIO Handle structure that contains registers to initialize
 *
 * @return		- none
 *
 * @Note		- Initialize the GPIO registers with the configuration given by the user
 * 				  through the GPIO handle.
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//enable clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);
	//configure GPIO pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		//Interrupts

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT){
			//Set the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//disable the RTSR by resetting it
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT){
			//Set the RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//disable the FTSR by resetting it
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT){
			//Set the RTSR and FTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);
		//enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp=0;

	//configure speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;
	//configure pupd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDcontrol << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;
	//configure otype
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;
	//configure alternative functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//pGPIOHandle->pGPIOx->AFR[] |= temp;
		uint8_t temp1=0,temp2=0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;

	}

}

/*
 * @fn			- GPIO_deinit
 *
 * @brief		- de-initialize GPIO
 *
 * @param1		- GPIO Handle structure that contains registers to de-initialize
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_deinit(GPIO_Reg *pGPIOx)
{
      if(pGPIOx == GPIOA)
	  {
		  GPIOA_REG_RESET();
	  }else if(pGPIOx == GPIOB)
	  {
		  GPIOB_REG_RESET();
	  }else if(pGPIOx == GPIOC)
	  {
		  GPIOC_REG_RESET();
	  }else if(pGPIOx == GPIOD)
	  {
		  GPIOD_REG_RESET();
	  }else if(pGPIOx == GPIOE)
	  {
		  GPIOE_REG_RESET();
	  }else if(pGPIOx == GPIOF)
	  {
		  GPIOF_REG_RESET();
	  }else if(pGPIOx == GPIOG)
	  {
		  GPIOG_REG_RESET();
	  }else if(pGPIOx == GPIOH)
	  {
		  GPIOH_REG_RESET();
	  }else if(pGPIOx == GPIOI)
	  {
		  GPIOI_REG_RESET();
	  }
}

/*
 * GPIO peripheral clock setup
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 *
 * @param1		- Base address of GPIO peripheral
 * @param2		- Enable or Disable macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_Reg *pGPIOx,uint8_t EnorDi)
{
  if(EnorDi == ENABLE)
  {
	  if(pGPIOx == GPIOA)
	  {
		  GPIOA_CLK_EN();
	  }else if(pGPIOx == GPIOB)
	  {
		  GPIOB_CLK_EN();
	  }else if(pGPIOx == GPIOC)
	  {
		  GPIOC_CLK_EN();
	  }else if(pGPIOx == GPIOD)
	  {
		  GPIOD_CLK_EN();
	  }else if(pGPIOx == GPIOE)
	  {
		  GPIOE_CLK_EN();
	  }else if(pGPIOx == GPIOF)
	  {
		  GPIOF_CLK_EN();
	  }else if(pGPIOx == GPIOG)
	  {
		  GPIOG_CLK_EN();
	  }else if(pGPIOx == GPIOH)
	  {
		  GPIOH_CLK_EN();
	  }else if(pGPIOx == GPIOI)
	  {
		  GPIOI_CLK_EN();
	  }
  }else
  {
	  if(pGPIOx == GPIOA)
	 	  {
	 		  GPIOA_CLK_DIS();
	 	  }else if(pGPIOx == GPIOB)
	 	  {
	 		  GPIOB_CLK_DIS();
	 	  }else if(pGPIOx == GPIOC)
	 	  {
	 		  GPIOC_CLK_DIS();
	 	  }else if(pGPIOx == GPIOD)
	 	  {
	 		  GPIOD_CLK_DIS();
	 	  }else if(pGPIOx == GPIOE)
	 	  {
	 		  GPIOE_CLK_DIS();
	 	  }else if(pGPIOx == GPIOF)
	 	  {
	 		  GPIOF_CLK_DIS();
	 	  }else if(pGPIOx == GPIOG)
	 	  {
	 		  GPIOG_CLK_DIS();
	 	  }else if(pGPIOx == GPIOH)
	 	  {
	 		  GPIOH_CLK_DIS();
	 	  }else if(pGPIOx == GPIOI)
	 	  {
	 		  GPIOI_CLK_DIS();
	 	  }
  }
}

/*
 * GPIO read,write
 */

/*
 * GPIO Read from an input pin
 * @fn			- GPIO_ReadInputPin
 *
 * @brief		- This function reads from an input pin
 *
 *
 * @param1		- Base address of GPIO peripheral
 * @param2		- Pin number to read from
 *
 * @return		- read value is returned
 *
 * @Note		- Read input pin value from the Input Data Register
 */
uint8_t GPIO_ReadInputPin(GPIO_Reg *pGPIOx,uint8_t PinNumber)
{
	uint8_t data = ((pGPIOx->IDR)>>PinNumber) & 0x00000001;
	return data;
}

/*
 * GPIO Read an entire input port
 * @fn			- GPIO_ReadInputPort
 *
 * @brief		- This function reads from an input pin
 *
 * @param1		- Base address of GPIO peripheral
 *
 * @return		- read value is returned
 *
 * @Note		- return the entire port value
 */
uint16_t GPIO_ReadInputPort(GPIO_Reg *pGPIOx)
{
	uint16_t port_value = pGPIOx->IDR;
	return port_value;
}

/*
 * GPIO write to a output pin
 * @fn			- GPIO_ReadOutputPin
 *
 * @brief		- This function writes to an output pin
 *
 * @param1		- Base address of GPIO peripheral
 * @param2		- Pin number to write
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteOutputPin(GPIO_Reg *pGPIOx,uint8_t PinNumber, uint8_t Value)
{
	if(Value == 1)
	{
		pGPIOx->ODR |= 1<<PinNumber;
	}else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
/*
 * GPIO write to an entire output port
 * @fn			- GPIO_ReadOutputPort
 *
 * @brief		- This function writes to an output port
 *
 * @param1		- Base address of GPIO peripheral
 * @param2		- Value to be written
 *
 * @return		- none
 *
 * @Note		- write the value into the Output Data Register
 */
void GPIO_WriteOutputPort(GPIO_Reg *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_Reg *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= 1<<PinNumber;
}

/*
 * GPIO Interrupt handling
 *  @fn			- GPIO_IRQConfig
 *
 * @brief		- This functiom enables or disables an interrupt  and sets the priority
 *
 * @param1		- IRQNumber
 * @param2		- Priority of the interrupt
 * @param3 		- Enable or Disable the interrupt
 *
 * @return		- none
 *
 * @Note		- Set NVIC_ISER or Reset NVIC_ICER register and set the priority for the interrupt
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		//Enable interrupt for that IRQNumber from 0 to 96
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));

		}
	}else
	{
		//Disable interrupt for that IRQNumber
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber%64));

		}
	}
}
/*
 * GPIO Interrupt handling
 *  @fn			- GPIO_PriorityConfig
 *
 * @brief		- This function sets the priority for an interrupt
 *
 * @param1		- IRQNumber
 * @param2		- Priority of the interrupt
 *
 * @return		- none
 *
 * @Note		- IPR(there's a total of 60 IPR) is a 32 bit register divided into four slots, each slot for an interrupt.
 * 				  Lower 4 bits for a priority slot is not implemented hence shift IRQPriority by 4;
 */
void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint32_t iprn = IRQNumber/4;
	uint8_t slot = IRQNumber%4;
	*(NVIC_IPR +(iprn)) |= ((IRQPriority<<4)<<(slot*8));
}
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the exti pr register corresponding to the pin number
		if(EXTI->PR & ( 1 << PinNumber))
		{
			//clear
			EXTI->PR |= ( 1 << PinNumber);
		}
}

