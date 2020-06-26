/*
 * SPI_FullDuplex.c
 *
 *  Created on: Jun 17, 2020
 *      Author: hariram asokan
 */


/*
 * SPI_SendData.c
 *
 *  Created on: May 23, 2020
 *      Author: Hariram Asokan
 *
 *		This program is used to test basic SPI send
 *		data API.
 *		No slave is used.
 *
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>
extern int initialise_monitor_handles();

//command controls
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

//Analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  13

void delay(){
	for(int i =0;i<600000;i++);
}
/*
 * Alternate function mode of GPIO is used for SPI.
 * SPI2 has been used and below is the following pinout mapping for SPI2 in AF mode
 *
 * SPI2_SCK  ->  PB10 --- AF5	PB3
 * SPI2_NSS  ->  PB9  --- AF5
 * SPI2_MISO ->  PB14 --- AF5
 * SPI2_MOSI ->  PB15 --- AF5	PB5
 *
 */
void SPI_GPIO_PinSetup()
{
	 GPIO_Handle_t spi;
	 spi.pGPIOx = GPIOB;										//port B address
	 spi.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;		//GPIO mode
	 spi.GPIO_PinConfig.GPIO_PinAltFunMode = 5;				//Alternate function mode
	 spi.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP; 		//push pull OP type
	 spi.GPIO_PinConfig.GPIO_PinPUPDcontrol = GPIO_PU;			 //PU
	 spi.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;		//GPIO speed

	 //SCK
	 spi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN10;
	 GPIO_init(&spi);

	 //NSS
	 spi.GPIO_PinConfig.GPIO_PinNumber = 9;
	 GPIO_init(&spi);

	 //MISO
	 spi.GPIO_PinConfig.GPIO_PinNumber = 14;
	 GPIO_init(&spi);

	 //MOSI
	 spi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN15;
	 GPIO_init(&spi);



}

void SPI_Setup()
{
	SPI_Handle_t SPI;

	SPI.pSPIx = SPI2;
	SPI.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;
	SPI.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI.SPIConfig.SPI_DFF = SPI_DFF_8bits;
	SPI.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV64;
	SPI.SPIConfig.SPI_SSM = SPI_SSM_HW;
	SPI.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD_MODE;


	SPI_init(&SPI);
}

void GPIO_buttonSetup()
{
	GPIO_Handle_t button;
	button.pGPIOx = GPIOC;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinPUPDcontrol = GPIO_PU;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_init(&button);

}

void GPIO_ledsetup()
{
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
	GPIO_init(&led);
}
int assertOK(uint8_t ack)
{
	if(ack == (uint8_t)0xF5)
	{
		return 1;
	}
	return 0;
}
int main()
{

	//initialise_monitor_handles();

	printf("Begin test\n");
	uint8_t dummyread;
	uint8_t dummywrite = 0xFF;
	uint8_t ACK;
	uint8_t cmdcode;
	uint8_t args[2];
	GPIO_buttonSetup();
	GPIO_ledsetup();
	//setup GPIO pins for SPI communication
	SPI_GPIO_PinSetup();

	//initialize SPI
	SPI_Setup();

	SPI_SSOEControl(SPI2, ENABLE);
	//SPI_SSIControl(SPI2, ENABLE);

	printf("Initialized\n");
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
	while(1){

	while(GPIO_ReadInputPin(GPIOC, GPIO_PIN13));
	delay();

	SPI_PeripheralControl(SPI2, ENABLE);



	cmdcode = COMMAND_LED_CTRL;

	SPI_SendData(SPI2, &cmdcode, 1);
	delay();

	SPI_RecieveData(SPI2, &dummyread,1);

	SPI_SendData(SPI2, &dummywrite, 1);

	SPI_RecieveData(SPI2, &ACK, 1);

	if(assertOK(ACK))
	{
		args[0] = LED_PIN;
		args[1] = SET;
		SPI_SendData(SPI2, args, 2);
	}

	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);

	delay();



	}

return 0;
}
