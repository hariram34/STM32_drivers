/*
 * stm32f446re_SPI_driver.h
 *
 *  Created on: May 17, 2020
 *      Author: Hariram Asokan
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_


#include "stm32f446xx.h"
#include <stdint.h>

/*Configuration structure for SPI*/
typedef struct
{
	uint8_t SPI_DeviceMode; 	//Device mode
	uint8_t SPI_BusConfig; 		//Bus configuration mode
	uint8_t SPI_SCLKSpeed; 		//serial clock speed
	uint8_t SPI_DFF;			//Data frame format
	uint8_t SPI_CPOL;			//clock polarity
	uint8_t SPI_CPHA;			//clock phase
	uint8_t SPI_SSM;			//slave select management
}SPI_Config_t;

/* Handle structure for SPIx peripheral */
typedef struct
{
	SPI_Reg *pSPIx;   /* base address of SPIx(x:0,1,2) peripheral */
	SPI_Config_t 	SPIConfig;

}SPI_Handle_t;

/*
 * Device mode options for SPI_DEVICEMODE
 */
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE  			0
/*
 * SPI Bus configuration for SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD_MODE						1
#define SPI_BUS_CONFIG_HD_MODE						2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY				3

/*
 * SCLK speed for SPI_SCLKSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * Data Frame Format options for SPI_DFF
 */
#define SPI_DFF_8bits					0
#define SPI_DFF_16bits					1
/*
 * clock polarity options for SPI_CPOL
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1
/*
 * clock phase option for SPI_CPHA
 */
#define SPI_CPHA_FIRST					0
#define SPI_CPHA_SECOND					1
/*
 * Slave Select management options for SPI_SSM
 */
#define SPI_SSM_SW						1
#define SPI_SSM_HW						0


/*******************************************************************
 * 				APIs for SPI driver
 *
 * 		API information in function definitions
 ********************************************************************/
/*
 * SPI init,deinit
 */
void SPI_init(SPI_Handle_t *pSPIHandle);
void SPI_deinit(SPI_Reg *pSPIx);

/*
 * SPI peripheral clock setup
 */
void SPI_PeriClockControl(SPI_Reg *pSPIx,uint8_t EnorDi);

/*
 * SPI send and recieve
 */
void SPI_SendData(SPI_Reg *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_Reg *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ config and handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void SPI_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(uint8_t PinNumber);

/*
 * utility functions
 */
int getFlagStatus(SPI_Reg *pSPIx, int flagbitpos);
void SPI_PeripheralControl(SPI_Reg *pSPIx, uint8_t EnOrDi);
void SPI_SSIControl(SPI_Reg *pSPIx, uint8_t EnOrDi);
void SPI_SSOEControl(SPI_Reg *pSPIx, uint8_t EnOrDi);


#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
