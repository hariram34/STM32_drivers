
#include "stm32f446re_SPI_driver.h"


/*
 * SPI peripheral clock setup
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given SPI
 *
 *
 * @param1		- Base address of SPI peripheral
 * @param2		- Enable or Disable macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_Reg *pSPIx,uint8_t EnorDi)
{
  if(EnorDi == ENABLE)
  {
	  if(pSPIx == SPI1)
	  {
		  SPI1_CLK_EN();
	  }else if(pSPIx == SPI2)
	  {
		  SPI2_CLK_EN();
	  }else if(pSPIx == SPI3)
	  {
			  SPI3_CLK_EN();
	  }

  }else
  {
	  if(pSPIx == SPI1)
	  {
	 	 SPI1_CLK_DIS();
	  }else if(pSPIx == SPI2)
	  {
	 	 SPI2_CLK_DIS();
	  }else if(pSPIx == SPI3)
	  {
	 	 SPI3_CLK_DIS();
	  }
}

}

/*
 * SPI peripheral Initialization
 * @fn			- SPI_init
 *
 * @brief		- This function initializes the appropriate SPI registers
 *
 *
 * @param1		- pointer to SPI handle
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_init(SPI_Handle_t *pSPIHandle)
{

	//enable spi clock
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//set device mode as master or slave
	uint32_t tempreg = 0;

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//set bus config, full duplex or half duplex or simplex
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD_MODE)
	{
		tempreg &=  ~(1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD_MODE)
	{
		tempreg |=  (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//set 2 line unidirectional(FD) mode
		tempreg &=  ~(1 << SPI_CR1_BIDIMODE);
		//set RXONLY bit
		tempreg |= 1 << SPI_CR1_RXONLY;

	}

	//set baudrate
	tempreg |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;

	//set Data frame format

	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//set clock polarity

	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//set clock phase

	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//set slave select management

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//set SSI bit, NSS will be pulled high internally
	//keeping the SSI bit in reset state will disable SPI peripheral and reset MSTR
	//tempreg |= SET << SPI_CR1_SSI;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*
 * SPI peripheral deinitialization
 * @fn			- SPI_deinit
 *
 * @brief		- This function deinitializes the SPI registers using
 * 				  appropriate RCC APB reset registers
 *
 *
 * @param1		- pointer to SPI base address
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_deinit(SPI_Reg *pSPIx)
{
if(pSPIx == SPI1)
{

	SPI1_REG_RESET();

}else if(pSPIx == SPI2)
{
	SPI2_REG_RESET();

}else if(pSPIx == SPI3)
{
	SPI3_REG_RESET();
}
}

int getFlagStatus(SPI_Reg *pSPIx, int flagbitpos)
{
	if(pSPIx->SR >> flagbitpos == SET)
	{
		return SET;
	}

	return RESET;
}
/*
 * SPI peripheral Send Data
 * @fn			- SPI_SendData
 *
 * @brief		- This function sends data to the outside world
 *
 *
 * @param1		- pointer to SPI base address
 * @param2		- pointer to the buffer to be transmitted
 * @param3		- length of the buffer in bytes
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_SendData(SPI_Reg *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

while(len > 0)
{
//wait until the TX buffer is empty
while(getFlagStatus(pSPIx, SPI_SR_TXE) == RESET);

if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
{
	//2bytes loaded into DR
	pSPIx->DR = *((uint16_t*)pTxBuffer);
	len = len-2;
	(uint16_t*)pTxBuffer++;
}else
{
	//1byte loaded into DR
	pSPIx->DR = *(pTxBuffer);
	len--;
	pTxBuffer++;
	char ch = pSPIx->DR;
}
}
}


/*
 * SPI peripheral receive Data
 * @fn			- SPI_ReceiveData
 *
 * @brief		- This function receives data from the outside world
 *
 *
 * @param1		- pointer to SPI base address
 * @param2		- pointer to the buffer to be transmitted
 * @param3		- length of the buffer in bytes
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_ReceiveData(SPI_Reg *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

while(len > 0)
{
//wait until the TX buffer is empty
while(getFlagStatus(pSPIx, SPI_SR_RXNE) == RESET);

if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
{
	//2bytes loaded into DR
	*((uint16_t*)pRxBuffer) = pSPIx->DR;
	len = len-2;
	(uint16_t*)pRxBuffer++;
}else
{
	//1byte loaded into DR
	*(pRxBuffer) = pSPIx->DR;
	len--;
	pRxBuffer++;
}
}
}

/*
 * SPI peripheral control
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- This function enables or disables the SPI peripheral
 *
 *
 * @param1		- pointer to SPI base address
 * @param2		- Enable or disable
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeripheralControl(SPI_Reg *pSPIx, uint8_t EnOrDi)
{

if(EnOrDi == ENABLE)
{
	pSPIx->CR1 |= (1<<SPI_CR1_SPE);
}else
{
	pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);

}
}
/*
 * SPI peripheral SSI control
 * @fn			- SPI_SSIControl
 *
 * @brief		- This function sets or resets the SSI bit
 *
 * @param1		- pointer to SPI base address
 * @param2		- Enable or disable
 *
 * @return		- none
 *
 * @Note		- SSI has to be set, if not MODF will occur which will reset MSTR and also disable the SPI peripheral
 * 				  when SSI is set, NSS is pulled high internally. This is necessary because in the multi master mode, when NSS is
 * 				  low indicates some other master has taken control.
 */
void SPI_SSIControl(SPI_Reg *pSPIx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);

	}
}

/*
 * SPI peripheral Send Data
 * @fn			- SPI_SSOEControl
 *
 * @brief		- This function sets or resets the SSOE bit
 *
 * @param1		- pointer to SPI base address
 * @param2		- Enable or disable
 *
 * @return		- none
 *
 * @Note		- SSOE has to be enabled for single master communication and disabled
 * 				  for multimaster communication.
 */
void SPI_SSOEControl(SPI_Reg *pSPIx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);

	}
}
