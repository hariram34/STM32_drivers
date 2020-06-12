/*
 * stm32f446xx.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Hariram Asokan
 */
#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include "stm32f446xx.h"
#include <stdint.h>

/*****ARM cortex M4 NVIC interrupt register addresses*******/

//Interrupt Set Enable Registers
#define NVIC_ISER0			((volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1			((volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2			((volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3			((volatile uint32_t*)0xE000E10C )
#define NVIC_ISER4			((volatile uint32_t*)0xE000E110 )
#define NVIC_ISER5			((volatile uint32_t*)0xE000E114 )
#define NVIC_ISER6			((volatile uint32_t*)0xE000E118 )
#define NVIC_ISER7			((volatile uint32_t*)0xE000E11C )
//Interrupt Clear Enable Registers
#define NVIC_ICER0			((volatile uint32_t*)0xE000E180 )
#define NVIC_ICER1			((volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2			((volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3			((volatile uint32_t*)0xE000E18C )
#define NVIC_ICER4			((volatile uint32_t*)0xE000E180 )
#define NVIC_ICER5			((volatile uint32_t*)0xE000E184 )
#define NVIC_ICER6			((volatile uint32_t*)0xE000E188 )
#define NVIC_ICER7			((volatile uint32_t*)0xE000E18C )
//Interrupt Priority Register Base address
#define NVIC_IPR					((volatile uint32_t*)0xE000E400 )

//Base address for SRAM and flash memories
#define FLASH_BASEADDR		 0x08000000U
#define SRAM1_BASEADDR		 0x20000000U
#define SRAM2_BASEADDR		 0x2001C000U
//AHB and APB bus base addresses
#define PERIPH_BASE 		 0x40000000U
#define APB1_BASEADDR  		 PERIPH_BASE
#define APB2_BASEADDR 		 0x40010000U
#define AHB1_BASEADDR		 0x40020000U
#define AHB2_BASEADDR        0x50000000U

/*AHB1 peripherals base addresses*/
//GPIOs
#define GPIOA_BASEADDR		 (AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		 (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		 (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		 (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		 (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		 (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		 (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		 (AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		 (AHB1_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR		 (AHB1_BASEADDR + 0x2400)
#define GPIOK_BASEADDR		 (AHB1_BASEADDR + 0x2800)
#define RCC_BASEADDR		 (AHB1_BASEADDR + 0x3800)
/*APB1 peripherals base addresses*/

#define WWDG_BASEADDR		 (APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR		 (APB1_BASEADDR + 0x3000)
//SPI and I2S base addresses
#define I2S2ext_BASEADDR	 (APB1_BASEADDR + 0x3400)
#define SPI2_BASEADDR		 (APB1_BASEADDR + 0x3800)    /*also supports I2S2*/
#define SPI3_BASEADDR		 (APB1_BASEADDR + 0x3C00)	 /*also supports I2S3*/
#define I2S3ext_BASEADDR	 (APB1_BASEADDR + 0x4000)
//USART and UART base addresses
#define USART2_BASEADDR	     (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR		 (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR		 (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR	     (APB1_BASEADDR + 0x5000)
//I2C base addresses
#define I2C1_BASEADDR		 (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR		 (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR		 (APB1_BASEADDR + 0x5C00)
//CAN base addresses
#define CAN1_BASEADDR		 (APB1_BASEADDR + 0x6400)
#define CAN2_BASEADDR		 (APB1_BASEADDR + 0x6800)
//DAC,PWR excluded here
//UART base addresses
#define UART7_BASEADDR		 (APB1_BASEADDR + 0x7800)
#define UART8_BASEADDR		 (APB1_BASEADDR + 0x7C00)

/*APB2 peripherals base addresses*/
#define EXTI_BASEADDR		 (APB2_BASEADDR + 0x3C00)
#define SPI1_BASEADDR		 (APB2_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR		 (APB2_BASEADDR + 0x3800)
#define USART1_BASEADDR		 (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		 (APB2_BASEADDR + 0x1400)

/*********************************************************************
 * Defining a general structure to hold GPIO registers
 *
 * The structure GPIO_Reg will hold the relevant registers for a GPIO
 *
 * ********************************************************************/
typedef struct
{
	volatile uint32_t MODER;		/*GPIO port mode register 					Address offset = 0x00*/
	volatile uint32_t OTYPER;		/*GPIO port output type register 			Address offset = 0x04*/
	volatile uint32_t OSPEEDR;		/*GPIO port output speed register			Address offset = 0x08*/
	volatile uint32_t PUPDR;		/*GPIO port pull-up/pull-down register		Address offset = 0x0C*/
	volatile uint32_t IDR;			/*GPIO port input data register				Address offset = 0x10*/
	volatile uint32_t ODR;			/*GPIO port output data register			Address offset = 0x14*/
	volatile uint32_t BSRR;			/*GPIO port bit set/reset register			Address offset = 0x18*/
	volatile uint32_t LCKR;			/*GPIO port configuration lock register		Address offset = 0x1C*/
	volatile uint32_t AFR[2]; 		/*GPIO alternate function low register		Address offset = 0x20(Low),0x24(High)*/
}GPIO_Reg;

/**************************************************************************
 * structure to hold RCC clock registers
 *
 *  The structure RCC will hold the relevant registers for RCC clock
 *
 ***************************************************************************/

typedef struct
{
	volatile uint32_t CR; 						/*Clock Control Register										Address offset = 0x00*/
	volatile uint32_t PLLCFGR; 					/*PLL configuration register									Address offset = 0x04*/
	volatile uint32_t CFGR; 					/*clock configuration register									Address offset = 0x08*/
	volatile uint32_t CIR; 						/*clock interrupt register										Address offset = 0x0C*/
	volatile uint32_t AHB1RSTR; 				/*AHB1 peripheral reset register								Address offset = 0x10*/
	volatile uint32_t AHB2RSTR; 				/*AHB2 peripheral reset register								Address offset = 0x14*/
	volatile uint32_t AHB3RSTR; 				/*AHB3 peripheral reset register								Address offset = 0x18*/
	volatile uint32_t reserved1;				/*Reserved														Address offset = 0x1C*/
	volatile uint32_t APB1RSTR; 				/*APB1 peripheral reset register								Address offset = 0x20*/
	volatile uint32_t APB2RSTR; 				/*APB2 peripheral reset register								Address offset = 0x24*/
	volatile uint32_t reserved2;				/*APB1 peripheral reset register								Address offset = 0x28*/
	volatile uint32_t reserved3;				/*APB2 peripheral reset register								Address offset = 0x2C*/
	volatile uint32_t AHB1ENR; 					/*AHB1 peripheral clock enable register							Address offset = 0x30*/
	volatile uint32_t AHB2ENR; 					/*AHB2 peripheral clock enable register							Address offset = 0x34*/
	volatile uint32_t AHB3ENR; 					/*AHB3 peripheral clock enable register							Address offset = 0x38*/
	volatile uint32_t reserved4;				/*APB1 peripheral reset register								Address offset = 0x3C*/
	volatile uint32_t APB1ENR; 					/*APB1 peripheral clock enable register						    Address offset = 0x40*/
	volatile uint32_t APB2ENR; 					/*APB2 peripheral clock enable register							Address offset = 0x44*/
	volatile uint32_t reserved5;				/*APB1 peripheral reset register								Address offset = 0x48*/
	volatile uint32_t reserved6;				/*APB2 peripheral reset register								Address offset = 0x4C*/
	volatile uint32_t AHB1LPENR; 				/*AHB1 peripheral clock enabled in low power mode register		Address offset = 0x50*/
	volatile uint32_t AHB2LPENR; 				/*AHB2 peripheral clock enabled in low power mode register		Address offset = 0x54*/
	volatile uint32_t AHB3LPENR; 				/*AHB3 peripheral clock enabled in low power mode register		Address offset = 0x58*/
	volatile uint32_t reserved7;				/*APB2 peripheral reset register								Address offset = 0x5C*/
	volatile uint32_t APB1LPENR; 				/*APB1 peripheral clock enabled in low power mode register		Address offset = 0x60*/
	volatile uint32_t APB2LPENR; 				/*APB2 peripheral clock enabled in low power mode register		Address offset = 0x64*/
	volatile uint32_t reserved8;				/*APB1 peripheral reset register								Address offset = 0x68*/
	volatile uint32_t reserved9;				/*APB2 peripheral reset register								Address offset = 0x6C*/
	volatile uint32_t BDCR;						/*Backup domain control register								Address offset = 0x70*/
	volatile uint32_t CSR;						/*clock control & status register								Address offset = 0x74*/
	volatile uint32_t reserved10;				/*APB1 peripheral reset register								Address offset = 0x78*/
	volatile uint32_t reserved11;				/*APB2 peripheral reset register								Address offset = 0x7C*/
	volatile uint32_t SSCGR;					/*spread spectrum clock generation register						Address offset = 0x80*/
	volatile uint32_t PLLI2SCFGR;				/*PLLI2S configuration register 								Address offset = 0x84*/
	volatile uint32_t PLLSAICFGR;				/*PLL configuration register									Address offset = 0x88*/
	volatile uint32_t DCKCFGR;					/*Dedicated Clock Configuration Register 						Address offset = 0x8C*/
	volatile uint32_t CKGATENR;					/*RCC clocks gated enable register		 						Address offset = 0x90*/
	volatile uint32_t DCKCFGR2;					/*Dedicated Clock Configuration Register 2 						Address offset = 0x94*/
}RCC;

/**************************************************************************
 * Structure to hold EXTI(External Interrupts) registers
 *
 *  The structure EXTI_Reg will hold the relevant registers for EXTI
 *
 ***************************************************************************/
typedef struct
{
	volatile uint32_t IMR;						/*Interrupt Mask Register 										Address offset = 0x00*/
	volatile uint32_t EMR;						/*Event Mask Register 										    Address offset = 0x04*/
	volatile uint32_t RTSR;						/*Rising Trigger Selection Register 							Address offset = 0x08*/
	volatile uint32_t FTSR;						/*Falling Trigger Selection Register 							Address offset = 0x0C*/
	volatile uint32_t SWIER;					/*Software Interrupt Event Register 							Address offset = 0x10*/
	volatile uint32_t PR;						/*Pending Register					 							Address offset = 0x14*/

}EXTI_Reg;

/**************************************************************************
 * Structure to hold System configuration controller(SYSCFG) registers
 *
 *  The structure SYSCFG_Reg will hold the relevant registers for SYSCFG
 *
 ***************************************************************************/
typedef struct
{
	volatile uint32_t MEMRMP;					/*Memory remap register 										Address offset = 0x00*/
	volatile uint32_t PMC;						/*Peripheral mode configuration register 					    Address offset = 0x04*/
	volatile uint32_t EXTICR[4];				/*External interrupt configuration register 1,2,3,4 			Address offset = 0x08,0x0C,0x010,0x14*/
	uint32_t reserved[2];						/*reserved														Address offset = 0x18,0x1C*/
	volatile uint32_t CMPCR;					/*Compensation cell control register 							Address offset = 0x20*/
	uint32_t reserved_[2];						/*reserved														Address offset = 0x24,0x28*/
	volatile uint32_t CFGR;						/*Configuration register			 							Address offset = 0x20*/


}SYSCFG_Reg;

/**************************************************************************
 * Structure to hold SPI registers
 *
 *  The structure SPI_Reg will hold the relevant registers for SYSCFG
 *
 ***************************************************************************/
typedef struct
{
	volatile uint32_t  CR1;					/*SPI control register 1											Address offset = 0x00*/
	volatile uint32_t  CR2;					/*SPI control register 2											Address offset = 0x04*/
	volatile uint32_t  SR;					/*SPI status register												Address offset = 0x08*/
	volatile uint32_t  DR;					/*SPI data register													Address offset = 0x0C*/
	volatile uint32_t  CRCPR;				/*SPI CRC polynomial register										Address offset = 0x10*/
	volatile uint32_t  RXCRCR;				/*SPI RX CRC register												Address offset = 0x14*/
	volatile uint32_t  TXCRCR;				/*SPI TX CRC register												Address offset = 0x18*/
	volatile uint32_t  I2SCFGR;				/*SPI_I2S configuration register									Address offset = 0x1C*/
	volatile uint32_t  I2SPR;				/*SPI_I2S prescaler register										Address offset = 0x20*/

}SPI_Reg;

/**************************************************************************
 * Making programming easier by defining macros below
 *
 * The macro will represent typecasted GPIOX for the GPIO_Reg
 * This allows us to assign GPIOA to a pointer without typecasting manually
 * *************************************************************************/
#define GPIOA 	((GPIO_Reg*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_Reg*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_Reg*)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_Reg*)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_Reg*)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_Reg*)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_Reg*)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_Reg*)GPIOH_BASEADDR)
#define GPIOI 	((GPIO_Reg*)GPIOI_BASEADDR)
#define GPIOJ 	((GPIO_Reg*)GPIOJ_BASEADDR)
#define GPIOK 	((GPIO_Reg*)GPIOJ_BASEADDR)

/*******************EXTI macro*********************/
#define EXTI 	((EXTI_Reg*)EXTI_BASEADDR)

/*******************SYSCFG macro*********************/
#define SYSCFG 	((SYSCFG_Reg*)SYSCFG_BASEADDR)

/*******************SPI macro*********************/
#define SPI1 	((SPI_Reg*)SPI1_BASEADDR)
#define SPI2 	((SPI_Reg*)SPI2_BASEADDR)
#define SPI3 	((SPI_Reg*)SPI3_BASEADDR)

/*---------------CLOCK ENABLE MACROS-----------*/
#define RCC 	((RCC*)RCC_BASEADDR)
/*Clock Enable Macros for GPIOx*/
#define GPIOA_CLK_EN()		(RCC->AHB1ENR |= 1<<0)
#define GPIOB_CLK_EN()		(RCC->AHB1ENR |= 1<<1)
#define GPIOC_CLK_EN()		(RCC->AHB1ENR |= 1<<2)
#define GPIOD_CLK_EN()		(RCC->AHB1ENR |= 1<<3)
#define GPIOE_CLK_EN()		(RCC->AHB1ENR |= 1<<4)
#define GPIOF_CLK_EN()		(RCC->AHB1ENR |= 1<<5)
#define GPIOG_CLK_EN()		(RCC->AHB1ENR |= 1<<6)
#define GPIOH_CLK_EN()		(RCC->AHB1ENR |= 1<<7)
#define GPIOI_CLK_EN()		(RCC->AHB1ENR |= 1<<8)
//#define GPIOJ_CLK_EN()		(RCC->AHB1ENR |= 1<<9)
//#define GPIOK_CLK_EN()		(RCC->AHB1ENR |= 1<<10)

/*Clock Enable Macros for I2Cx peripherals*/
#define I2C1_CLK_EN()		(RCC->APB1ENR |= 1<<21)
#define I2C2_CLK_EN()		(RCC->APB1ENR |= 1<<22)
#define I2C3_CLK_EN()		(RCC->APB1ENR |= 1<<23)

/*Clock Enable Macros for SPIx peripherals*/
#define SPI1_CLK_EN()		(RCC->APB2ENR |= 1<<12)
#define SPI2_CLK_EN()		(RCC->APB1ENR |= 1<<14)
#define SPI3_CLK_EN()		(RCC->APB1ENR |= 1<<15)
#define SPI4_CLK_EN()		(RCC->APB2ENR |= 1<<13)
#define SPI5_CLK_EN()		(RCC->APB2ENR |= 1<<20)
#define SPI6_CLK_EN()		(RCC->APB2ENR |= 1<<21)


/*Clock Enable Macros for USARTx and UARTx peripherals*/
#define USART1_CLK_EN()		(RCC->APB2ENR |= 1<<4)
#define USART2_CLK_EN()		(RCC->APB1ENR |= 1<<17)
#define USART3_CLK_EN()		(RCC->APB1ENR |= 1<<18)
#define UART4_CLK_EN()		(RCC->APB1ENR |= 1<<19)
#define UART5_CLK_EN()		(RCC->APB1ENR |= 1<<20)
#define USART6_CLK_EN()		(RCC->APB2ENR |= 1<<5)
#define UART7_CLK_EN()		(RCC->APB1ENR |= 1<<30)
#define UART8_CLK_EN()		(RCC->APB1ENR |= 1<<31)

/*Clock Enable macros for SYSCFG peripheral*/
#define SYSCFG_CLK_EN()		(RCC->APB2ENR |= 1<<14)

/*---------------CLOCK DISABLE MACROS-----------*/
/*Clock disable Macros for GPIOx*/
#define GPIOA_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_CLK_DIS()		(RCC->AHB1ENR &= ~(1<<8))
//#define GPIOJ_CLK_EN()		(RCC->AHB1ENR |= 1<<9)
//#define GPIOK_CLK_EN()		(RCC->AHB1ENR |= 1<<10)

/*Clock disable Macros for I2Cx peripherals*/
#define I2C1_CLK_DIS()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_CLK_DIS()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_CLK_DIS()		(RCC->APB1ENR &= ~(1<<23))

/*Clock disable Macros for SPIx peripherals*/
#define SPI1_CLK_DIS()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DIS()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DIS()		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_CLK_DIS()		(RCC->APB2ENR &= ~(1<<13))
#define SPI5_CLK_DIS()		(RCC->APB2ENR &= ~(1<<20))
#define SPI6_CLK_DIS()		(RCC->APB2ENR &= ~(1<<21))


/*Clock disable Macros for USARTx and UARTx peripherals*/
#define USART1_CLK_DIS()		(RCC->APB2ENR &= ~(1<<4))
#define USART2_CLK_DIS()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_CLK_DIS()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_CLK_DIS()			(RCC->APB1ENR &= ~(1<<19))
#define UART5_CLK_DIS()			(RCC->APB1ENR &= ~(1<<20))
#define USART6_CLK_DIS()		(RCC->APB2ENR &= ~(1<<5))
#define UART7_CLK_DIS()			(RCC->APB1ENR &= ~(1<<30))
#define UART8_CLK_DIS()			(RCC->APB1ENR &= ~(1<<31))

/*Clock disable macros for SYSCFG peripheral*/
#define SYSCFG_CLK_DIS()		(RCC->APB2ENR &= ~(1<<14))

/*GPIO Register reset macros*/
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<0);		(RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<1);		(RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<2);		(RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<3);		(RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<4);		(RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<5);		(RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<6);		(RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<7);		(RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= 1<<8);		(RCC->AHB1RSTR &= ~(1<<8)); }while(0)

/*Port code translation macro*/
#define GPIO_BASEADDR_TO_CODE(x) ((x==GPIOA) ? 0 :\
								 (x==GPIOB) ? 1 :\
								 (x==GPIOC) ? 2 :\
								 (x==GPIOD) ? 3 :\
								 (x==GPIOE) ? 4 :\
								 (x==GPIOF) ? 5 :\
								 (x==GPIOG) ? 6 :\
								 (x==GPIOH) ? 7 :\
								 (x==GPIOI) ? 8 : 0)

/*SPI reset macro*/

#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= 1<<12);		(RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= 1<<14);		(RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= 1<<15);		(RCC->APB1RSTR &= ~(1<<15)); }while(0)

/*SPI bit position values*/

//SPI Control Register 1

#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSB_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

//SPI Control Register 2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXNEIE			7

//SPI Status Register
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRC_ERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/*IRQ number for some EXTI lines*/
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI5_9			23
#define IRQ_NO_EXTI15_10		40
/*some useful macros*/
#define ENABLE 1
#define DISABLE 0
#define SET 1
#define RESET 0
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_SPI_driver.h"
#endif /* INC_STM32F446XX_H_ */
