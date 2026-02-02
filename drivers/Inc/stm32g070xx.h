/*
 * stm32g070xx.h
 *
 *  Created on: 23-Jan-2026
 *      Author: Parth
 */

#ifndef INC_STM32G070XX_H_
#define INC_STM32G070XX_H_

#include <stdint.h>

#define __vol volatile

/* ************************************* Processor Specific Details ************************************* */

//ARM Cortex M0+ Processor

//NVIC ISER Register Addresses
#define NVIC_ISER				((__vol uint32_t*)0xE000E100)
#define NVIC_ICER				((__vol uint32_t*)0xE000E180)

//NVIC Priority Register Base Address
#define NVIC_PR_BASE			((__vol uint32_t*)0xE000E400)

//Priority Bits Implemented by ARM Cortex Mx Processor (M0+ = 2 Bits implemented)
#define PR_BITS_IMPLEMENTED		2

/* ************************************* Peripheral Specific Details ************************************* */

#define FLASH_BASE				0x08000000U			/* Base Address of FLASH Memory */
#define SRAM_BASE				0x20000000U			/* Base Address of SRAM */
#define ROM_BASE				0x1FFF0000U			/* Base Address of ROM */

#define PERIPH_BASE				0x40000000U
#define APB_BASE				PERIPH_BASE
#define AHB_BASE				0x40020000U

#define GPIO_BASE				0x50000000U
#define GPIOA_BASE				GPIO_BASE
#define GPIOB_BASE				(GPIO_BASE + 0x0400U)
#define GPIOC_BASE				(GPIO_BASE + 0x0800U)
#define GPIOD_BASE				(GPIO_BASE + 0x0C00U)
#define GPIOE_BASE				(GPIO_BASE + 0x1000U)
#define GPIOF_BASE				(GPIO_BASE + 0x1400U)

// Base Addresses for APB Peripherals
#define I2C1_BASE				(APB_BASE + 0x5400U)
#define I2C2_BASE				(APB_BASE + 0x5800U)
#define I2C3_BASE				(APB_BASE + 0x8800U)

#define SPI1_BASE				(APB_BASE + 0x13000U)
#define SPI2_BASE				(APB_BASE + 0x3800U)
#define SPI3_BASE				(APB_BASE + 0x3C00U)

#define USART1_BASE				(APB_BASE + 0x13800U)
#define USART2_BASE				(APB_BASE + 0x4400U)
#define USART3_BASE				(APB_BASE + 0x4800U)
#define USART4_BASE				(APB_BASE + 0x4C00U)
#define USART5_BASE				(APB_BASE + 0x5000U)
#define USART6_BASE				(APB_BASE + 0x13C00U)

#define SYSCFG_BASE				(APB_BASE + 0x10000U)

// Base Addresses for AHB Peripherals
#define RCC_BASE				(AHB_BASE + 0x1000U)
#define EXTI_BASE				(AHB_BASE + 0x1800U)

/************************** Peripheral Register Definition Structs **************************/

// GPIO Register Definition
typedef struct {
	__vol uint32_t MODER;
	__vol uint32_t OTYPER;
	__vol uint32_t OSPEEDR;
	__vol uint32_t PUPDR;
	__vol uint32_t IDR;
	__vol uint32_t ODR;
	__vol uint32_t BSRR;
	__vol uint32_t LCKR;
	__vol uint32_t AFRL;
	__vol uint32_t AFRH;
	__vol uint32_t BRR;
}GPIO_RegDef_t;

// RCC Register Definition
typedef struct{
	__vol uint32_t RCC_CR;
	__vol uint32_t RCC_ICSCR;
	__vol uint32_t RCC_CFGR;
	__vol uint32_t RCC_PLLCFGR;
	uint32_t RESERVED0;
	uint32_t RESERVED1;
	__vol uint32_t RCC_CIER;
	__vol uint32_t RCC_CIFR;
	__vol uint32_t RCC_CICR;
	__vol uint32_t RCC_IOPRSTR;
	__vol uint32_t RCC_AHBRSTR;
	__vol uint32_t RCC_APBRSTR1;
	__vol uint32_t RCC_APBRSTR2;
	__vol uint32_t RCC_IOPENR;
	__vol uint32_t RCC_AHBENR;
	__vol uint32_t RCC_APBENR1;
	__vol uint32_t RCC_APBENR2;
	__vol uint32_t RCC_IOPSMENR;
	__vol uint32_t RCC_AHBSMENR;
	__vol uint32_t RCC_APBSMENR1;
	__vol uint32_t RCC_APBSMENR2;
	__vol uint32_t RCC_CCIPR;
	__vol uint32_t RCC_CCIPR2;
	__vol uint32_t RCC_BDCR;
	__vol uint32_t RCC_CSR;
}RCC_RegDef_t;

// EXTI Register Definition
typedef struct{
	__vol uint32_t EXTI_RTSR1;
	__vol uint32_t EXTI_FTSR1;
	__vol uint32_t EXTI_SWIER1;
	__vol uint32_t EXTI_RPR1;
	__vol uint32_t EXTI_FPR1;
	uint32_t RESERVED1;				//0x014
	uint32_t RESERVED2;				//0x018
	uint32_t RESERVED3;				//0x01C
	uint32_t RESERVED4;				//0x020
	uint32_t RESERVED5;				//0x024
	uint32_t RESERVED6;				//0x028
	uint32_t RESERVED7;				//0x02C
	uint32_t RESERVED8;				//0x030
	uint32_t RESERVED9;				//0x034
	uint32_t RESERVED10;			//0x038
	uint32_t RESERVED11;			//0x03C
	uint32_t RESERVED12;			//0x040
	uint32_t RESERVED13;			//0x044
	uint32_t RESERVED14;			//0x048
	uint32_t RESERVED15;			//0x04C
	uint32_t RESERVED16;			//0x050
	uint32_t RESERVED17;			//0x054
	uint32_t RESERVED18;			//0x058
	uint32_t RESERVED19;			//0x05C
	__vol uint32_t EXTI_EXTICR[4];
	uint32_t RESERVED20;			//0x070
	uint32_t RESERVED21;			//0x074
	uint32_t RESERVED22;			//0x078
	uint32_t RESERVED23;			//0x07C
	__vol uint32_t EXTI_IMR1;
	__vol uint32_t EXTI_EMR1;
	uint32_t RESERVED24;			//0x088
	uint32_t RESERVED25;			//0x08C
}EXTI_RegDef_t;

/* ***************************peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t) *************************** */

#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASE)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASE)
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASE)
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASE)
#define GPIOE			((GPIO_RegDef_t*) GPIOE_BASE)
#define GPIOF			((GPIO_RegDef_t*) GPIOF_BASE)

#define RCC				((RCC_RegDef_t*)RCC_BASE)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASE)

/* ************************************* CLK ENABLE MACROS ************************************* */

//Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PCLK_EN() (RCC->RCC_IOPENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->RCC_IOPENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->RCC_IOPENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->RCC_IOPENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->RCC_IOPENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->RCC_IOPENR |= (1<<5))

//Clock Enable Macros for I2Cx Peripherals
#define I2C1_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<21))
#define I2C2_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<22))
#define I2C3_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<23))

//Clock Enable Macros for SPIx Peripherals
#define SPI1_PCLK_EN() (RCC->RCC_APBENR2 |= (1<<12))
#define SPI2_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<14))
#define SPI3_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<15))

//Clock Enable Macros for USARTx Peripherals
#define USART1_PCLK_EN() (RCC->RCC_APBENR2 |= (1<<14))
#define USART2_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<17))
#define USART3_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<18))
#define USART4_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<19))
#define USART5_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<8))
#define USART6_PCLK_EN() (RCC->RCC_APBENR1 |= (1<<9))

//Clock Enable Macros for Syscfg Peripherals
#define SYSCFG_PCLK_EN() (RCC->RCC_APBENR2 |= (1<<0))

/* ************************************* CLK DISABLE MACROS ************************************* */

//Clock Disable Macros for GPIOx Peripherals
#define GPIOA_PCLK_DI() (RCC->RCC_IOPENR &= ~(1<<0))
#define GPIOB_PCLK_DI() (RCC->RCC_IOPENR &= ~(1<<1))
#define GPIOC_PCLK_DI() (RCC->RCC_IOPENR &= ~(1<<2))
#define GPIOD_PCLK_DI() (RCC->RCC_IOPENR &= ~(1<<3))
#define GPIOE_PCLK_DI() (RCC->RCC_IOPENR &= ~(1<<4))
#define GPIOF_PCLK_DI() (RCC->RCC_IOPENR &= ~(1<<5))

//Clock Disable Macros for I2Cx Peripherals
#define I2C1_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<21))
#define I2C2_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<22))
#define I2C3_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<23))

//Clock Disable Macros for SPIx Peripherals
#define SPI1_PCLK_DI() (RCC->RCC_APBENR2 &= ~(1<<12))
#define SPI2_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<14))
#define SPI3_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<15))

//Clock Disable Macros for USARTx Peripherals
#define USART1_PCLK_DI() (RCC->RCC_APBENR2 &= ~(1<<14))
#define USART2_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<18))
#define USART4_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<19))
#define USART5_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<8))
#define USART6_PCLK_DI() (RCC->RCC_APBENR1 &= ~(1<<9))

//Clock Disable Macros for Syscfg Peripherals
#define SYSCFG_PCLK_DI() (RCC->RCC_APBENR2 &= ~(1<<0))

/* ******************************* MACROS TO RESET GPIOx PERIPHERALS ******************************* */
#define GPIOA_REG_RESET() do{(RCC->RCC_IOPRSTR |= (1 << 0)); (RCC->RCC_IOPRSTR &= ~ (1 << 0));} while(0)
#define GPIOB_REG_RESET() do{(RCC->RCC_IOPRSTR |= (1 << 1)); (RCC->RCC_IOPRSTR &= ~ (1 << 1));} while(0)
#define GPIOC_REG_RESET() do{(RCC->RCC_IOPRSTR |= (1 << 2)); (RCC->RCC_IOPRSTR &= ~ (1 << 2));} while(0)
#define GPIOD_REG_RESET() do{(RCC->RCC_IOPRSTR |= (1 << 3)); (RCC->RCC_IOPRSTR &= ~ (1 << 3));} while(0)
#define GPIOE_REG_RESET() do{(RCC->RCC_IOPRSTR |= (1 << 4)); (RCC->RCC_IOPRSTR &= ~ (1 << 4));} while(0)
#define GPIOF_REG_RESET() do{(RCC->RCC_IOPRSTR |= (1 << 5)); (RCC->RCC_IOPRSTR &= ~ (1 << 5));} while(0)


/* ************************************* GENERIC MACROS ************************************* */
#define ENABLE				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32G070XX_H_ */
